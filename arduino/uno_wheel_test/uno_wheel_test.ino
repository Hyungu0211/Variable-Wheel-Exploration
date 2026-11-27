#define _DEFAULT_SOURCE

#include <lgpio.h>

#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

/* =====================================================
 * Raspberry Pi 5 / Raspberry Pi OS
 * 가변 휠 캠 구동 시험 - 연속회전 서보 1개
 *
 * GPIO signal : BCM GPIO18 (40-pin header physical pin 12)
 * PWM         : lgpio software-timed servo, 50 Hz
 * Input       : keyboard in the terminal
 * ===================================================== */

#define GPIO_CHIP_PRIMARY  0   /* Recent Raspberry Pi OS: /dev/gpiochip0 */
#define GPIO_CHIP_LEGACY   4   /* Some older Pi 5 images: /dev/gpiochip4 */
#define SERVO_GPIO         18  /* BCM GPIO18, physical pin 12 */

#define SERVO_FREQ_HZ      50
#define SERVO_MIN_US       1000
#define SERVO_MAX_US       2000
#define MAX_RUN_MS         20000ULL
#define LOOP_WAIT_MS       10

#define EXPAND_DIR         1   /* If first movement contracts the wheel, change to -1 */

static int gpio_handle = -1;
static int gpio_chip_number = -1;
static int gpio_claimed = 0;
static int stop_us = 1500;
static int speed_us = 200;

static volatile sig_atomic_t running = 1;
static struct termios saved_termios;
static int terminal_was_changed = 0;

typedef enum {
    CONTRACTED,
    EXPANDING,
    EXPANDED,
    CONTRACTING
} State;

static State state = CONTRACTED;
static uint64_t t_start_ms = 0;
static uint64_t t_expand_ms = 0;

static const char *state_name(State s)
{
    switch (s) {
        case CONTRACTED:  return "수축(대기)";
        case EXPANDING:   return "확장 중";
        case EXPANDED:    return "확장(멈춤)";
        case CONTRACTING: return "수축 중";
        default:          return "알 수 없음";
    }
}

static uint64_t monotonic_ms(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        perror("clock_gettime");
        return 0;
    }

    return (uint64_t)ts.tv_sec * 1000ULL +
           (uint64_t)ts.tv_nsec / 1000000ULL;
}

static void signal_handler(int signum)
{
    (void)signum;
    running = 0;
}

static void install_signal_handlers(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
}

static void enable_terminal_key_mode(void)
{
    struct termios raw;

    if (!isatty(STDIN_FILENO)) {
        return;
    }

    if (tcgetattr(STDIN_FILENO, &saved_termios) != 0) {
        perror("tcgetattr");
        return;
    }

    raw = saved_termios;
    cfmakeraw(&raw);  /* key is delivered immediately, without Enter */

    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) == 0) {
        terminal_was_changed = 1;
    } else {
        perror("tcsetattr");
    }
}

static void restore_terminal(void)
{
    if (terminal_was_changed) {
        (void)tcsetattr(STDIN_FILENO, TCSAFLUSH, &saved_termios);
        terminal_was_changed = 0;
    }
}

/* Detect the Raspberry Pi 5 RP1 GPIO chip.
 * Newer images normally expose it as gpiochip0; some older images used gpiochip4.
 */
static int open_rp1_gpiochip(void)
{
    const int candidates[] = { GPIO_CHIP_PRIMARY, GPIO_CHIP_LEGACY };
    size_t i;

    for (i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
        int h = lgGpiochipOpen(candidates[i]);
        lgChipInfo_t info;

        if (h < 0) {
            continue;
        }

        if (lgGpioGetChipInfo(h, &info) == 0 &&
            (strstr(info.label, "pinctrl-rp1") != NULL ||
             strstr(info.name, "pinctrl-rp1") != NULL)) {
            gpio_chip_number = candidates[i];
            return h;
        }

        (void)lgGpiochipClose(h);
    }

    return -1;
}

static int servo_set_pulse_us(int pulse_us)
{
    int rc;

    if (pulse_us < SERVO_MIN_US) pulse_us = SERVO_MIN_US;
    if (pulse_us > SERVO_MAX_US) pulse_us = SERVO_MAX_US;

    /* pulseWidth=0 means stop transmitting; servoCycles=0 means continuous. */
    rc = lgTxServo(gpio_handle, SERVO_GPIO, pulse_us,
                   SERVO_FREQ_HZ, 0, 0);

    if (rc < 0) {
        fprintf(stderr, "서보 PWM 오류: %s (%d)\n", lguErrorText(rc), rc);
        return -1;
    }

    return 0;
}

static int servo_init(void)
{
    int rc;

    gpio_handle = open_rp1_gpiochip();
    if (gpio_handle < 0) {
        fprintf(stderr,
                "RP1 GPIO 칩을 열지 못했습니다.\n"
                "터미널에서 'gpiodetect'를 실행해 gpiochip 정보를 확인하세요.\n"
                "GPIO 접근 권한도 확인하세요.\n");
        return -1;
    }

    rc = lgGpioClaimOutput(gpio_handle, 0, SERVO_GPIO, 0);
    if (rc < 0) {
        fprintf(stderr, "GPIO%d 출력 점유 실패: %s (%d)\n",
                SERVO_GPIO, lguErrorText(rc), rc);
        (void)lgGpiochipClose(gpio_handle);
        gpio_handle = -1;
        return -1;
    }

    gpio_claimed = 1;
    printf("GPIO chip: gpiochip%d (RP1), servo signal: BCM GPIO%d\n",
           gpio_chip_number, SERVO_GPIO);

    if (servo_set_pulse_us(stop_us) != 0) {
        return -1;
    }

    return 0;
}

static void servo_cleanup(void)
{
    if (gpio_handle < 0) {
        return;
    }

    /* Send neutral pulse briefly before stopping the pulse stream. */
    if (gpio_claimed) {
        (void)servo_set_pulse_us(stop_us);
        (void)lguSleep(0.15);
        (void)lgTxServo(gpio_handle, SERVO_GPIO, 0, SERVO_FREQ_HZ, 0, 0);
        (void)lgGpioFree(gpio_handle, SERVO_GPIO);
        gpio_claimed = 0;
    }

    (void)lgGpiochipClose(gpio_handle);
    gpio_handle = -1;
}

static void drive(int direction)
{
    int pulse = stop_us + direction * EXPAND_DIR * speed_us;
    (void)servo_set_pulse_us(pulse);
}

static void stop_servo(void)
{
    (void)servo_set_pulse_us(stop_us);
}

static void report_state(void)
{
    printf("상태=%s  확장시간=%llu ms  속도=%d us  정지=%d us\n",
           state_name(state),
           (unsigned long long)t_expand_ms,
           speed_us,
           stop_us);
    fflush(stdout);
}

static void on_space(void)
{
    switch (state) {
        case CONTRACTED:
            state = EXPANDING;
            t_start_ms = monotonic_ms();
            drive(+1);
            puts("▶ 확장 시작 (스페이스로 멈춤)");
            break;

        case EXPANDING:
            t_expand_ms = monotonic_ms() - t_start_ms;
            stop_servo();
            state = EXPANDED;
            printf("■ 확장 멈춤 %llu ms (스페이스로 수축)\n",
                   (unsigned long long)t_expand_ms);
            break;

        case EXPANDED:
            state = CONTRACTING;
            t_start_ms = monotonic_ms();
            drive(-1);
            printf("◀ 수축 %llu ms 역회전\n",
                   (unsigned long long)t_expand_ms);
            break;

        case CONTRACTING:
            stop_servo();
            state = CONTRACTED;
            puts("■ 수축 중단 → 대기 (위치는 어긋났을 수 있음, x로 기준 재설정)");
            break;
    }

    fflush(stdout);
}

static void process_key(char c)
{
    switch (c) {
        case ' ':
        case 'e':
        case 'E':
        case '\r':
        case '\n':
            on_space();
            break;

        case 'x':
        case 'X':
            stop_servo();
            state = CONTRACTED;
            t_expand_ms = 0;
            puts("정지·초기화 (현재 위치를 수축 기준으로 간주)");
            fflush(stdout);
            break;

        case '+':
        case '=':
            speed_us += 50;
            if (speed_us > 500) speed_us = 500;
            if (state == EXPANDING) drive(+1);
            if (state == CONTRACTING) drive(-1);
            report_state();
            break;

        case '-':
        case '_':
            speed_us -= 50;
            if (speed_us < 50) speed_us = 50;
            if (state == EXPANDING) drive(+1);
            if (state == CONTRACTING) drive(-1);
            report_state();
            break;

        case '[':
            stop_us -= 10;
            if (stop_us < SERVO_MIN_US) stop_us = SERVO_MIN_US;
            if (state == CONTRACTED || state == EXPANDED) stop_servo();
            report_state();
            break;

        case ']':
            stop_us += 10;
            if (stop_us > SERVO_MAX_US) stop_us = SERVO_MAX_US;
            if (state == CONTRACTED || state == EXPANDED) stop_servo();
            report_state();
            break;

        case '?':
            report_state();
            break;

        default:
            break;
    }
}

static void check_motion_timeouts(void)
{
    uint64_t now = monotonic_ms();

    /* Contract for the same elapsed time recorded during expansion. */
    if (state == CONTRACTING && (now - t_start_ms) >= t_expand_ms) {
        stop_servo();
        state = CONTRACTED;
        puts("■ 수축 완료 → 대기");
        fflush(stdout);
    }

    /* Safety timeout for expansion. */
    if (state == EXPANDING && (now - t_start_ms) >= MAX_RUN_MS) {
        t_expand_ms = MAX_RUN_MS;
        stop_servo();
        state = EXPANDED;
        puts("! 20 s 초과 자동 정지");
        fflush(stdout);
    }
}

static void read_keyboard_nonblocking(void)
{
    fd_set read_fds;
    struct timeval timeout;
    int ready;
    char buffer[64];
    ssize_t count;
    ssize_t i;

    FD_ZERO(&read_fds);
    FD_SET(STDIN_FILENO, &read_fds);

    timeout.tv_sec = 0;
    timeout.tv_usec = LOOP_WAIT_MS * 1000;

    ready = select(STDIN_FILENO + 1, &read_fds, NULL, NULL, &timeout);
    if (ready < 0) {
        if (errno != EINTR) perror("select");
        return;
    }

    if (ready == 0 || !FD_ISSET(STDIN_FILENO, &read_fds)) {
        return;
    }

    count = read(STDIN_FILENO, buffer, sizeof(buffer));
    if (count == 0) {
        running = 0;
        return;
    }
    if (count < 0) {
        if (errno != EINTR && errno != EAGAIN) perror("read");
        return;
    }

    for (i = 0; i < count; ++i) {
        process_key(buffer[i]);
    }
}

int main(void)
{
    puts("가변 휠 캠 시험 - Raspberry Pi 5 / C");
    puts("스페이스: 확장→멈춤→수축");
    puts("e: 스페이스와 동일, x: 정지/초기화");
    puts("+/-: 회전 속도 조절, [/]: 정지 펄스 보정, ?: 상태 출력");
    puts("Ctrl+C: 프로그램 종료");

    install_signal_handlers();

    if (servo_init() != 0) {
        servo_cleanup();
        return EXIT_FAILURE;
    }

    enable_terminal_key_mode();
    report_state();

    while (running) {
        check_motion_timeouts();
        read_keyboard_nonblocking();
    }

    /* Stop the servo and restore the user's terminal settings. */
    stop_servo();
    (void)lguSleep(0.15);
    restore_terminal();
    servo_cleanup();

    puts("프로그램 종료");  
    return EXIT_SUCCESS;
}
