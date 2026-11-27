# Raspberry Pi 5용 배관 탐사 로봇 B안

Python이 Pi 5의 카메라 영상에서 균열·퇴적물·이물질을 찾고, C가 거리·자세·조종 명령과 결합해 바퀴를 제어한다. 기본 실행은 모의 주행이다. 아래 설치 예시는 Raspberry Pi OS **Bookworm 64비트**를 기준으로 한다.

### Pi 4와 Pi 5의 차이

| 구성 | Pi 5에서 할 일 |
|---|---|
| C 제어와 프로세스 간 통신 | Pi 5에서 `make`로 제어기와 `libpipeipc.so`를 함께 빌드한다. 스레드 4개, Mutex, 공유 메모리, UDP 조종 방식은 공통이다. |
| AX-12A 구동 모터 | 기존 USB–TTL 반이중 어댑터(예: U2D2의 3핀 TTL 포트)를 사용한다. `ls -l /dev/serial/by-id/`로 어댑터를 식별하고 해당 전체 경로를 `DXL_PORT`에 넣는다. by-id 항목이 없으면 어댑터를 뺐다 꽂아 확인한 `/dev/ttyUSB0` 등의 경로를 쓴다. 이 구성에서는 GPIO UART 설정이 필요 없다. 모터 전원은 별도로 공급한다. |
| PCA9685·거리·자세 센서 | `sudo raspi-config`의 Interface Options에서 I2C를 켠다. 기본 헤더에서 SDA는 GPIO2(물리 핀 3), SCL은 GPIO3(핀 5)이다. `i2cdetect -l`로 버스 목록을 확인하고, ToF·IMU 드라이버가 실제로 노출한 측정 파일을 지정한다. |
| Camera Module 3 | 전원을 끈 상태에서 카메라 쪽 15핀과 Pi 5 CAM/DISP 쪽 22핀을 잇는 Standard–Mini **카메라용** 케이블을 연결한다. `--list-cameras`에서 보이는 번호를 `--camera`에 준다. 한 대만 인식되면 일반적으로 0이다. 번호는 물리 커넥터 라벨과 별개다. |
| 전원 | Pi 5 권장 전원은 5 V·5 A USB-C다. 기존 24 V 테더와 DC-DC 변환기를 쓴다면 Pi 입력 전압과 공급 가능 전류를 카메라·모터 동작 중 확인한다. |

## 1. 부품 없이 실행하기

압축을 푼 뒤 `Makefile`이 있는 디렉터리에서 실행한다. Python 3.9 이상을 사용한다. 모의 실행은 Python 표준 라이브러리만 사용한다.

**Raspberry Pi OS / Linux**

~~~sh
sudo apt install build-essential
make
make test
~~~

**Windows 10/11, Visual Studio 2022**

시작 메뉴에서 **x64 Native Tools Command Prompt for VS 2022**를 연다. C++ 데스크톱 개발 도구와 64비트 Python이 필요하다.

~~~bat
build_windows.cmd
python tests\integration.py
~~~

통합 시험은 C 제어기와 Python 발행기를 실행해 동일 프레임 반복, 서로 다른 두 프레임, 감속·정지, 재개, 전후진, 휠 조절, 장애물, AI 갱신 중단, UDP 단절을 확인한다. 시험을 진행할 때는 수동으로 실행한 제어기와 발행기를 먼저 종료한다.

**직접 조종하기** — 터미널 세 개를 같은 폴더에서 연다.

| 터미널 | Pi / Linux | Windows |
|---|---|---|
| 1. C 제어 | `./build/robot` | `build\robot.exe` |
| 2. Python 검출 | `python3 py/vision.py --scene clear` | `python py\vision.py --scene clear` |
| 3. 지상국 | `python3 py/station.py` | `python py\station.py` |

1. 지상국에서 `status`를 입력한다. 정상 영상과 거리값이 들어오면 `DISARMED` 상태가 된다.
2. `arm`을 입력한 후 `status`에서 `READY`를 확인하고 `drive 40`을 입력한다. `speed`가 40이면 전진 출력을 지시한 것이다. 응답은 명령 수신 시점의 상태이므로 제어 반영 여부는 다음 `status`에서 확인한다.
3. `simdist 100`을 입력하면 거리 100 mm에서 `OBSTACLE`로 정지한다. `simdist 1000`으로 복구해도 정지를 유지한다. `arm`을 다시 입력해야 한다.
4. `wheel 250`은 250 mm **시험관용 프리셋**이다. 정지 확인 후 적용하고 설정된 서보 이동 시간 동안 주행 출력을 0으로 유지한다. `wheel` 상태값은 마지막 전송 프리셋이며 실제 휠 직경 측정값은 아니다.
5. 지상국을 `quit`로 종료한다. 다른 두 프로그램은 Ctrl+C로 종료한다.

**결함 장면을 바꾸는 시연** — 텍스트 파일 `scene.txt`를 만들고 `clear` 한 단어를 넣는다. 터미널 2에서 `python3 py/vision.py --scene-file scene.txt --fps 2`를 실행한다. 주행 중 파일 내용을 `foreign`, `deposit`, `crack` 중 하나로 바꾸면 첫 검출에서 감속하고 같은 종류가 두 프레임 연속 확인되면 정지한다. `clear`로 바꾼 다음 지상국에서 `arm`을 입력하면 재개할 수 있다. Windows에서는 파일을 UTF-8로 저장하고 `python`을 사용한다.

## 2. 코드에서 Mutex 찾기

| 파일 | 맡은 기능 |
|---|---|
| `c/robot.c` | C 스레드 4개, 상태·명령 공유, UDP 지상국 통신 |
| `c/policy.c` | 워치독, 결함 반복 확인, 정지 우선순위와 재개 조건 |
| `c/ipc.c` | 최신 검출 결과 1개를 공유 메모리에서 읽고 쓰는 C 인터페이스 |
| `c/devices.c` | AX-12A 통신, PCA9685 출력, ToF·IMU 값 읽기 |
| `py/vision.py` | Python 모의 발행기, Pi 카메라 입력, YOLO 추론 및 결함 기록 |
| `py/station.py` | 노트북 조종 및 상태 조회. 0.1초 간격으로 연결 확인 메시지 전송 |
| `py/preview.py` | 브라우저로 최신 검출 영상을 확인하는 선택 기능 |
| `py/train_export.py` | 팀 데이터로 YOLO 학습 후 NCNN 형식으로 변환 |

C 제어 루프의 목표 주기는 20 ms(50 Hz)다. 센서·서보 스레드가 I2C 장치를, 모터 스레드가 AX 시리얼 포트를 전담한다. 주기는 하드웨어와 AI 부하에 따라 달라진다.

| C Mutex | 기록하는 쪽 | 읽는 쪽 | 함께 보호하는 값 |
|---|---|---|---|
| `sensor_mutex` | 센서 스레드 | 제어·통신 스레드 | 전방 거리, 기울기, 측정 시각, 유효 여부 |
| `command_mutex` | 통신 스레드 | 제어 스레드 | 속도, 휠 프리셋, 수신 시각, 정지·재개 요청 번호 |
| `actuator_mutex` | 제어·모터·센서 스레드 | 모터·센서·통신·제어 스레드 | 최종 출력, 모터 정지 확인, 서보 이동 상태, 상태 보고용 복사본 |

예를 들어 `sensors()`는 잠금 밖에서 측정한 뒤 `sensor_mutex`를 잡고 구조체 전체를 교체한다. `control()`도 같은 Mutex를 잡고 구조체를 지역 변수에 복사한다. 제어 스레드는 거리와 측정 시각이 같은 갱신에서 나온 값임을 보장받고, 잠금을 풀고 판단한다. `run_mutex`는 종료 여부만 보호한다.

Python 프로세스는 `ctypes`로 작은 C 라이브러리를 호출한다. Pi에서는 `shm_open`과 `mmap`으로 공유 공간을 열고 초기값 1인 네임드 세마포어로 48바이트 결과 하나를 보호한다. 정상 프레임도 기록한다. C는 `sem_trywait`로 최신 결과를 복사하며 잠금 대기가 길어져도 제어 루프를 계속 돈다. Windows에서는 같은 데이터 형식을 Windows 공유 메모리·세마포어와 배타적 SRW 잠금으로 시험한다.

## 3. 주행 판단 기준

아래 수치는 시제품 초기 설정이다.

| 입력 조건 | 출력 |
|---|---|
| 유효한 조종 패킷이 500 ms 넘게 끊김 | CMD_TIMEOUT, 정지 |
| 센서값 무효 또는 측정 후 300 ms 초과 | SENSOR_STALE, 정지 |
| 전방 거리가 설정한 기준 이하 | OBSTACLE, 정지 |
| AI 프레임이 없거나 촬영 시작 후 1,500 ms 초과 | AI_STALE, 정지 |
| 신뢰도 0.60 이상 결함 1프레임 | 요청 출력의 50%로 감속 |
| 같은 종류의 결함이 서로 다른 연속 프레임 2개에서 검출 | 균열은 CRACK, 퇴적물·이물질은 BLOCKAGE로 정지 |
| 두 모터가 200 ms 이상 정지 상태이고 휠 프리셋이 새로 요청됨 | PWM 프리셋 전송, 설정한 이동 시간까지 주행 금지 |

정지 후에는 원인을 해소하고 `arm`을 다시 입력해야 한다. 지상국의 `stop`도 잠금 정지를 요청한다. `drive 0`은 출력만 0으로 바꾼다. 지상국의 통신이 살아 있는 동안 마지막 속도 명령이 유지된다. 실물 구동에서는 `speed`가 방향을 포함한 **구동 출력 명령의 백분율**이다. AX-12A 휠 모드의 출력 제어 특성상 실제 RPM이나 차체 속도를 뜻하지 않는다.

## 4. 카메라와 실제 결함 모델 연결

1. PC의 학습 환경에서 `python -m pip install ultralytics`로 학습 도구를 준비한다. 정상 장면과 3개 결함 클래스를 촬영하고 YOLO 라벨을 작성한다. 촬영 시퀀스 단위로 학습·검증 데이터를 나눈다.
2. `dataset.example.yaml`의 경로를 실제 데이터 폴더로 고친다. 각 `images/train`, `images/val` 이미지에 대응하는 YOLO 라벨은 `labels/train`, `labels/val`에 둔다. 학습용 사진·가중치는 패키지에 포함하지 않았다.
3. `python py/train_export.py dataset.example.yaml --epochs 50 --imgsz 320`으로 학습과 변환을 실행한다. 출력된 NCNN 디렉터리 전체를 Pi로 복사한다.
4. Raspberry Pi OS에서 Camera Module 3를 활성화하고 `rpicam-hello --list-cameras`로 장치를 확인한다. Pi의 Python 가상환경은 시스템의 Picamera2를 읽을 수 있어야 한다. 아래는 준비 예다.

~~~sh
sudo apt install python3-picamera2 python3-opencv python3-venv rpicam-apps i2c-tools
python3 -m venv --system-site-packages .venv
. .venv/bin/activate
python py/vision.py --list-cameras
python -m pip install ultralytics ncnn
python -c "import cv2, ncnn; from picamera2 import Picamera2; from ultralytics import YOLO"
make
python py/vision.py --mode camera --camera 0 --model ./best_ncnn_model --preview ./preview/latest.jpg
~~~

C 제어기는 별도 터미널에서 실행한다. `--preview`를 주면 최신 검출 화면을 JPEG로 갱신한다. `python py/preview.py ./preview/latest.jpg --bind 192.168.10.2` 실행 후 노트북에서 http://192.168.10.2:8080/ 을 열면 된다(예시 주소). 결함 프레임과 시각·신뢰도·영역 좌표는 `events/`에 기록한다. 동일 종류가 계속 검출되면 최소 2초 간격으로 기록한다.

학습 모델 클래스 이름은 `crack`, `deposit`, `foreign`을 사용한다. 이름이 다르면 예를 들어 `{"균열":"crack","퇴적물":"deposit","이물질":"foreign"}`를 JSON 파일로 저장한 뒤 `--labels labels.json`으로 전달한다. 모델이 여러 물체를 검출하면 제어기에는 균열을 우선해 한 건을 보내고 저장 영상에는 전체 검출을 표시한다.

## 5. 실제 로봇 연결

첫 동작 시험은 바퀴를 띄우고 수행한다. 실물 모드는 카메라 발행기의 유효한 프레임을 요구한다. 직접적인 모터·전원 고장은 소프트웨어 워치독으로 정지를 보장할 수 없으므로 실험대에서 전원을 차단할 수 있어야 한다.

- **AX-12A 2개**는 TTL 반이중 USB 어댑터와 ROBOTIS C SDK를 사용한다. 실제 통신 속도, 모터 ID, 정방향 부호와 출력 상한을 입력한다. 두 모터를 Wheel Mode에 설정하고 읽기·쓰기에 대한 Status Return Level을 2로 맞춘다. 프로그램은 두 각도 제한 레지스터가 모두 0인지 확인한다. 오른쪽·왼쪽 모터는 장착 방향에 맞춰 부호를 보정한다.
- **PCA9685**는 전용 보드의 채널 0~3을 사용하고 공칭 25 MHz 클록에서 약 50 Hz PWM을 생성한다. 200/250은 **대응 배관 내경**이다. 휠마다 기구 간섭 없이 해당 내경에 맞는 실제 펄스폭을 재서 8개 값을 채운다. 기구의 정착 시간도 실측해 입력한다. 소프트웨어는 서보 위치를 직접 측정하지 않는다.
- **ToF·IMU**는 원안에 정확한 칩 모델이 없어 Linux IIO/sysfs에 드라이버가 제공하는 수치 파일을 읽도록 구현했다. `/sys/bus/iio/devices/iio:device*/name`에서 장치를 식별하고 거리값 경로, mm 환산계수, 같은 단위인 가속도 3축 경로를 넣는다. IMU 기울기는 센서 X축 기준이며 장착 방향에 맞춰 조정한다. 다른 인터페이스를 사용하는 센서는 `devices_sense()`를 해당 드라이버에 맞춰 바꿔야 한다.
- **지상국**은 한 대만 연결한다. 제어기는 설정한 노트북 IP의 UDP만 수신한다. 실습용 전용 이더넷에서 사용한다.

DYNAMIXEL C SDK는 Pi에서 빌드한다. 공식 SDK의 ARM용 Makefile을 사용하는 예시는 다음과 같다. 프로젝트 폴더에서 실행한다. 이미 CMake 방식으로 `/usr/local/include/dynamixel_sdk_c`와 `libdxl_c.so`를 설치했다면 `make hardware`로 바로 빌드할 수 있다.

~~~sh
sudo apt install git build-essential
git clone --depth 1 https://github.com/ROBOTIS-GIT/DynamixelSDK.git ../DynamixelSDK
make -C ../DynamixelSDK/c/build/linux_sbc
sudo make -C ../DynamixelSDK/c/build/linux_sbc install
sudo ldconfig
make hardware DXL_INC=../DynamixelSDK/c/include/dynamixel_sdk DXL_LIB=-ldxl_sbc_c
~~~

`--check-hardware`는 시리얼·I²C 장치 경로의 접근 권한을 확인하고 ToF·IMU를 한 번 읽는다. 모터나 서보에 명령을 보내지 않으므로 이 단계에서 모터 응답이나 서보 동작까지 확인되지는 않는다. 이 점검에는 장치 경로와 거리 환산계수만 필요하다. 주행을 시작할 때는 모터 설정과 채널별 서보 보정값도 채워야 한다.

~~~sh
cp hardware.env.example hardware.env
nano hardware.env
set -a
. ./hardware.env
set +a
./build/robot-hw --check-hardware
# 예시 IP. 마지막 두 인수는 시험으로 정한 정지 거리(mm)와 서보 이동 대기 시간(ms).
./build/robot-hw --hardware 192.168.10.2 192.168.10.1 43771 180 1800
~~~

Pi의 다른 터미널에서는 위 카메라 발행기를 실행한다. 노트북에서는 `python py/station.py --host 192.168.10.2`로 접속한다. 두 프로세스는 Pi에서 같은 사용자로 실행한다. 네임드 세마포어를 소유한 프로세스가 비정상 종료되어 결과가 계속 멈춰 있으면 제어기를 정지한 뒤 다시 실행해 IPC 객체를 재생성한다.

## 검증 범위와 참고 자료

Windows x64에서 C 제어기와 IPC DLL을 다시 빌드하고 Python과 UDP·공유 메모리로 연결해 모의 주행 통합 시험을 통과했다. 카메라 번호 선택·프레임 전달과 장치 경로·센서값 점검은 모의 장치로 시험했다. 실제 Pi 5에서의 SDK 빌드, 카메라 촬영·추론 속도, 모터 통신, 제동 거리와 서보 보정은 팀 장비에서 확인해야 한다.

- [ROBOTIS AX-12A 제어 테이블](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/)
- [DYNAMIXEL C SDK 설치](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/c/)
- [Raspberry Pi용 YOLO와 NCNN](https://docs.ultralytics.com/guides/raspberry-pi/)
- [Pi 5 사양과 전원](https://www.raspberrypi.com/products/raspberry-pi-5/)
- [Pi 5 카메라 케이블](https://www.raspberrypi.com/products/camera-cable/)
- [ROBOTIS U2D2 연결 및 별도 전원](https://emanual.robotis.com/docs/en/parts/interface/u2d2/)
- [NXP PCA9685 데이터시트](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf)
