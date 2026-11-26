#include "ipc.h"
#include <stdio.h>
#include <string.h>
_Static_assert(sizeof(Detection) == 48, "IPC layout mismatch");
API uint64_t ipc_clock_ms(void) {
    return now_ms();
}

#ifdef _WIN32
static HANDLE map, gate;
static Detection *area;
static const char *mapname = "Local\\PipeBotDetectionV1", *semname = "Local\\PipeBotGateV1";
int ipc_start(void) {
    map = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, sizeof(Detection),
                             mapname);
    if (!map || GetLastError() == ERROR_ALREADY_EXISTS)
        return -1;
    gate = CreateSemaphoreA(NULL, 1, 1, semname);
    area = MapViewOfFile(map, FILE_MAP_ALL_ACCESS, 0, 0, sizeof *area);
    return gate && area ? 0 : -1;
}
bool ipc_snapshot(Detection *out) {
    if (WaitForSingleObject(gate, 0) != WAIT_OBJECT_0)
        return false;
    *out = *area;
    ReleaseSemaphore(gate, 1, NULL);
    return true;
}
void ipc_close(void) {
    if (area)
        UnmapViewOfFile(area);
    if (gate)
        CloseHandle(gate);
    if (map)
        CloseHandle(map);
}
API int ipc_publish(const Detection *v) {
    HANDLE m = OpenFileMappingA(FILE_MAP_WRITE, FALSE, mapname);
    if (!m)
        return -1;
    HANDLE s = OpenSemaphoreA(SYNCHRONIZE | SEMAPHORE_MODIFY_STATE, FALSE, semname);
    Detection *dst = MapViewOfFile(m, FILE_MAP_WRITE, 0, 0, sizeof *dst);
    int result = -1;
    if (s && dst && WaitForSingleObject(s, 5) == WAIT_OBJECT_0) {
        *dst = *v;
        ReleaseSemaphore(s, 1, NULL);
        result = 0;
    }
    if (dst)
        UnmapViewOfFile(dst);
    if (s)
        CloseHandle(s);
    CloseHandle(m);
    return result;
}
#else
#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <stdlib.h>
static int fd = -1, owner = -1;
static sem_t *gate = SEM_FAILED;
static Detection *area = MAP_FAILED;
static void names(char *m, char *s) {
    snprintf(m, 64, "/pipebot_%lu_ai_v1", (unsigned long)getuid());
    snprintf(s, 64, "/pipebot_%lu_sem_v1", (unsigned long)getuid());
}
int ipc_start(void) {
    char m[64], s[64], lock[96];
    names(m, s);
    snprintf(lock, sizeof lock, "/dev/shm/pipebot_%lu_owner.lock", (unsigned long)getuid());
    owner = open(lock, O_CREAT | O_RDWR | O_NOFOLLOW, 0600);
    if (owner < 0 || flock(owner, LOCK_EX | LOCK_NB) < 0)
        return -1;
    shm_unlink(m);
    sem_unlink(s);
    fd = shm_open(m, O_CREAT | O_EXCL | O_RDWR, 0600);
    if (fd < 0 || ftruncate(fd, sizeof(Detection)) < 0)
        return -1;
    area = mmap(NULL, sizeof *area, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    gate = sem_open(s, O_CREAT | O_EXCL, 0600, 1);
    return area != MAP_FAILED && gate != SEM_FAILED ? 0 : -1;
}
bool ipc_snapshot(Detection *out) {
    if (sem_trywait(gate) != 0)
        return false;
    *out = *area;
    sem_post(gate);
    return true;
}
void ipc_close(void) {
    char m[64], s[64];
    names(m, s);
    sem_unlink(s);
    shm_unlink(m);
    if (area != MAP_FAILED)
        munmap(area, sizeof *area);
    if (gate != SEM_FAILED)
        sem_close(gate);
    if (fd >= 0)
        close(fd);
    if (owner >= 0)
        close(owner);
}
API int ipc_publish(const Detection *v) {
    char m[64], s[64];
    names(m, s);
    int f = shm_open(m, O_RDWR, 0);
    if (f < 0)
        return -1;
    struct stat st;
    if (fstat(f, &st) < 0 || st.st_size != (off_t)sizeof *v) {
        close(f);
        return -1;
    }
    sem_t *g = sem_open(s, 0);
    Detection *dst = mmap(NULL, sizeof *dst, PROT_WRITE | PROT_READ, MAP_SHARED, f, 0);
    int result = -1;
    if (g != SEM_FAILED && dst != MAP_FAILED && sem_trywait(g) == 0) {
        *dst = *v;
        sem_post(g);
        result = 0;
    }
    if (dst != MAP_FAILED)
        munmap(dst, sizeof *dst);
    if (g != SEM_FAILED)
        sem_close(g);
    close(f);
    return result;
}
#endif
