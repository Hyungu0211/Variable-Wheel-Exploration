CC ?= cc
CFLAGS ?= -std=c11 -O2 -Wall -Wextra -Wpedantic
CPPFLAGS += -D_POSIX_C_SOURCE=200809L
LDLIBS += -pthread -lrt -lm
SRC = c/robot.c c/policy.c c/ipc.c c/devices.c
HEADERS = c/platform.h c/ipc.h c/policy.h c/device.h
.PHONY: all hardware test
all: build/robot build/libpipeipc.so
build:
	mkdir -p build
build/robot: $(SRC) $(HEADERS) | build
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SRC) -o $@ $(LDLIBS)
build/libpipeipc.so: c/ipc.c c/ipc.h c/platform.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) -fPIC -shared c/ipc.c -o $@ $(LDLIBS)
DXL_INC ?= /usr/local/include/dynamixel_sdk_c
DXL_LIB ?= -ldxl_c
hardware: build/robot-hw build/libpipeipc.so
build/robot-hw: $(SRC) $(HEADERS) | build
	$(CC) $(CPPFLAGS) $(CFLAGS) -D_XOPEN_SOURCE=700 -DPIPE_HARDWARE -I$(DXL_INC) $(SRC) -o $@ $(DXL_LIB) $(LDLIBS)
test: all
	python3 tests/integration.py
