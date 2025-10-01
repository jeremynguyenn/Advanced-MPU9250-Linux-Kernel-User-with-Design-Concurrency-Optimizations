# Makefile
# For building MPU9250 kernel module and user-space tests on Raspberry Pi 4.
# Supports cross-compilation for arm64 (Pi 4).
# Enhanced for Topics: Add flags for RT, io_uring, RCU, Lockdep/KCSAN (8), liburcu/uring for user.
# Usage: make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
# Install: make install
# Test: ./user_test or ./mpu9250_ipc_test [--mq | --shm | --fifo]
# Generate docs: make docs
# Benchmark: make bench (runs performance tests)

# Note on Memory Layout: During compilation, code segment holds compiled binaries (.ko, executables),
# data segment for globals, stack for function calls, heap for dynamic allocations (e.g., via malloc in user tests).

obj-m += mpu9250_driver.o mpu9250_ops.o mpu9250_fileops.o

KDIR ?= /lib/modules/$(shell uname -r)/build
ARCH ?= arm64
CROSS_COMPILE ?= aarch64-linux-gnu-
CC = $(CROSS_COMPILE)gcc
CFLAGS = -Wall -Wextra -std=c99 -lurcu -luring -fsanitize=thread # Enhanced: libs for urcu/uring/TSan (Topics 2,5,8)
EXTRA_CFLAGS += -DCONFIG_MPU9250_DEBUG -DCONFIG_LOCKDEP -DCONFIG_KCSAN -DCONFIG_PREEMPT_RT -DCONFIG_NUMA # Improved: Add NUMA flag

all: check_deps dtbo
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules
	$(CC) -o user_test user_space_test.c -lm -pthread -lrt $(CFLAGS)
	$(CC) -o mpu9250_ipc_test mpu9250_ipc_test.c -lrt -pthread $(CFLAGS)

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) clean
	rm -f user_test mpu9250_ipc_test *.o *.ko *.mod.* Module.symvers modules.order *.dtbo # Full clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules_install
	sudo insmod mpu9250_driver.ko
	major=$$(awk '/mpu9250/ {print $$1}' /proc/devices); \
	sudo mknod /dev/mpu9250 c $$major 0; \
	sudo chmod 666 /dev/mpu9250; \
	echo 'SUBSYSTEM=="char", KERNEL=="mpu9250", MODE="0666"' | sudo tee /etc/udev/rules.d/99-mpu9250.rules
	sudo udevadm control --reload-rules && sudo udevadm trigger

uninstall:
	sudo rmmod mpu9250_driver
	sudo rm -f /dev/mpu9250
	sudo rm -f /etc/udev/rules.d/99-mpu9250.rules
	sudo udevadm control --reload-rules && sudo udevadm trigger

check_deps:
	@which $(CROSS_COMPILE)gcc >/dev/null || (echo "Error: Cross-compiler $(CROSS_COMPILE)gcc not found"; exit 1)
	@which dtc >/dev/null || (echo "Error: dtc not found"; exit 1)
	@test -d $(KDIR) || (echo "Error: Kernel directory $(KDIR) not found"; exit 1)
	# Improved: Check memory usage (demo)
	@free -h || echo "Memory check not available"
	# New: Check vmalloc (kernel memory)
	@cat /proc/vmallocinfo || echo "Vmalloc check not available"

docs:
	@echo "Generating documentation..."
	@doxygen Doxyfile 2>/dev/null || echo "Warning: Doxygen not found, skipping documentation"
	@echo "Documentation generated in ./docs directory"

# New: Compile DTS
dtbo:
	dtc -@ -I dts -O dtb -o mpu9250.dtbo mpu9250.dts

# New: Calibration test target
calib_test: user_test
	sudo ./user_test --calibrate # Assume added arg

# New: Benchmark target for performance (e.g., NUMA locality)
bench: user_test mpu9250_ipc_test
	@echo "Running benchmarks..."
	sudo ./user_test --bench-numa # Assume --bench-numa arg added for timing
	sudo ./mpu9250_ipc_test --bench # Similar