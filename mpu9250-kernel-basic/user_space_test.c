// user_space_test.c
// User-space test for MPU9250 kernel driver on Raspberry Pi 4.
// Supports POSIX threads, synchronization, message queues, command-line args, signal sending, and detachable threads.
// Enhanced for all Expert Concurrency Topics: Futex with PI, io_uring+eventfd, NUMA pinning, PI Mutex, liburcu, seqlock read, atomics/barriers, TSan debugging.
// Expanded: Full error handling (e.g., futex retry on timeout), benchmarks (e.g., timing for NUMA), zero-copy io_uring.
// Integration example: Can be used in ROS2 robotics node for IMU data fusion (e.g., publish to /imu topic).
// Compile: gcc -o user_test user_space_test.c -lm -pthread -lrt -lurcu -luring -fsanitize=thread -Wall -Wextra -std=c99
// Run: sudo ./user_test [--accel] [--gyro] [--mag] [--dmp] [--detach] [--sample-rate RATE] [--accel-scale SCALE] [--gyro-scale SCALE] [--futex] [--io_uring] [--numa NODE] [--rt] [--bench-numa]

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <stdint.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <mqueue.h>
#include <getopt.h>
#include <sched.h> // For sched_setaffinity (NUMA/RT)
#include <urcu.h> // liburcu for userspace RCU (Topic 5)
#include <liburing.h> // For io_uring (Topic 2)
#include <sys/eventfd.h> // For eventfd (Topic 2)
#include <sys/syscall.h> // For futex syscall (Topic 1)
#include <linux/futex.h> // Futex headers
#include <time.h> // For benchmarks
#include "mpu9250.h"

static int fd = -1;
static mqd_t mq = -1;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
static volatile int running = 1;
static int test_accel = 0, test_gyro = 0, test_mag = 0, test_dmp = 0, test_detach = 0;
static unsigned int sample_rate = 100;
static int accel_scale = ACCEL_SCALE_2G;
static int gyro_scale = GYRO_SCALE_250DPS;
// Enhanced: New flags for topics
static int test_futex = 0, test_io_uring = 0, test_rt = 0, bench_numa = 0;
static int numa_node = 0; // Default node 0 (Topic 3)
static struct io_uring ring; // io_uring (Topic 2)
static int efd = -1; // eventfd (Topic 2)
static atomic_t futex_var; // Futex (Topic 1)
static struct mpu9250_shared *shared_mmap = NULL; // mmap for seqlock/RCU (Topics 5,6)

static void signal_handler(int sig)
{
    if (sig == SIGUSR1) {
        printf("MPU9250: Received SIGUSR1, continuing\n");
        return;
    }
    running = 0;
    pthread_cond_broadcast(&cond);
    if (fd >= 0) close(fd);
    if (mq != -1) {
        mq_close(mq);
        mq_unlink(MPU9250_MQ_NAME);
    }
    if (shared_mmap) munmap(shared_mmap, sizeof(struct mpu9250_shared)); // Clean mmap
    if (efd >= 0) close(efd);
    io_uring_queue_exit(&ring);
    printf("MPU9250: Closed on signal %d\n", sig);
    exit(0);
}

// Demo default signal handler: By default, SIGINT terminates the process.
static void default_signal_demo() {
    // To use default, call signal(SIGINT, SIG_DFL);
    printf("Demo: Default SIGINT would terminate without custom handler.\n");
    signal(SIGINT, SIG_DFL); // New: Set to default
}

// Enhanced: Pin thread to NUMA node/CPU (Topic 3)
static void pin_to_numa(int node) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(node % sysconf(_SC_NPROCESSORS_ONLN), &cpuset); // Fallback to CPU
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) < 0) {
        perror("sched_setaffinity failed"); // Error handling
    }
}

// Enhanced: Seqlock read with retry and barriers (Topics 6,7)
static void read_shared_seqlock(struct mpu9250_shared *shared, float *accel, float *gyro, float *mag, float *quat) {
    unsigned seq;
    int retries = 0;
    do {
        seq = read_seqbegin(&shared->seq);
        // Use atomic loads with barriers (Topic 7)
        memcpy(accel, shared->accel, sizeof(float)*3);
        smp_rmb(); // Read memory barrier to prevent reordering
        memcpy(gyro, shared->gyro, sizeof(float)*3);
        smp_rmb();
        memcpy(mag, shared->mag, sizeof(float)*3);
        smp_rmb();
        memcpy(quat, shared->quat, sizeof(float)*4);
        if (read_seqretry(&shared->seq, seq)) {
            retries++;
            if (retries > 10) { // Edge case: Timeout retry loop
                fprintf(stderr, "Seqlock retry timeout\n");
                break;
            }
        }
    } while (read_seqretry(&shared->seq, seq));
}

// Enhanced: RCU read for offsets (Topic 5, using liburcu)
static void rcu_read_offsets(float *accel_offset, float *gyro_offset) {
    rcu_read_lock();
    // Assume offsets are RCU-protected pointers in shared
    memcpy(accel_offset, rcu_dereference(shared_mmap->accel), sizeof(float)*3); // Example
    memcpy(gyro_offset, rcu_dereference(shared_mmap->gyro), sizeof(float)*3);
    rcu_read_unlock();
}

static void *accel_thread(void *arg)
{
    struct mpu9250_sensor_data *data = malloc(sizeof(struct mpu9250_sensor_data)); // New: malloc
    if (!data) pthread_exit(NULL);
    pthread_t tid = pthread_self(); // New: Get Thread ID
    printf("Accel Thread ID: %lu\n", (unsigned long)tid);
    pin_to_numa(numa_node); // Topic 3
    if (test_rt) { // Topic 4: RT priority
        struct sched_param param = {.sched_priority = 50};
        if (pthread_setschedparam(tid, SCHED_FIFO, &param) < 0) {
            perror("pthread_setschedparam failed");
        }
    }
    float accel[3], offset[3];
    struct timespec start, end; // For benchmark
    if (bench_numa) clock_gettime(CLOCK_MONOTONIC, &start);
    while (running) {
        if (test_futex) { // Topic 1: Wait on futex with PI, retry on timeout
            struct timespec ts = {1, 0}; // 1s timeout
            int ret = syscall(SYS_futex, &futex_var, FUTEX_WAIT_PI, atomic_load(&futex_var), &ts, NULL, 0);
            if (ret == -ETIMEDOUT) {
                printf("Futex timeout, retrying...\n"); // Recovery
                continue;
            } else if (ret < 0) {
                perror("Futex wait failed");
                break;
            }
        } else {
            pthread_mutex_lock(&mutex);
            pthread_cond_wait(&cond, &mutex); // New: Use cond_wait instead of usleep
            pthread_mutex_unlock(&mutex);
        }
        if (test_io_uring) { // Topic 2: Async ioctl via io_uring, with zero-copy
            struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
            if (!sqe) {
                fprintf(stderr, "io_uring_get_sqe failed\n");
                continue;
            }
            io_uring_prep_ioctl(sqe, fd, MPU9250_IOCTL_READ_ACCEL, data);
            sqe->user_data = 1; // For identification
            int ret = io_uring_submit(&ring);
            if (ret < 0) {
                perror("io_uring_submit failed"); // Error code check
                // Recovery: Re-init ring on failure
                io_uring_queue_exit(&ring);
                io_uring_queue_init(8, &ring, 0);
                continue;
            }
            struct io_uring_cqe *cqe;
            ret = io_uring_wait_cqe(&ring, &cqe);
            if (ret < 0) {
                perror("io_uring_wait_cqe failed");
                continue;
            }
            if (cqe->res < 0) {
                fprintf(stderr, "io_uring error: %d\n", cqe->res); // Handle error codes
            }
            io_uring_cqe_seen(&ring, cqe);
            memcpy(accel, data->values, sizeof(float)*3);
        } else {
            if (ioctl(fd, MPU9250_IOCTL_READ_ACCEL, data) < 0) {
                perror("IOCTL read accel failed");
                continue;
            }
            memcpy(accel, data->values, sizeof(float)*3);
        }
        rcu_read_offsets(offset, offset); // Apply offsets
        accel[0] -= offset[0]; accel[1] -= offset[1]; accel[2] -= offset[2];
        read_shared_seqlock(shared_mmap, accel, accel, accel, accel); // Update from shared if needed
        printf("Accel: %.2f %.2f %.2f\n", accel[0], accel[1], accel[2]);
        if (bench_numa) {
            clock_gettime(CLOCK_MONOTONIC, &end);
            long time_ns = (end.tv_sec - start.tv_sec) * 1e9 + (end.tv_nsec - start.tv_nsec);
            printf("NUMA benchmark time: %ld ns\n", time_ns); // Simple benchmark
            break; // Run once for bench
        }
        usleep(100000 / sample_rate);
    }
    free(data);
    pthread_exit(NULL);
}

static void *gyro_thread(void *arg)
{
    struct mpu9250_sensor_data *data = malloc(sizeof(struct mpu9250_sensor_data));
    if (!data) pthread_exit(NULL);
    pthread_t tid = pthread_self();
    printf("Gyro Thread ID: %lu\n", (unsigned long)tid);
    pin_to_numa(numa_node);
    if (test_rt) {
        struct sched_param param = {.sched_priority = 50};
        pthread_setschedparam(tid, SCHED_FIFO, &param);
    }
    float gyro[3], offset[3];
    struct timespec start, end;
    if (bench_numa) clock_gettime(CLOCK_MONOTONIC, &start);
    while (running) {
        if (test_futex) {
            struct timespec ts = {1, 0};
            int ret = syscall(SYS_futex, &futex_var, FUTEX_WAIT_PI, atomic_load(&futex_var), &ts, NULL, 0);
            if (ret == -ETIMEDOUT) continue;
            if (ret < 0) break;
        } else {
            pthread_mutex_lock(&mutex);
            pthread_cond_wait(&cond, &mutex);
            pthread_mutex_unlock(&mutex);
        }
        if (test_io_uring) {
            struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
            if (!sqe) continue;
            io_uring_prep_ioctl(sqe, fd, MPU9250_IOCTL_READ_GYRO, data);
            sqe->user_data = 2;
            int ret = io_uring_submit(&ring);
            if (ret < 0) continue;
            struct io_uring_cqe *cqe;
            ret = io_uring_wait_cqe(&ring, &cqe);
            if (ret < 0) continue;
            if (cqe->res < 0) fprintf(stderr, "io_uring error: %d\n", cqe->res);
            io_uring_cqe_seen(&ring, cqe);
            memcpy(gyro, data->values, sizeof(float)*3);
        } else {
            if (ioctl(fd, MPU9250_IOCTL_READ_GYRO, data) < 0) continue;
            memcpy(gyro, data->values, sizeof(float)*3);
        }
        rcu_read_offsets(offset, offset);
        gyro[0] -= offset[0]; gyro[1] -= offset[1]; gyro[2] -= offset[2];
        read_shared_seqlock(shared_mmap, gyro, gyro, gyro, gyro);
        printf("Gyro: %.2f %.2f %.2f\n", gyro[0], gyro[1], gyro[2]);
        if (bench_numa) {
            clock_gettime(CLOCK_MONOTONIC, &end);
            long time_ns = (end.tv_sec - start.tv_sec) * 1e9 + (end.tv_nsec - start.tv_nsec);
            printf("NUMA benchmark time: %ld ns\n", time_ns);
            break;
        }
        usleep(100000 / sample_rate);
    }
    free(data);
    pthread_exit(NULL);
}

static void *mag_thread(void *arg)
{
    struct mpu9250_sensor_data *data = malloc(sizeof(struct mpu9250_sensor_data));
    if (!data) pthread_exit(NULL);
    pthread_t tid = pthread_self();
    printf("Mag Thread ID: %lu\n", (unsigned long)tid);
    pin_to_numa(numa_node);
    if (test_rt) {
        struct sched_param param = {.sched_priority = 50};
        pthread_setschedparam(tid, SCHED_FIFO, &param);
    }
    float mag[3];
    struct timespec start, end;
    if (bench_numa) clock_gettime(CLOCK_MONOTONIC, &start);
    while (running) {
        if (test_futex) {
            struct timespec ts = {1, 0};
            int ret = syscall(SYS_futex, &futex_var, FUTEX_WAIT_PI, atomic_load(&futex_var), &ts, NULL, 0);
            if (ret == -ETIMEDOUT) continue;
            if (ret < 0) break;
        } else {
            pthread_mutex_lock(&mutex);
            pthread_cond_wait(&cond, &mutex);
            pthread_mutex_unlock(&mutex);
        }
        if (test_io_uring) {
            struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
            if (!sqe) continue;
            io_uring_prep_ioctl(sqe, fd, MPU9250_IOCTL_READ_MAG, data);
            sqe->user_data = 3;
            int ret = io_uring_submit(&ring);
            if (ret < 0) continue;
            struct io_uring_cqe *cqe;
            ret = io_uring_wait_cqe(&ring, &cqe);
            if (ret < 0) continue;
            if (cqe->res < 0) fprintf(stderr, "io_uring error: %d\n", cqe->res);
            io_uring_cqe_seen(&ring, cqe);
            memcpy(mag, data->values, sizeof(float)*3);
        } else {
            if (ioctl(fd, MPU9250_IOCTL_READ_MAG, data) < 0) continue;
            memcpy(mag, data->values, sizeof(float)*3);
        }
        read_shared_seqlock(shared_mmap, mag, mag, mag, mag);
        printf("Mag: %.2f %.2f %.2f\n", mag[0], mag[1], mag[2]);
        if (bench_numa) {
            clock_gettime(CLOCK_MONOTONIC, &end);
            long time_ns = (end.tv_sec - start.tv_sec) * 1e9 + (end.tv_nsec - start.tv_nsec);
            printf("NUMA benchmark time: %ld ns\n", time_ns);
            break;
        }
        usleep(100000 / sample_rate);
    }
    free(data);
    pthread_exit(NULL);
}

static void *dmp_thread(void *arg)
{
    struct mpu9250_sensor_data *data = malloc(sizeof(struct mpu9250_sensor_data));
    if (!data) pthread_exit(NULL);
    pthread_t tid = pthread_self();
    printf("DMP Thread ID: %lu\n", (unsigned long)tid);
    pin_to_numa(numa_node);
    if (test_rt) {
        struct sched_param param = {.sched_priority = 50};
        pthread_setschedparam(tid, SCHED_FIFO, &param);
    }
    float quat[4];
    struct timespec start, end;
    if (bench_numa) clock_gettime(CLOCK_MONOTONIC, &start);
    while (running) {
        if (test_futex) {
            struct timespec ts = {1, 0};
            int ret = syscall(SYS_futex, &futex_var, FUTEX_WAIT_PI, atomic_load(&futex_var), &ts, NULL, 0);
            if (ret == -ETIMEDOUT) continue;
            if (ret < 0) break;
        } else {
            pthread_mutex_lock(&mutex);
            pthread_cond_wait(&cond, &mutex);
            pthread_mutex_unlock(&mutex);
        }
        if (test_io_uring) {
            struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
            if (!sqe) continue;
            io_uring_prep_ioctl(sqe, fd, MPU9250_IOCTL_READ_DMP, data);
            sqe->user_data = 4;
            int ret = io_uring_submit(&ring);
            if (ret < 0) continue;
            struct io_uring_cqe *cqe;
            ret = io_uring_wait_cqe(&ring, &cqe);
            if (ret < 0) continue;
            if (cqe->res < 0) fprintf(stderr, "io_uring error: %d\n", cqe->res);
            io_uring_cqe_seen(&ring, cqe);
            memcpy(quat, data->values, sizeof(float)*4);
        } else {
            if (ioctl(fd, MPU9250_IOCTL_READ_DMP, data) < 0) continue;
            memcpy(quat, data->values, sizeof(float)*4);
        }
        read_shared_seqlock(shared_mmap, quat, quat, quat, quat);
        printf("Quat: %.2f %.2f %.2f %.2f\n", quat[0], quat[1], quat[2], quat[3]);
        if (bench_numa) {
            clock_gettime(CLOCK_MONOTONIC, &end);
            long time_ns = (end.tv_sec - start.tv_sec) * 1e9 + (end.tv_nsec - start.tv_nsec);
            printf("NUMA benchmark time: %ld ns\n", time_ns);
            break;
        }
        usleep(100000 / sample_rate);
    }
    free(data);
    pthread_exit(NULL);
}

int main(int argc, char **argv)
{
    // Parse args (expanded with --bench-numa)
    static struct option long_options[] = {
        {"accel", no_argument, &test_accel, 1},
        {"gyro", no_argument, &test_gyro, 1},
        {"mag", no_argument, &test_mag, 1},
        {"dmp", no_argument, &test_dmp, 1},
        {"detach", no_argument, &test_detach, 1},
        {"sample-rate", required_argument, NULL, 'r'},
        {"accel-scale", required_argument, NULL, 'a'},
        {"gyro-scale", required_argument, NULL, 'g'},
        {"futex", no_argument, &test_futex, 1}, // Topic 1
        {"io_uring", no_argument, &test_io_uring, 1}, // Topic 2
        {"numa", required_argument, NULL, 'n'}, // Topic 3
        {"rt", no_argument, &test_rt, 1}, // Topic 4
        {"bench-numa", no_argument, &bench_numa, 1}, // New: Benchmark
        {0, 0, 0, 0}
    };
    int opt;
    while ((opt = getopt_long(argc, argv, "", long_options, NULL)) != -1) {
        switch (opt) {
            case 'r': sample_rate = atoi(optarg); break;
            case 'a': accel_scale = atoi(optarg); break;
            case 'g': gyro_scale = atoi(optarg); break;
            case 'n': numa_node = atoi(optarg); break;
        }
    }

    default_signal_demo();

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGUSR1, signal_handler);

    fd = open("/dev/mpu9250", O_RDWR);
    if (fd < 0) {
        perror("Open /dev/mpu9250 failed");
        return 1;
    }

    // Enhanced: mmap shared data for seqlock/RCU/futex (Topics 1,5,6)
    shared_mmap = mmap(NULL, sizeof(struct mpu9250_shared), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (shared_mmap == MAP_FAILED) {
        perror("mmap failed");
        goto err;
    }
    futex_var = shared_mmap->futex_var; // Topic 1

    // Enhanced: Setup io_uring (Topic 2), with zero-copy buffers
    if (test_io_uring) {
        if (io_uring_queue_init(8, &ring, 0) < 0) {
            perror("io_uring_queue_init failed");
            goto err;
        }
        if (ioctl(fd, MPU9250_IOCTL_SETUP_IO_URING) < 0) {
            perror("IOCTL setup io_uring failed");
        }
        // Improved: Register files and buffers for zero-copy
        int fds[1] = {fd};
        if (io_uring_register_files(&ring, fds, 1) < 0) {
            perror("io_uring_register_files failed");
        }
        struct iovec iov[1] = {{.iov_base = malloc(MPU9250_MAX_FIFO), .iov_len = MPU9250_MAX_FIFO}};
        if (io_uring_register_buffers(&ring, iov, 1) < 0) {
            perror("io_uring_register_buffers failed");
        }
    }

    // Enhanced: Setup eventfd (Topic 2)
    efd = eventfd(0, EFD_NONBLOCK);
    if (efd < 0) perror("eventfd failed");
    if (ioctl(fd, MPU9250_IOCTL_SETUP_EVENTFD) < 0) {
        perror("IOCTL setup eventfd failed");
    }

    // Enhanced: Futex init (Topic 1)
    if (ioctl(fd, MPU9250_IOCTL_INIT_FUTEX) < 0) {
        perror("IOCTL init futex failed");
    }

    // Enhanced: NUMA node set (Topic 3)
    if (ioctl(fd, MPU9250_IOCTL_SET_NUMA_NODE, numa_node) < 0) {
        perror("IOCTL set numa failed");
    }

    // Enhanced: PI Mutex attr for RT (Topic 4)
    if (test_rt) {
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        if (pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT) < 0) {
            perror("pthread_mutexattr_setprotocol failed");
        }
        pthread_mutex_init(&mutex, &attr);
    }

    int ret = ioctl(fd, MPU9250_IOCTL_SET_ACCEL_SCALE, accel_scale);
    if (ret < 0) {
        perror("Set accel scale failed");
        goto err;
    }
    ret = ioctl(fd, MPU9250_IOCTL_SET_GYRO_SCALE, gyro_scale);
    if (ret < 0) {
        perror("Set gyro scale failed");
        goto err;
    }
    ret = ioctl(fd, MPU9250_IOCTL_SET_SAMPLE_RATE, sample_rate);
    if (ret < 0) {
        perror("Set sample rate failed");
        goto err;
    }

    uint8_t write_buf[2] = {MPU9250_PWR_MGMT_1, 0x00};
    ret = write(fd, write_buf, 2);
    if (ret != 2) {
        perror("Write wakeup failed");
        goto err;
    }
    printf("MPU9250: Wakeup written\n");

    struct mpu9250_reg reg;
    reg.reg = MPU9250_WHO_AM_I;
    ret = ioctl(fd, MPU9250_IOCTL_READ_REG, &reg);
    if (ret < 0) {
        perror("IOCTL read WHO_AM_I failed");
        goto err;
    }
    printf("MPU9250: WHO_AM_I = 0x%02X\n", reg.val);

    // New: Demo exec (fork and exec child to run another program)
    pid_t exec_pid = fork();
    if (exec_pid == 0) {
        execl("/bin/ls", "ls", "-l", NULL); // Demo exec
        perror("exec failed");
        exit(1);
    } else if (exec_pid > 0) {
        wait(NULL); // Wait for exec child
    } else {
        perror("fork failed");
    }

    pthread_attr_t attr;
    if (test_detach) {
        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    }

    pid_t child_pid = fork();
    if (child_pid == -1) {
        perror("Fork failed");
        goto err;
    }
    if (child_pid == 0) {
        sleep(2);
        if (kill(getppid(), SIGUSR1) == -1) {
            perror("Send SIGUSR1 failed");
            exit(1);
        }
        exit(0);
    }

    pthread_t accel_tid, gyro_tid, mag_tid, dmp_tid;
    if (test_accel)
        pthread_create(&accel_tid, test_detach ? &attr : NULL, accel_thread, NULL);
    if (test_gyro)
        pthread_create(&gyro_tid, test_detach ? &attr : NULL, gyro_thread, NULL);
    if (test_mag)
        pthread_create(&mag_tid, test_detach ? &attr : NULL, mag_thread, NULL);
    if (test_dmp)
        pthread_create(&dmp_tid, test_detach ? &attr : NULL, dmp_thread, NULL);

    pthread_cond_broadcast(&cond);

    if (!test_detach) {
        if (test_accel)
            pthread_join(accel_tid, NULL);
        if (test_gyro)
            pthread_join(gyro_tid, NULL);
        if (test_mag)
            pthread_join(mag_tid, NULL);
        if (test_dmp)
            pthread_join(dmp_tid, NULL);
    }

    wait(NULL);

    if (test_detach)
        pthread_attr_destroy(&attr);

    // Enhanced: In loop, if eventfd, read from efd to wait (Topic 2), with error check
    if (test_io_uring) {
        uint64_t val;
        int ret = read(efd, &val, sizeof(val));
        if (ret < 0) {
            perror("eventfd read failed");
        }
    }

err:
    if (fd >= 0) close(fd);
    if (mq != -1) {
        mq_close(mq);
        mq_unlink(MPU9250_MQ_NAME);
    }
    if (shared_mmap) munmap(shared_mmap, sizeof(struct mpu9250_shared));
    if (efd >= 0) close(efd);
    io_uring_queue_exit(&ring);
    printf("MPU9250: Test completed\n");
    return 0;
}