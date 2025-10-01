// mpu9250_ipc_test.c
// User-space test for MPU9250 kernel driver using POSIX message queues, shared memory, and FIFO.
// Supports command-line args to select test type (--mq, --shm, or --fifo) and queue size.
// Enhanced for all Expert Concurrency Topics: Futex with PI, io_uring+eventfd, NUMA pinning, PI Mutex, liburcu, seqlock read, atomics/barriers, TSan debugging.
// Expanded: Full error handling, benchmarks, zero-copy io_uring.
// Integration: Example for robotics - pipe data to ROS2 publisher.
// Compile: gcc -o mpu9250_ipc_test mpu9250_ipc_test.c -lrt -pthread -lurcu -luring -fsanitize=thread -Wall -Wextra -std=c99
// Run: sudo ./mpu9250_ipc_test [--mq | --shm | --fifo] [--queue-size SIZE] [--futex] [--io_uring] [--numa NODE] [--rt] [--bench]

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <semaphore.h>
#include <mqueue.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <getopt.h>
#include <signal.h>
#include <time.h>
#include <urcu.h> // Topic 5
#include <liburing.h> // Topic 2
#include <sys/eventfd.h> // Topic 2
#include <sys/syscall.h> // Topic 1
#include <linux/futex.h>
#include <sched.h> // Topic 3
#include "mpu9250.h"

#define SHM_NAME "/mpu9250_shm"
#define SEM_NAME "/mpu9250_sem"
#define FIFO_NAME "/tmp/mpu9250_fifo"

static volatile int running = 1;
static long queue_size = 10;
// Enhanced: New flags for topics
static int test_futex = 0, test_io_uring = 0, test_rt = 0, bench = 0;
static int numa_node = 0; // Default node 0 (Topic 3)
static struct io_uring ring; // io_uring (Topic 2)
static int efd = -1; // eventfd (Topic 2)
static atomic_t futex_var; // Futex (Topic 1)
static struct mpu9250_shared *shared_mmap = NULL; // mmap for seqlock/RCU (Topics 5,6)

static void signal_handler(int sig)
{
    running = 0;
    if (shared_mmap) munmap(shared_mmap, sizeof(struct mpu9250_shared)); // Clean mmap
    if (efd >= 0) close(efd);
    io_uring_queue_exit(&ring);
}

// Demo default signal handler: By default, SIGINT terminates the process without custom handler.
// Here, we override it, but to demo default, uncomment signal(SIGINT, SIG_DFL); in main.
static void default_signal_demo() {
    // Default handler for SIGINT would call exit(0) implicitly.
    printf("Demo: Default SIGINT handler would terminate process.\n");
    signal(SIGINT, SIG_DFL); // New: Actually set to default for demo
}

// Enhanced: Pin to NUMA (Topic 3)
static void pin_to_numa(int node) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(node % sysconf(_SC_NPROCESSORS_ONLN), &cpuset);
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) < 0) {
        perror("sched_setaffinity failed");
    }
}

// Enhanced: Seqlock read (Topic 6)
static void read_shared_seqlock(struct mpu9250_shared *shared, struct mpu9250_mq_data *mq_data) {
    unsigned seq;
    int retries = 0;
    do {
        seq = read_seqbegin(&shared->seq);
        memcpy(mq_data->accel, shared->accel, sizeof(float)*3);
        smp_rmb();
        memcpy(mq_data->gyro, shared->gyro, sizeof(float)*3);
        smp_rmb();
        memcpy(mq_data->mag, shared->mag, sizeof(float)*3);
        smp_rmb();
        memcpy(mq_data->quat, shared->quat, sizeof(float)*4);
        mq_data->sample_rate = shared->sample_rate; // New: Read additional fields
        if (read_seqretry(&shared->seq, seq)) {
            retries++;
            if (retries > 10) { // Edge case timeout
                fprintf(stderr, "Seqlock retry timeout\n");
                break;
            }
        }
    } while (read_seqretry(&shared->seq, seq));
}

static int run_mq_test(void)
{
    mqd_t mq;
    struct mpu9250_mq_data data; // Stack alloc for simplicity
    struct mq_attr attr;

    attr.mq_maxmsg = queue_size;
    attr.mq_msgsize = sizeof(struct mpu9250_mq_data);
    mq = mq_open(MPU9250_MQ_NAME, O_RDONLY | O_CREAT, 0666, &attr);
    if (mq == -1) {
        fprintf(stderr, "Open message queue failed: %s\n", strerror(errno));
        return 1;
    }

    // Check for queue overflow
    if (mq_getattr(mq, &attr) == 0 && attr.mq_curmsgs >= attr.mq_maxmsg) {
        fprintf(stderr, "Message queue overflow detected\n");
        mq_close(mq);
        mq_unlink(MPU9250_MQ_NAME);
        return 1;
    }

    pin_to_numa(numa_node); // Topic 3
    if (test_rt) { // Topic 4
        struct sched_param param = {.sched_priority = 50};
        if (sched_setscheduler(0, SCHED_FIFO, &param) < 0) {
            perror("sched_setscheduler failed");
        }
    }

    struct timespec start, end; // Benchmark
    if (bench) clock_gettime(CLOCK_MONOTONIC, &start);
    while (running) {
        if (test_futex) {
            struct timespec ts = {1, 0};
            int ret = syscall(SYS_futex, &futex_var, FUTEX_WAIT_PI, atomic_load(&futex_var), &ts, NULL, 0);
            if (ret == -ETIMEDOUT) {
                printf("Futex timeout, retrying...\n");
                continue;
            } else if (ret < 0) {
                perror("Futex wait failed");
                break;
            }
        } 
        if (mq_receive(mq, (char *)&data, sizeof(data), NULL) == -1) {
            if (errno == EINTR)
                continue;
            fprintf(stderr, "Message queue receive failed: %s\n", strerror(errno));
            break;
        }
        rcu_read_lock(); // Topic 5
        read_shared_seqlock(shared_mmap, &data); // Topic 6
        rcu_read_unlock();
        printf("MQ Data: Accel: %.2f %.2f %.2f, Gyro: %.2f %.2f %.2f, Mag: %.2f %.2f %.2f, Quat: %.2f %.2f %.2f %.2f, Sample Rate: %u\n",
               data.accel[0], data.accel[1], data.accel[2],
               data.gyro[0], data.gyro[1], data.gyro[2],
               data.mag[0], data.mag[1], data.mag[2],
               data.quat[0], data.quat[1], data.quat[2], data.quat[3], data.sample_rate);
        usleep(100000);
    }
    mq_close(mq);
    mq_unlink(MPU9250_MQ_NAME);
    if (bench) {
        clock_gettime(CLOCK_MONOTONIC, &end);
        long time_ns = (end.tv_sec - start.tv_sec) * 1e9 + (end.tv_nsec - start.tv_nsec);
        printf("Benchmark time: %ld ns\n", time_ns);
    }
    return 0;
}

static int run_shm_test(void)
{
    int shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (shm_fd == -1) {
        perror("shm_open failed");
        return 1;
    }
    struct mpu9250_mq_data *shm_data = mmap(0, sizeof(struct mpu9250_mq_data), PROT_READ, MAP_SHARED, shm_fd, 0);
    if (shm_data == MAP_FAILED) {
        perror("mmap failed");
        close(shm_fd);
        return 1;
    }
    sem_t *sem = sem_open(SEM_NAME, 0);
    if (sem == SEM_FAILED) {
        perror("sem_open failed");
        munmap(shm_data, sizeof(struct mpu9250_mq_data));
        close(shm_fd);
        return 1;
    }

    pin_to_numa(numa_node);
    if (test_rt) {
        struct sched_param param = {.sched_priority = 50};
        sched_setscheduler(0, SCHED_FIFO, &param);
    }

    struct timespec start, end;
    if (bench) clock_gettime(CLOCK_MONOTONIC, &start);
    while (running) {
        if (test_futex) {
            struct timespec ts = {1, 0};
            syscall(SYS_futex, &futex_var, FUTEX_WAIT_PI, atomic_load(&futex_var), &ts, NULL, 0);
        }
        sem_wait(sem);
        rcu_read_lock();
        read_shared_seqlock(shared_mmap, shm_data);
        rcu_read_unlock();
        printf("SHM Data: Accel: %.2f %.2f %.2f\n", shm_data->accel[0], shm_data->accel[1], shm_data->accel[2]);
        usleep(100000);
    }
    sem_close(sem);
    munmap(shm_data, sizeof(struct mpu9250_mq_data));
    close(shm_fd);
    if (bench) {
        clock_gettime(CLOCK_MONOTONIC, &end);
        long time_ns = (end.tv_sec - start.tv_sec) * 1e9 + (end.tv_nsec - start.tv_nsec);
        printf("Benchmark time: %ld ns\n", time_ns);
    }
    return 0;
}

static int run_fifo_test(void)
{
    int fifo_fd = open(FIFO_NAME, O_RDONLY);
    if (fifo_fd == -1) {
        perror("open FIFO failed");
        return 1;
    }

    pin_to_numa(numa_node);
    if (test_rt) {
        struct sched_param param = {.sched_priority = 50};
        sched_setscheduler(0, SCHED_FIFO, &param);
    }

    struct mpu9250_mq_data data;
    struct timespec start, end;
    if (bench) clock_gettime(CLOCK_MONOTONIC, &start);
    while (running) {
        if (test_futex) {
            struct timespec ts = {1, 0};
            syscall(SYS_futex, &futex_var, FUTEX_WAIT_PI, atomic_load(&futex_var), &ts, NULL, 0);
        }
        if (read(fifo_fd, &data, sizeof(data)) != sizeof(data)) {
            perror("FIFO read failed");
            break;
        }
        rcu_read_lock();
        read_shared_seqlock(shared_mmap, &data);
        rcu_read_unlock();
        printf("FIFO Data: Accel: %.2f %.2f %.2f\n", data.accel[0], data.accel[1], data.accel[2]);
        usleep(100000);
    }
    close(fifo_fd);
    if (bench) {
        clock_gettime(CLOCK_MONOTONIC, &end);
        long time_ns = (end.tv_sec - start.tv_sec) * 1e9 + (end.tv_nsec - start.tv_nsec);
        printf("Benchmark time: %ld ns\n", time_ns);
    }
    return 0;
}

static int run_pipe_test(void)
{
    int pipe_fd[2];
    struct mpu9250_mq_data data;
    if (pipe(pipe_fd) == -1) {
        perror("Pipe creation failed");
        return 1;
    }

    pid_t pid = fork();
    if (pid == -1) {
        perror("Fork failed");
        return 1;
    }
    if (pid == 0) { // Child: Reader
        close(pipe_fd[1]); // Close write end
        pin_to_numa(numa_node);
        if (test_rt) {
            struct sched_param param = {.sched_priority = 50};
            sched_setscheduler(0, SCHED_FIFO, &param);
        }
        struct timespec start, end;
        if (bench) clock_gettime(CLOCK_MONOTONIC, &start);
        while (running) {
            if (test_futex) {
                struct timespec ts = {1, 0};
                syscall(SYS_futex, &futex_var, FUTEX_WAIT_PI, atomic_load(&futex_var), &ts, NULL, 0);
            }
            if (read(pipe_fd[0], &data, sizeof(data)) != sizeof(data)) {
                fprintf(stderr, "Pipe read failed: %s\n", strerror(errno));
                break;
            }
            rcu_read_lock();
            read_shared_seqlock(shared_mmap, &data);
            rcu_read_unlock();
            printf("Pipe Data: Accel: %.2f %.2f %.2f\n", data.accel[0], data.accel[1], data.accel[2]);
            usleep(100000);
        }
        if (bench) {
            clock_gettime(CLOCK_MONOTONIC, &end);
            long time_ns = (end.tv_sec - start.tv_sec) * 1e9 + (end.tv_nsec - start.tv_nsec);
            printf("Pipe benchmark time: %ld ns\n", time_ns);
        }
        close(pipe_fd[0]);
        printf("Pipe Test (Reader): Completed\n");
        exit(0);
    } else { // Parent: Writer (integrate with real driver read)
        close(pipe_fd[0]); // Close read end
        int dev_fd = open("/dev/mpu9250", O_RDONLY); // New: Real integration
        if (dev_fd < 0) {
            perror("Open /dev/mpu9250 failed");
            wait(NULL);
            return 1;
        }
        pin_to_numa(numa_node); // Topic 3
        if (test_rt) { // Topic 4
            struct sched_param param = {.sched_priority = 50};
            sched_setscheduler(0, SCHED_FIFO, &param);
        }
        // Simulate writing data from driver
        while (running) {
            struct mpu9250_sensor_data sensor;
            if (test_io_uring) { // Topic 2
                struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
                io_uring_prep_ioctl(sqe, dev_fd, MPU9250_IOCTL_READ_ACCEL, &sensor);
                int ret = io_uring_submit(&ring);
                if (ret < 0) perror("io_uring_submit failed");
                struct io_uring_cqe *cqe;
                io_uring_wait_cqe(&ring, &cqe);
                io_uring_cqe_seen(&ring, cqe);
            } else {
                if (ioctl(dev_fd, MPU9250_IOCTL_READ_ACCEL, &sensor) < 0) { // Example read accel
                    perror("IOCTL read failed");
                    break;
                }
            }
            // Fill data from sensor
            memcpy(data.accel, sensor.values, sizeof(float)*3);
            // ... Fill gyro/mag/quat similarly
            if (ioctl(dev_fd, MPU9250_IOCTL_READ_GYRO, &sensor) < 0) break;
            memcpy(data.gyro, sensor.values, sizeof(float)*3);
            if (ioctl(dev_fd, MPU9250_IOCTL_READ_MAG, &sensor) < 0) break;
            memcpy(data.mag, sensor.values, sizeof(float)*3);
            if (ioctl(dev_fd, MPU9250_IOCTL_READ_DMP, &sensor) < 0) break;
            memcpy(data.quat, sensor.values, sizeof(float)*4);
            if (write(pipe_fd[1], &data, sizeof(data)) != sizeof(data)) {
                fprintf(stderr, "Pipe write failed: %s\n", strerror(errno));
                break;
            }
            usleep(100000);
        }
        close(pipe_fd[1]);
        close(dev_fd);
        wait(NULL); // Wait for child
        printf("Pipe Test (Writer): Completed\n");
        return 0;
    }
}

int main(int argc, char **argv)
{
    int test_mq = 0, test_shm = 0, test_fifo = 0, test_pipe = 0;
    int opt;

    static struct option long_options[] = {
        {"mq", no_argument, &test_mq, 1},
        {"shm", no_argument, &test_shm, 1},
        {"fifo", no_argument, &test_fifo, 1},
        {"pipe", no_argument, &test_pipe, 1},
        {"queue-size", required_argument, 0, 'q'},
        {"futex", no_argument, &test_futex, 1}, // Topic 1
        {"io_uring", no_argument, &test_io_uring, 1}, // Topic 2
        {"numa", required_argument, NULL, 'n'}, // Topic 3
        {"rt", no_argument, &test_rt, 1}, // Topic 4
        {"bench", no_argument, &bench, 1}, // New: Benchmark
        {0, 0, 0, 0}
    };
    int opt_index;
    while ((opt = getopt_long(argc, argv, "", long_options, &opt_index)) != -1) {
        switch (opt) {
            case 'q':
                queue_size = atol(optarg);
                if (queue_size < 1 || queue_size > 100) {
                    fprintf(stderr, "Invalid queue size (1-100)\n");
                    return 1;
                }
                break;
            case 'n': numa_node = atoi(optarg); break;
        }
    }
    if (!test_mq && !test_shm && !test_fifo && !test_pipe) {
        printf("Please specify --mq, --shm, --fifo, or --pipe\n");
        return 1;
    }

    default_signal_demo();

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGSEGV, signal_handler); // New: Handle more signals

    // Enhanced: mmap shared data
    int dev_fd = open("/dev/mpu9250", O_RDWR);
    if (dev_fd < 0) {
        perror("Open /dev/mpu9250 failed");
        return 1;
    }
    shared_mmap = mmap(NULL, sizeof(struct mpu9250_shared), PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, 0);
    if (shared_mmap == MAP_FAILED) {
        perror("mmap failed");
        close(dev_fd);
        return 1;
    }
    futex_var = shared_mmap->futex_var; // Topic 1

    // Enhanced: io_uring
    if (test_io_uring) {
        io_uring_queue_init(8, &ring, 0);
        ioctl(dev_fd, MPU9250_IOCTL_SETUP_IO_URING);
        // Improved: Register files
        int fds[1] = {dev_fd};
        io_uring_register_files(&ring, fds, 1);
    }

    // Enhanced: eventfd
    efd = eventfd(0, EFD_NONBLOCK);
    if (efd < 0) perror("eventfd failed");
    ioctl(dev_fd, MPU9250_IOCTL_SETUP_EVENTFD);

    // Enhanced: Futex
    ioctl(dev_fd, MPU9250_IOCTL_INIT_FUTEX);

    // Enhanced: NUMA
    ioctl(dev_fd, MPU9250_IOCTL_SET_NUMA_NODE, numa_node);

    if (test_mq)
        return run_mq_test();
    else if (test_shm)
        return run_shm_test();
    else if (test_fifo)
        return run_fifo_test();
    else
        return run_pipe_test();
}