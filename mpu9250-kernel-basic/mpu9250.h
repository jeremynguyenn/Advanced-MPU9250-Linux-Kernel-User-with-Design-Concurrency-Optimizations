// mpu9250.h
// Header file for MPU9250 kernel driver on Raspberry Pi 4 with advanced features.
// Combines core definitions and driver-specific constants/prototypes.
// Enhanced for all Expert Concurrency Topics: Futex, io_uring, NUMA, PI Mutex, RCU, Seqlock, Barriers, Debugging.
// Expanded: Multi-instance support (instance_id), full RCU heads, docs on race conditions.
// Author: Nguyen Nhan

#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/input.h> // For input subsystem
#include <linux/rtmutex.h> // Enhanced for PI Mutex (Topic 4)
#include <linux/seqlock.h> // Enhanced for Seqlocks (Topic 6)
#include <linux/rcupdate.h> // Enhanced for RCU (Topic 5)
#include <linux/eventfd.h> // Enhanced for eventfd (Topic 2)
#include <linux/io_uring.h> // Enhanced for io_uring (Topic 2)
#include <linux/futex.h> // Enhanced for Futex (Topic 1)
#include <linux/numa.h> // Enhanced for NUMA (Topic 3)
#include <linux/atomic.h> // Enhanced for Barriers (Topic 7)
#include <linux/lockdep.h> // Enhanced for Lockdep (Topic 8)
#include <linux/kcsan.h> // KCSAN

// Constants
#define MPU9250_DEFAULT_ADDR    0x68
#define MPU9250_DEVICE_NAME     "mpu9250"
#define MPU9250_CLASS_NAME      "mpu9250_class"
#define MPU9250_MAX_FIFO        4096
#define DRIVER_VERSION          "2.2"
#define MPU9250_MQ_NAME         "/mpu9250_mq"

// Scaling factors
#define ACCEL_SCALE_2G  16384.0f
#define ACCEL_SCALE_4G  8192.0f
#define ACCEL_SCALE_8G  4096.0f
#define ACCEL_SCALE_16G 2048.0f
#define GYRO_SCALE_250  131.0f
#define GYRO_SCALE_500  65.5f
#define GYRO_SCALE_1000 32.8f
#define GYRO_SCALE_2000 16.4f
#define MAG_SCALE       0.15f

// Registers
#define MPU9250_WHO_AM_I        0x75
#define MPU9250_PWR_MGMT_1      0x6B
#define MPU9250_ACCEL_XOUT_H    0x3B
#define MPU9250_GYRO_XOUT_H     0x43
#define MPU9250_MAG_XOUT_L      0x03
#define MPU9250_FIFO_EN         0x23
#define MPU9250_FIFO_COUNTH     0x72
#define MPU9250_FIFO_COUNTL     0x73
#define MPU9250_FIFO_R_W        0x74
#define MPU9250_DMP_CFG_1       0x01
#define MPU9250_DMP_CFG_2       0x02
#define MPU9250_ACCEL_CONFIG    0x1C
#define MPU9250_GYRO_CONFIG     0x1B
#define MPU9250_SMPLRT_DIV      0x19
#define MPU9250_INT_STATUS      0x3A
#define MPU9250_INT_ENABLE      0x38
#define MPU9250_INT_PIN_CFG     0x37
#define MPU9250_USER_CTRL       0x6A // Added for FIFO reset

// Enums
enum mpu9250_accel_scale {
    ACCEL_SCALE_2G = 0,
    ACCEL_SCALE_4G,
    ACCEL_SCALE_8G,
    ACCEL_SCALE_16G
};

enum mpu9250_gyro_scale {
    GYRO_SCALE_250DPS = 0,
    GYRO_SCALE_500DPS,
    GYRO_SCALE_1000DPS,
    GYRO_SCALE_2000DPS
};

enum mpu9250_interrupt {
    MPU9250_INTERRUPT_DATA_READY = 0,
    MPU9250_INTERRUPT_FIFO_OVERFLOW,
    MPU9250_INTERRUPT_DMP
};

// Structs
// Note on Memory Layout: In kernel mode, structs like mpu9250_data are allocated in heap (via kmalloc). 
// Kernel memory layout includes: code (text segment for driver functions), data (global vars like mpu9250_regmap_config), 
// stack (for local vars in functions), heap (dynamic allocations like fifo_buf via vmalloc).
// Race condition note: shared_data is protected by seqlock for consistent cross-domain reads; RCU for pointer updates to avoid use-after-free.
struct mpu9250_reg {
    uint8_t reg;
    uint8_t val;
};

struct mpu9250_sensor_data {
    float values[4]; // For accel/gyro/mag/quat
};

struct mpu9250_mq_data {
    float accel[3];
    float gyro[3];
    float mag[3];
    float quat[4];
    unsigned int sample_rate; // Expanded
};

// Enhanced: Shared struct for mmap, with seqlock and futex var.
struct mpu9250_shared {
    seqlock_t seq; // Seqlock for cross-domain sharing (Topic 6): Writers update atomically, readers retry on inconsistency
    atomic_t futex_var; // Futex variable (Topic 1): Atomic inc/wake to avoid races in signaling
    struct rcu_head rcu; // RCU head for updates (Topic 5): Safe deferred free
    float accel[3];
    float gyro[3];
    float mag[3];
    float quat[4];
    // New: Add for seqlock expansion
    unsigned int sample_rate;
    enum mpu9250_accel_scale accel_scale;
    enum mpu9250_gyro_scale gyro_scale;
    int instance_id; // For multi-instance
};

struct mpu9250_data {
    struct i2c_client *client;
    struct regmap *regmap;
    struct i2c_client *mag_client;
    struct rt_mutex lock; // Enhanced: rt_mutex for PI (Topic 4): Avoids priority inversion in RT
    spinlock_t data_lock;
    struct work_struct read_work;
    wait_queue_head_t wq;
    dev_t dev_t;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    float accel_g[3];
    float gyro_dps[3];
    float mag_uT[3];
    float quat[4];
    uint8_t *fifo_buf;
    int fifo_len;
    bool data_ready;
    uint32_t gs_flag;
    enum mpu9250_accel_scale accel_scale;
    enum mpu9250_gyro_scale gyro_scale;
    unsigned int sample_rate;
    int instance_id; // New: For multi-instance support
    // New: Calibration offsets (RCU-protected)
    float *accel_offset; // Pointer for RCU (Topic 5)
    float *gyro_offset;
    // New: Input device for evdev integration
    struct input_dev *input_dev;
    // Enhanced: io_uring context (Topic 2)
    struct io_uring uring; // Changed to embedded for full control
    int efd_fd; // For eventfd setup
    // Enhanced: eventfd for async signaling (Topic 2)
    struct eventfd_ctx *efd;
    // Enhanced: Shared data for mmap/futex/seqlock
    struct mpu9250_shared *shared_data; // NUMA-allocated (Topic 3)
    int numa_node; // For NUMA awareness (Topic 3)
};

// IOCTL commands (enhanced with new ones for topics)
#define MPU9250_IOC_MAGIC 'm'
#define MPU9250_IOCTL_READ_REG   _IOR('m', 1, struct mpu9250_reg)
#define MPU9250_IOCTL_WRITE_REG  _IOW('m', 2, struct mpu9250_reg)
#define MPU9250_IOCTL_READ_ACCEL _IOR('m', 3, struct mpu9250_sensor_data)
#define MPU9250_IOCTL_READ_GYRO  _IOR('m', 4, struct mpu9250_sensor_data)
#define MPU9250_IOCTL_READ_MAG   _IOR('m', 5, struct mpu9250_sensor_data)
#define MPU9250_IOCTL_READ_FIFO  _IOR('m', 6, struct mpu9250_sensor_data)
#define MPU9250_IOCTL_RESET      _IO('m', 7)
#define MPU9250_IOCTL_INIT_DMP   _IO('m', 8)
#define MPU9250_IOCTL_READ_DMP   _IOR('m', 9, struct mpu9250_sensor_data)
#define MPU9250_IOCTL_SET_NONBLOCK _IOW('m', 10, int)
#define MPU9250_IOCTL_SET_ACCEL_SCALE _IOW('m', 11, enum mpu9250_accel_scale)
#define MPU9250_IOCTL_SET_GYRO_SCALE _IOW('m', 12, enum mpu9250_gyro_scale)
#define MPU9250_IOCTL_SET_SAMPLE_RATE _IOW('m', 13, unsigned int)
#define MPU9250_IOCTL_CALIBRATE  _IO('m', 14) // New: For calibration
// Enhanced IOCTLs
#define MPU9250_IOCTL_INIT_FUTEX _IO('m', 15) // Topic 1
#define MPU9250_IOCTL_SETUP_IO_URING _IO('m', 16) // Topic 2
#define MPU9250_IOCTL_SETUP_EVENTFD _IOW('m', 17, int) // Topic 2, pass efd_fd
#define MPU9250_IOCTL_SET_NUMA_NODE _IOW('m', 18, int) // Topic 3

// Function prototypes (enhanced with new ones)
int mpu9250_alloc_buffers(struct mpu9250_data *data);
void mpu9250_free_buffers(struct mpu9250_data *data);
int mpu9250_read(struct mpu9250_data *data, uint8_t reg, uint8_t *buf, uint16_t len);
int mpu9250_write(struct mpu9250_data *data, uint8_t reg, uint8_t *buf, uint16_t len);
int mpu9250_i2c_init(struct mpu9250_data *data);
int mpu9250_fifo_init(struct mpu9250_data *data);
int mpu9250_read_fifo(struct mpu9250_data *data, uint8_t *buf, int *len);
int mpu9250_dmp_init(struct mpu9250_data *data);
void mpu9250_read_dmp_quat(struct mpu9250_data *data, float *quat);
void mpu9250_read_work(struct work_struct *work);
irqreturn_t mpu9250_irq_handler(int irq, void *dev_id);
int mpu9250_fileops_init(struct mpu9250_data *data);
void mpu9250_fileops_cleanup(struct mpu9250_data *data);
int mpu9250_set_accel_scale(struct mpu9250_data *data, enum mpu9250_accel_scale scale);
int mpu9250_set_gyro_scale(struct mpu9250_data *data, enum mpu9250_gyro_scale scale);
int mpu9250_set_sample_rate(struct mpu9250_data *data, unsigned int rate);
int mpu9250_calibrate(struct mpu9250_data *data); // New

// New: Vmalloc for large buffers if needed
void *mpu9250_vmalloc(size_t size);
void mpu9250_vfree(void *ptr);

// Enhanced prototypes for topics
int mpu9250_init_futex(struct mpu9250_data *data); // Topic 1
int mpu9250_setup_io_uring(struct mpu9250_data *data); // Topic 2
int mpu9250_setup_eventfd(struct mpu9250_data *data); // Topic 2
int mpu9250_set_numa_node(struct mpu9250_data *data, int node); // Topic 3
void mpu9250_update_offsets_rcu(struct mpu9250_data *data, float *new_accel, float *new_gyro); // Topic 5

#endif /* _MPU9250_H_ */