// mpu9250_futex.c
// Futex-based locking for MPU9250 kernel driver.
#include "mpu9250.h"

// Initialize futex mutex
int mpu9250_futex_mutex_init(struct mpu9250_data *data)
{
    atomic_set(&data->shared_data->futex_var, 0);
    return 0;
}

// Custom futex-based lock with error handling (e.g., FUTEX_OWNER_DIED)
int mpu9250_futex_mutex_lock(struct mpu9250_data *data)
{
    int val = atomic_xchg(&data->shared_data->futex_var, -1);
    if (val == 0) return 0;
    while (1) {
        int ret = syscall(SYS_futex, &data->shared_data->futex_var, FUTEX_WAIT_PI, val, NULL, NULL, 0);
        if (ret == 0) return 0;
        if (ret == -EOWNERDEAD) { // New: Handle FUTEX_OWNER_DIED
            atomic_set(&data->shared_data->futex_var, -1); // Claim ownership
            dev_warn(&data->client->dev, "Futex owner died, recovered\n");
            return 0;
        }
        val = atomic_read(&data->shared_data->futex_var);
        if (val == 0 && atomic_cmpxchg(&data->shared_data->futex_var, 0, -1) == 0) return 0;
    }
}

// Unlock futex mutex
int mpu9250_futex_mutex_unlock(struct mpu9250_data *data)
{
    atomic_set(&data->shared_data->futex_var, 0);
    syscall(SYS_futex, &data->shared_data->futex_var, FUTEX_WAKE_PI, 1, NULL, NULL, 0);
    return 0;
}

// Advanced: Futex requeue for queueing
int mpu9250_futex_requeue(struct mpu9250_data *data, int nr_wake, int nr_requeue)
{
    return syscall(SYS_futex, &data->shared_data->futex_var, FUTEX_CMP_REQUEUE_PI, nr_wake, nr_requeue, &data->shared_data->futex_var, 0);
}