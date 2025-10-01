// mpu9250_debug.c
// Debugging utilities for MPU9250 kernel driver.
#include "mpu9250.h"
#include <linux/timekeeping.h>

// Measure RCU grace period
void mpu9250_measure_rcu_grace_period(struct mpu9250_data *data)
{
    u64 start = ktime_get_ns();
    synchronize_rcu();
    u64 end = ktime_get_ns();
    dev_info(&data->client->dev, "RCU grace period measured: %llu ns\n", end - start);
}

// Measure RT latency (placeholder for critical section timing)
int mpu9250_rt_latency_measure(struct mpu9250_data *data)
{
    u64 start = ktime_get_ns();
    // Critical section demo
    rt_mutex_lock(&data->lock);
    rt_mutex_unlock(&data->lock);
    u64 end = ktime_get_ns();
    dev_info(&data->client->dev, "RT latency: %llu ns\n", end - start);
    return 0;
}