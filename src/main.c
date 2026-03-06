/* Complete Working main.c for LSM6DS3 on nRF52832 - Zephyr 4.1.99 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <math.h>
#include "imu.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);


/* Main application */
int main(void)
{
    const struct device *dev;
    accel_data_t accel;
    gyro_data_t gyro;
    int ret;
    
    LOG_INF("LSM6DS3 IMU Application Starting...");
    
    /* Get device */
    dev = get_lsm6ds3_device();
    if (dev == NULL) {
        LOG_ERR("Failed to get sensor device");
        return -1;
    }
    
    /* Configure sensor */
    ret = configure_sensor(dev);
    if (ret != 0) {
        LOG_ERR("Failed to configure sensor");
        return -1;
    }
    
    LOG_INF("Sensor initialized successfully!");
    LOG_INF("Starting data polling...\n");
    
    /* Main polling loop */
    while (1) {
        /* Read IMU data */
        ret = read_imu_data(dev, &accel, &gyro);
        if (ret == 0) {
            /* Print accelerometer data (m/s²) */
            printf("Accel: X=%7.3f  Y=%7.3f  Z=%7.3f m/s²\n",
                   (double)accel.x, (double)accel.y, (double)accel.z);
            
            /* Print gyroscope data (rad/s) */
            printf("Gyro:  X=%7.3f  Y=%7.3f  Z=%7.3f rad/s\n",
                   (double)gyro.x, (double)gyro.y, (double)gyro.z);
            
            /* Calculate and print acceleration magnitude */
            float accel_mag = sqrtf(accel.x * accel.x + 
                                   accel.y * accel.y + 
                                   accel.z * accel.z);
            printf("Accel magnitude: %.3f m/s²\n\n", (double)accel_mag);
        }
        
        /* Poll at ~10 Hz (100ms delay) */
        k_msleep(100);
    }
    
    return 0;
}