// imu.h
#ifndef IMU_H
#define IMU_H

/* zephyr.h was removed in newer Zephyr versions; include necessary headers directly */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <math.h>

typedef struct {
    float x;
    float y;
    float z;
} accel_data_t;

typedef struct {
    float x;
    float y;
    float z;
} gyro_data_t;

const struct device *get_lsm6ds3_device(void);

int configure_sensor(const struct device *dev);

int read_imu_data(const struct device *dev, 
                  accel_data_t *accel, 
                  gyro_data_t *gyro);

#endif /* IMU_H */