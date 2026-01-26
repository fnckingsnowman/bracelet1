/* Complete Working main.c for LSM6DS3 on nRF52832 - Zephyr 4.1.99 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <math.h>

LOG_MODULE_REGISTER(IMU_APP, LOG_LEVEL_INF);

/* Data structures for sensor readings */
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

/* Get the LSM6DS3 device (using LSM6DSO driver) */
const struct device *get_lsm6ds3_device(void)
{
    const struct device *dev = DEVICE_DT_GET_ANY(st_lsm6dso);
    
    if (dev == NULL) {
        LOG_ERR("No LSM6DSO device found in device tree!");
        return NULL;
    }
    
    if (!device_is_ready(dev)) {
        LOG_ERR("Device %s is not ready", dev->name);
        return NULL;
    }
    
    LOG_INF("Found device %s", dev->name);
    return dev;
}

/* Configure sensor sampling rate */
int configure_sensor(const struct device *dev)
{
    struct sensor_value odr;
    int ret;
    
    /* Set Output Data Rate (ODR) to 104 Hz */
    odr.val1 = 104;
    odr.val2 = 0;
    
    /* Configure Accelerometer ODR */
    ret = sensor_attr_set(dev, 
                          SENSOR_CHAN_ACCEL_XYZ,
                          SENSOR_ATTR_SAMPLING_FREQUENCY, 
                          &odr);
    if (ret != 0) {
        LOG_ERR("Failed to set accel ODR: %d", ret);
        return ret;
    }
    
    /* Configure Gyroscope ODR */
    ret = sensor_attr_set(dev, 
                          SENSOR_CHAN_GYRO_XYZ,
                          SENSOR_ATTR_SAMPLING_FREQUENCY, 
                          &odr);
    if (ret != 0) {
        LOG_ERR("Failed to set gyro ODR: %d", ret);
        return ret;
    }
    
    LOG_INF("Sensor configured: ODR = %d Hz", odr.val1);
    return 0;
}

/* Read both accelerometer and gyroscope data */
int read_imu_data(const struct device *dev, 
                  accel_data_t *accel, 
                  gyro_data_t *gyro)
{
    int ret;
    struct sensor_value accel_val[3];
    struct sensor_value gyro_val[3];
    
    /* Fetch ALL sensor data at once (more efficient) */
    ret = sensor_sample_fetch(dev);
    if (ret != 0) {
        LOG_ERR("Failed to fetch IMU data: %d", ret);
        return ret;
    }
    
    /* Get accelerometer data */
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_val[0]);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_val[1]);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_val[2]);
    
    /* Get gyroscope data */
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &gyro_val[0]);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_val[1]);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gyro_val[2]);
    
    /* Convert to float using Zephyr's built-in function */
    accel->x = sensor_value_to_float(&accel_val[0]);
    accel->y = sensor_value_to_float(&accel_val[1]);
    accel->z = sensor_value_to_float(&accel_val[2]);
    
    gyro->x = sensor_value_to_float(&gyro_val[0]);
    gyro->y = sensor_value_to_float(&gyro_val[1]);
    gyro->z = sensor_value_to_float(&gyro_val[2]);
    
    return 0;
}

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