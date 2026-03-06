#include "imu.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);  // Changed from IMU_APP to imu

#define st_lsm6dso DT_NODELABEL(lsm6dso)

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

int configure_sensor(const struct device *dev)
{
    struct sensor_value odr;
    int ret;
    
    odr.val1 = 104;
    odr.val2 = 0;
    
    ret = sensor_attr_set(dev, 
                          SENSOR_CHAN_ACCEL_XYZ,
                          SENSOR_ATTR_SAMPLING_FREQUENCY, 
                          &odr);
    if (ret != 0) {
        LOG_ERR("Failed to set accel ODR: %d", ret);
        return ret;
    }
    
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

int read_imu_data(const struct device *dev, 
                  accel_data_t *accel, 
                  gyro_data_t *gyro)
{
    int ret;
    struct sensor_value accel_val[3];
    struct sensor_value gyro_val[3];
    
    ret = sensor_sample_fetch(dev);
    if (ret != 0) {
        LOG_ERR("Failed to fetch IMU data: %d", ret);
        return ret;
    }
    
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_val[0]);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_val[1]);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_val[2]);
    
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &gyro_val[0]);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_val[1]);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gyro_val[2]);
    
    accel->x = sensor_value_to_float(&accel_val[0]);
    accel->y = sensor_value_to_float(&accel_val[1]);
    accel->z = sensor_value_to_float(&accel_val[2]);
    
    gyro->x = sensor_value_to_float(&gyro_val[0]);
    gyro->y = sensor_value_to_float(&gyro_val[1]);
    gyro->z = sensor_value_to_float(&gyro_val[2]);
    
    return 0;
}