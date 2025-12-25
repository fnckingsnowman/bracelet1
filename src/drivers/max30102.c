/**
 * @file max30102.c
 * @brief MAX30102 Pulse Oximeter and Heart Rate Sensor Driver Implementation
 */

#include "max30102.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(max30102, LOG_LEVEL_DBG);

/*
 * ============================================================================
 * Private Definitions
 * ============================================================================
 */

/* I2C address (7-bit, fixed) */
#define MAX30102_I2C_ADDR       0x57

/* Reset bit in MODE_CONFIG register */
#define MAX30102_RESET_BIT      0x40

/* Shutdown bit in MODE_CONFIG register */
#define MAX30102_SHUTDOWN_BIT   0x80

/* FIFO almost full interrupt enable bit */
#define MAX30102_INT_A_FULL_EN  0x80

/* Number of bytes per sample in SpO2 mode (3 bytes RED + 3 bytes IR) */
#define BYTES_PER_SAMPLE_SPO2   6

/* Number of bytes per sample in HR mode (3 bytes RED only) */
#define BYTES_PER_SAMPLE_HR     3

/* Maximum FIFO depth */
#define MAX30102_FIFO_DEPTH     32

/* Timeout for operations */
#define MAX30102_TIMEOUT_MS     100

/*
 * ============================================================================
 * Private Helper Functions
 * ============================================================================
 */

/**
 * @brief Read a single register
 */
static int max30102_read_reg(struct max30102_device *dev, 
                             uint8_t reg, uint8_t *value)
{
    int ret;
    
    ret = i2c_reg_read_byte(dev->i2c, MAX30102_I2C_ADDR, reg, value);
    if (ret != 0) {
        LOG_ERR("Failed to read register 0x%02X: %d", reg, ret);
    }
    
    return ret;
}

/**
 * @brief Write a single register
 */
static int max30102_write_reg(struct max30102_device *dev,
                              uint8_t reg, uint8_t value)
{
    int ret;
    
    ret = i2c_reg_write_byte(dev->i2c, MAX30102_I2C_ADDR, reg, value);
    if (ret != 0) {
        LOG_ERR("Failed to write register 0x%02X: %d", reg, ret);
    }
    
    return ret;
}

/**
 * @brief Read multiple bytes starting from a register
 */
static int max30102_read_burst(struct max30102_device *dev,
                               uint8_t start_reg,
                               uint8_t *buffer,
                               size_t length)
{
    int ret;
    
    ret = i2c_burst_read(dev->i2c, MAX30102_I2C_ADDR, start_reg, buffer, length);
    if (ret != 0) {
        LOG_ERR("Failed burst read from 0x%02X (len=%d): %d", 
                start_reg, length, ret);
    }
    
    return ret;
}

/**
 * @brief Modify specific bits in a register
 */
static int max30102_modify_reg(struct max30102_device *dev,
                               uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t current;
    int ret;
    
    ret = max30102_read_reg(dev, reg, &current);
    if (ret != 0) {
        return ret;
    }
    
    current = (current & ~mask) | (value & mask);
    
    return max30102_write_reg(dev, reg, current);
}

/*
 * ============================================================================
 * Public API Implementation
 * ============================================================================
 */

int max30102_init(struct max30102_device *dev, const struct device *i2c_dev)
{
    int ret;
    
    if (dev == NULL || i2c_dev == NULL) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    /* Clear the device structure */
    memset(dev, 0, sizeof(struct max30102_device));
    dev->i2c = i2c_dev;
    
    LOG_INF("Initializing MAX30102...");
    
    /* Verify I2C communication by reading Part ID */
    ret = max30102_read_reg(dev, MAX30102_REG_PART_ID, &dev->part_id);
    if (ret != 0) {
        LOG_ERR("Failed to communicate with MAX30102");
        return ret;
    }
    
    /* Verify Part ID */
    if (dev->part_id != MAX30102_EXPECTED_PART_ID) {
        LOG_ERR("Invalid Part ID: 0x%02X (expected 0x%02X)",
                dev->part_id, MAX30102_EXPECTED_PART_ID);
        return -ENODEV;
    }
    
    /* Read Revision ID */
    ret = max30102_read_reg(dev, MAX30102_REG_REV_ID, &dev->rev_id);
    if (ret != 0) {
        LOG_WRN("Failed to read Revision ID");
        dev->rev_id = 0;
    }
    
    LOG_INF("MAX30102 detected - Part ID: 0x%02X, Rev ID: 0x%02X",
            dev->part_id, dev->rev_id);
    
    /* Perform software reset */
    ret = max30102_reset(dev);
    if (ret != 0) {
        LOG_ERR("Reset failed");
        return ret;
    }
    
    /* Wait for reset to complete */
    k_sleep(K_MSEC(10));
    
    dev->initialized = true;
    LOG_INF("MAX30102 initialization complete");
    
    return 0;
}

int max30102_reset(struct max30102_device *dev)
{
    int ret;
    uint8_t mode_config;
    int timeout = 100;
    
    LOG_DBG("Performing software reset...");
    
    /* Set reset bit */
    ret = max30102_write_reg(dev, MAX30102_REG_MODE_CONFIG, MAX30102_RESET_BIT);
    if (ret != 0) {
        return ret;
    }
    
    /* Wait for reset bit to clear (indicates reset complete) */
    do {
        k_sleep(K_MSEC(1));
        ret = max30102_read_reg(dev, MAX30102_REG_MODE_CONFIG, &mode_config);
        if (ret != 0) {
            return ret;
        }
        timeout--;
    } while ((mode_config & MAX30102_RESET_BIT) && (timeout > 0));
    
    if (timeout == 0) {
        LOG_ERR("Reset timeout");
        return -ETIMEDOUT;
    }
    
    LOG_DBG("Reset complete");
    return 0;
}

int max30102_configure(struct max30102_device *dev,
                       const struct max30102_config *config)
{
    int ret;
    
    if (!dev->initialized) {
        LOG_ERR("Device not initialized");
        return -ENODEV;
    }
    
    if (config == NULL) {
        LOG_ERR("Invalid config");
        return -EINVAL;
    }
    
    LOG_INF("Configuring MAX30102...");
    LOG_INF("  Mode: 0x%02X", config->mode);
    LOG_INF("  Sample Rate: %d", 
            (config->sample_rate == MAX30102_SR_50) ? 50 :
            (config->sample_rate == MAX30102_SR_100) ? 100 :
            (config->sample_rate == MAX30102_SR_200) ? 200 :
            (config->sample_rate == MAX30102_SR_400) ? 400 : 0);
    LOG_INF("  LED1 Current: %d (%.1f mA)", 
            config->led1_current, config->led1_current * 0.2f);
    LOG_INF("  LED2 Current: %d (%.1f mA)", 
            config->led2_current, config->led2_current * 0.2f);
    
    /* Clear FIFO pointers first */
    ret = max30102_clear_fifo(dev);
    if (ret != 0) {
        return ret;
    }
    
    /*
     * Configure FIFO (Register 0x08)
     * 
     * Bit 7:5 - SMP_AVE[2:0]: Sample averaging
     * Bit 4   - FIFO_ROLLOVER_EN: Enable rollover when full
     * Bit 3:0 - FIFO_A_FULL[3:0]: FIFO almost full trigger (samples remaining)
     */
    uint8_t fifo_config = (config->sample_avg << 5);
    if (config->fifo_rollover) {
        fifo_config |= 0x10;  /* Enable rollover */
    }
    fifo_config |= 0x0F;  /* Almost full at 17 samples remaining */
    
    ret = max30102_write_reg(dev, MAX30102_REG_FIFO_CONFIG, fifo_config);
    if (ret != 0) {
        return ret;
    }
    
    /*
     * Configure SPO2 (Register 0x0A)
     * 
     * Bit 6:5 - SPO2_ADC_RGE[1:0]: ADC range
     * Bit 4:2 - SPO2_SR[2:0]: Sample rate
     * Bit 1:0 - LED_PW[1:0]: LED pulse width
     */
    uint8_t spo2_config = (config->adc_range << 5) |
                          (config->sample_rate << 2) |
                          (config->pulse_width);
    
    ret = max30102_write_reg(dev, MAX30102_REG_SPO2_CONFIG, spo2_config);
    if (ret != 0) {
        return ret;
    }
    
    /* Configure LED currents */
    ret = max30102_write_reg(dev, MAX30102_REG_LED1_PA, config->led1_current);
    if (ret != 0) {
        return ret;
    }
    
    ret = max30102_write_reg(dev, MAX30102_REG_LED2_PA, config->led2_current);
    if (ret != 0) {
        return ret;
    }
    
    /*
     * Set operating mode (Register 0x09)
     * This should be done last as it starts the sampling
     */
    ret = max30102_write_reg(dev, MAX30102_REG_MODE_CONFIG, config->mode);
    if (ret != 0) {
        return ret;
    }
    
    /* Store the configuration */
    memcpy(&dev->config, config, sizeof(struct max30102_config));
    
    LOG_INF("Configuration complete");
    return 0;
}

int max30102_configure_default(struct max30102_device *dev)
{
    /*
     * Default configuration for heart rate monitoring:
     * - SpO2 mode: Both Red and IR LEDs active
     * - 100 Hz sample rate: Good balance of resolution and power
     * - 411 µs pulse width: Maximum ADC resolution (18-bit)
     * - 4096 nA ADC range: Good for typical skin types
     * - 4-sample averaging: Reduces noise, effective rate = 25 Hz
     * - 7 mA LED current: Moderate, adjust based on signal quality
     */
    struct max30102_config config = {
        .mode           = MAX30102_MODE_SPO2,
        .sample_rate    = MAX30102_SR_100,
        .pulse_width    = MAX30102_PW_411,
        .adc_range      = MAX30102_ADC_4096,
        .sample_avg     = MAX30102_AVG_4,
        .led1_current   = 0x24,         /* ~7 mA (0x24 * 0.2 = 7.2 mA) */
        .led2_current   = 0x24,         /* ~7 mA */
        .fifo_rollover  = true,
    };
    
    return max30102_configure(dev, &config);
}

int max30102_clear_fifo(struct max30102_device *dev)
{
    int ret;
    
    /* Clear all FIFO pointers by writing 0 */
    ret = max30102_write_reg(dev, MAX30102_REG_FIFO_WR_PTR, 0x00);
    if (ret != 0) return ret;
    
    ret = max30102_write_reg(dev, MAX30102_REG_FIFO_OVF_CNT, 0x00);
    if (ret != 0) return ret;
    
    ret = max30102_write_reg(dev, MAX30102_REG_FIFO_RD_PTR, 0x00);
    if (ret != 0) return ret;
    
    LOG_DBG("FIFO cleared");
    return 0;
}

int max30102_get_fifo_status(struct max30102_device *dev,
                             struct max30102_fifo_status *status)
{
    int ret;
    uint8_t buffer[3];
    
    /* Read write pointer, overflow counter, and read pointer */
    ret = max30102_read_burst(dev, MAX30102_REG_FIFO_WR_PTR, buffer, 3);
    if (ret != 0) {
        return ret;
    }
    
    status->write_ptr = buffer[0] & 0x1F;       /* 5-bit pointer */
    status->overflow_cnt = buffer[1] & 0x1F;    /* 5-bit counter */
    status->read_ptr = buffer[2] & 0x1F;        /* 5-bit pointer */
    
    /* Calculate available samples */
    if (status->write_ptr >= status->read_ptr) {
        status->available = status->write_ptr - status->read_ptr;
    } else {
        status->available = (32 - status->read_ptr) + status->write_ptr;
    }
    
    return 0;
}

int max30102_read_samples(struct max30102_device *dev,
                          struct max30102_sample *samples,
                          uint8_t max_samples,
                          uint8_t *num_read)
{
    int ret;
    struct max30102_fifo_status status;
    uint8_t to_read;
    uint8_t bytes_per_sample;
    uint8_t fifo_buffer[BYTES_PER_SAMPLE_SPO2 * MAX30102_FIFO_DEPTH];
    
    *num_read = 0;
    
    /* Determine bytes per sample based on mode */
    if (dev->config.mode == MAX30102_MODE_HR_ONLY) {
        bytes_per_sample = BYTES_PER_SAMPLE_HR;
    } else {
        bytes_per_sample = BYTES_PER_SAMPLE_SPO2;
    }
    
    /* Get FIFO status */
    ret = max30102_get_fifo_status(dev, &status);
    if (ret != 0) {
        return ret;
    }
    
    if (status.available == 0) {
        return 0;  /* No samples available */
    }
    
    /* Determine how many samples to read */
    to_read = (status.available < max_samples) ? status.available : max_samples;
    
    /* Read all samples in one burst from FIFO data register */
    ret = max30102_read_burst(dev, MAX30102_REG_FIFO_DATA,
                              fifo_buffer, to_read * bytes_per_sample);
    if (ret != 0) {
        return ret;
    }
    
    /* Get current timestamp */
    int64_t now = k_uptime_get();
    
    /* Parse the samples */
    for (uint8_t i = 0; i < to_read; i++) {
        uint8_t *ptr = &fifo_buffer[i * bytes_per_sample];
        
        /*
         * Each channel is 3 bytes, MSB first, but only 18 bits are valid.
         * Data format: [23:18] = unused, [17:0] = sample data
         */
        
        /* Parse Red LED value (always present) */
        samples[i].red = ((uint32_t)ptr[0] << 16) |
                         ((uint32_t)ptr[1] << 8) |
                         (uint32_t)ptr[2];
        samples[i].red &= 0x3FFFF;  /* Mask to 18 bits */
        
        /* Parse IR LED value (only in SpO2 mode) */
        if (bytes_per_sample == BYTES_PER_SAMPLE_SPO2) {
            samples[i].ir = ((uint32_t)ptr[3] << 16) |
                            ((uint32_t)ptr[4] << 8) |
                            (uint32_t)ptr[5];
            samples[i].ir &= 0x3FFFF;  /* Mask to 18 bits */
        } else {
            samples[i].ir = 0;
        }
        
        /* Estimate timestamp (samples are from past) */
        samples[i].timestamp = now - ((to_read - 1 - i) * 10);  /* Assuming ~100Hz */
    }
    
    *num_read = to_read;
    
    LOG_DBG("Read %d samples from FIFO", to_read);
    return 0;
}

int max30102_read_sample(struct max30102_device *dev,
                         struct max30102_sample *sample)
{
    int ret;
    uint8_t num_read;
    int timeout = MAX30102_TIMEOUT_MS;
    
    /* Wait for at least one sample to be available */
    while (timeout > 0) {
        ret = max30102_read_samples(dev, sample, 1, &num_read);
        if (ret != 0) {
            return ret;
        }
        
        if (num_read > 0) {
            return 0;
        }
        
        k_sleep(K_MSEC(5));
        timeout -= 5;
    }
    
    LOG_WRN("Timeout waiting for sample");
    return -ETIMEDOUT;
}

int max30102_read_temperature(struct max30102_device *dev, float *temperature)
{
    int ret;
    uint8_t temp_int, temp_frac;
    uint8_t temp_config;
    int timeout = 100;
    
    /* Trigger temperature measurement */
    ret = max30102_write_reg(dev, MAX30102_REG_TEMP_CONFIG, 0x01);
    if (ret != 0) {
        return ret;
    }
    
    /* Wait for measurement to complete */
    do {
        k_sleep(K_MSEC(1));
        ret = max30102_read_reg(dev, MAX30102_REG_TEMP_CONFIG, &temp_config);
        if (ret != 0) {
            return ret;
        }
        timeout--;
    } while ((temp_config & 0x01) && (timeout > 0));
    
    if (timeout == 0) {
        LOG_ERR("Temperature measurement timeout");
        return -ETIMEDOUT;
    }
    
    /* Read temperature registers */
    ret = max30102_read_reg(dev, MAX30102_REG_TEMP_INT, &temp_int);
    if (ret != 0) return ret;
    
    ret = max30102_read_reg(dev, MAX30102_REG_TEMP_FRAC, &temp_frac);
    if (ret != 0) return ret;
    
    /* Calculate temperature: integer part is signed 8-bit, fraction is 0.0625°C steps */
    *temperature = (float)((int8_t)temp_int) + (temp_frac * 0.0625f);
    
    LOG_DBG("Temperature: %.2f °C", *temperature);
    return 0;
}

int max30102_shutdown(struct max30102_device *dev)
{
    LOG_DBG("Entering shutdown mode");
    return max30102_modify_reg(dev, MAX30102_REG_MODE_CONFIG,
                               MAX30102_SHUTDOWN_BIT, MAX30102_SHUTDOWN_BIT);
}

int max30102_wakeup(struct max30102_device *dev)
{
    LOG_DBG("Waking up from shutdown");
    return max30102_modify_reg(dev, MAX30102_REG_MODE_CONFIG,
                               MAX30102_SHUTDOWN_BIT, 0x00);
}

int max30102_set_led_current(struct max30102_device *dev,
                             float led1_ma, float led2_ma)
{
    int ret;
    
    /* Convert mA to register value (0.2 mA per step) */
    uint8_t led1_reg = (uint8_t)(led1_ma / 0.2f);
    uint8_t led2_reg = (uint8_t)(led2_ma / 0.2f);
    
    /* Clamp to valid range */
    if (led1_reg > 0xFF) led1_reg = 0xFF;
    if (led2_reg > 0xFF) led2_reg = 0xFF;
    
    LOG_INF("Setting LED currents: Red=%.1f mA (0x%02X), IR=%.1f mA (0x%02X)",
            led1_ma, led1_reg, led2_ma, led2_reg);
    
    ret = max30102_write_reg(dev, MAX30102_REG_LED1_PA, led1_reg);
    if (ret != 0) return ret;
    
    ret = max30102_write_reg(dev, MAX30102_REG_LED2_PA, led2_reg);
    if (ret != 0) return ret;
    
    dev->config.led1_current = led1_reg;
    dev->config.led2_current = led2_reg;
    
    return 0;
}