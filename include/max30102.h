/**
 * @file max30102.h
 * @brief MAX30102 Pulse Oximeter and Heart Rate Sensor Driver
 * 
 * This driver provides an interface to the MAX30102 sensor for:
 * - Reading raw PPG (photoplethysmography) data
 * - Configuring LED intensity, sample rate, and ADC resolution
 * - Basic heart rate detection from PPG signals
 */

#ifndef MAX30102_H
#define MAX30102_H

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * Register Definitions
 * ============================================================================
 */

/* Interrupt Status/Enable Registers */
#define MAX30102_REG_INT_STATUS_1       0x00    /* Interrupt status 1 */
#define MAX30102_REG_INT_STATUS_2       0x01    /* Interrupt status 2 */
#define MAX30102_REG_INT_ENABLE_1       0x02    /* Interrupt enable 1 */
#define MAX30102_REG_INT_ENABLE_2       0x03    /* Interrupt enable 2 */

/* FIFO Registers */
#define MAX30102_REG_FIFO_WR_PTR        0x04    /* FIFO write pointer */
#define MAX30102_REG_FIFO_OVF_CNT       0x05    /* FIFO overflow counter */
#define MAX30102_REG_FIFO_RD_PTR        0x06    /* FIFO read pointer */
#define MAX30102_REG_FIFO_DATA          0x07    /* FIFO data register */

/* Configuration Registers */
#define MAX30102_REG_FIFO_CONFIG        0x08    /* FIFO configuration */
#define MAX30102_REG_MODE_CONFIG        0x09    /* Mode configuration */
#define MAX30102_REG_SPO2_CONFIG        0x0A    /* SpO2 configuration */

/* LED Pulse Amplitude Registers */
#define MAX30102_REG_LED1_PA            0x0C    /* LED1 (Red) pulse amplitude */
#define MAX30102_REG_LED2_PA            0x0D    /* LED2 (IR) pulse amplitude */

/* Multi-LED Mode Control */
#define MAX30102_REG_MULTI_LED_CTRL1    0x11    /* Multi-LED mode control 1 */
#define MAX30102_REG_MULTI_LED_CTRL2    0x12    /* Multi-LED mode control 2 */

/* Temperature Registers */
#define MAX30102_REG_TEMP_INT           0x1F    /* Temperature integer */
#define MAX30102_REG_TEMP_FRAC          0x20    /* Temperature fraction */
#define MAX30102_REG_TEMP_CONFIG        0x21    /* Temperature configuration */

/* Part ID Registers */
#define MAX30102_REG_REV_ID             0xFE    /* Revision ID */
#define MAX30102_REG_PART_ID            0xFF    /* Part ID (should be 0x15) */

/* Expected Part ID value */
#define MAX30102_EXPECTED_PART_ID       0x15

/*
 * ============================================================================
 * Configuration Enumerations
 * ============================================================================
 */

/**
 * @brief Operating modes
 * 
 * The MAX30102 can operate in different modes depending on what
 * measurements you need.
 */
typedef enum {
    MAX30102_MODE_HR_ONLY   = 0x02,    /* Heart Rate mode (Red LED only) */
    MAX30102_MODE_SPO2      = 0x03,    /* SpO2 mode (Red + IR LEDs) */
    MAX30102_MODE_MULTI_LED = 0x07,    /* Multi-LED mode (programmable) */
} max30102_mode_t;

/**
 * @brief Sample rates (samples per second)
 * 
 * Higher rates give more resolution but fill the FIFO faster.
 * For heart rate, 50-100 Hz is typically sufficient.
 */
typedef enum {
    MAX30102_SR_50      = 0x00,    /* 50 samples/sec */
    MAX30102_SR_100     = 0x01,    /* 100 samples/sec */
    MAX30102_SR_200     = 0x02,    /* 200 samples/sec */
    MAX30102_SR_400     = 0x03,    /* 400 samples/sec */
    MAX30102_SR_800     = 0x04,    /* 800 samples/sec */
    MAX30102_SR_1000    = 0x05,    /* 1000 samples/sec */
    MAX30102_SR_1600    = 0x06,    /* 1600 samples/sec */
    MAX30102_SR_3200    = 0x07,    /* 3200 samples/sec */
} max30102_sample_rate_t;

/**
 * @brief LED pulse width / ADC resolution
 * 
 * Longer pulse width = more light = better SNR but more power.
 * Also determines ADC resolution.
 */
typedef enum {
    MAX30102_PW_69      = 0x00,    /* 69 µs pulse, 15-bit ADC */
    MAX30102_PW_118     = 0x01,    /* 118 µs pulse, 16-bit ADC */
    MAX30102_PW_215     = 0x02,    /* 215 µs pulse, 17-bit ADC */
    MAX30102_PW_411     = 0x03,    /* 411 µs pulse, 18-bit ADC */
} max30102_pulse_width_t;

/**
 * @brief ADC full-scale range
 * 
 * Determines the maximum signal level before saturation.
 */
typedef enum {
    MAX30102_ADC_2048   = 0x00,    /* 2048 nA full scale */
    MAX30102_ADC_4096   = 0x01,    /* 4096 nA full scale */
    MAX30102_ADC_8192   = 0x02,    /* 8192 nA full scale */
    MAX30102_ADC_16384  = 0x03,    /* 16384 nA full scale */
} max30102_adc_range_t;

/**
 * @brief Sample averaging (FIFO)
 * 
 * Averages multiple samples together to reduce noise.
 * Reduces effective sample rate by the averaging factor.
 */
typedef enum {
    MAX30102_AVG_1      = 0x00,    /* No averaging */
    MAX30102_AVG_2      = 0x01,    /* 2 samples averaged */
    MAX30102_AVG_4      = 0x02,    /* 4 samples averaged */
    MAX30102_AVG_8      = 0x03,    /* 8 samples averaged */
    MAX30102_AVG_16     = 0x04,    /* 16 samples averaged */
    MAX30102_AVG_32     = 0x05,    /* 32 samples averaged */
} max30102_sample_avg_t;

/*
 * ============================================================================
 * Data Structures
 * ============================================================================
 */

/**
 * @brief Sensor configuration structure
 */
struct max30102_config {
    max30102_mode_t         mode;           /* Operating mode */
    max30102_sample_rate_t  sample_rate;    /* Sample rate */
    max30102_pulse_width_t  pulse_width;    /* LED pulse width */
    max30102_adc_range_t    adc_range;      /* ADC range */
    max30102_sample_avg_t   sample_avg;     /* Sample averaging */
    uint8_t                 led1_current;   /* Red LED current (0-255, 0.2mA/step) */
    uint8_t                 led2_current;   /* IR LED current (0-255, 0.2mA/step) */
    bool                    fifo_rollover;  /* Enable FIFO rollover */
};

/**
 * @brief Raw sample data from sensor
 */
struct max30102_sample {
    uint32_t red;       /* Red LED reading (18-bit, right-aligned) */
    uint32_t ir;        /* IR LED reading (18-bit, right-aligned) */
    int64_t  timestamp; /* Kernel timestamp in milliseconds */
};

/**
 * @brief FIFO status information
 */
struct max30102_fifo_status {
    uint8_t write_ptr;      /* Current write pointer */
    uint8_t read_ptr;       /* Current read pointer */
    uint8_t overflow_cnt;   /* Overflow counter */
    uint8_t available;      /* Number of samples available */
};

/**
 * @brief Device handle structure
 */
struct max30102_device {
    const struct device *i2c;           /* I2C device handle */
    struct max30102_config config;      /* Current configuration */
    bool initialized;                   /* Initialization status */
    uint8_t part_id;                    /* Part ID read from device */
    uint8_t rev_id;                     /* Revision ID read from device */
};

/*
 * ============================================================================
 * Function Prototypes
 * ============================================================================
 */

/**
 * @brief Initialize the MAX30102 sensor
 * 
 * This function:
 * - Verifies I2C communication
 * - Reads and validates the Part ID
 * - Performs a software reset
 * - Leaves the device in a known state
 * 
 * @param dev Pointer to device handle structure
 * @param i2c_dev Zephyr I2C device handle
 * @return 0 on success, negative error code on failure
 */
int max30102_init(struct max30102_device *dev, const struct device *i2c_dev);

/**
 * @brief Configure the sensor with specified parameters
 * 
 * @param dev Pointer to initialized device handle
 * @param config Pointer to configuration structure
 * @return 0 on success, negative error code on failure
 */
int max30102_configure(struct max30102_device *dev, 
                       const struct max30102_config *config);

/**
 * @brief Apply default configuration suitable for heart rate monitoring
 * 
 * Default settings:
 * - SpO2 mode (both LEDs)
 * - 100 Hz sample rate
 * - 411 µs pulse width (18-bit)
 * - 4096 nA ADC range
 * - 4-sample averaging
 * - 7mA LED current
 * 
 * @param dev Pointer to initialized device handle
 * @return 0 on success, negative error code on failure
 */
int max30102_configure_default(struct max30102_device *dev);

/**
 * @brief Read available samples from the FIFO
 * 
 * @param dev Pointer to initialized device handle
 * @param samples Array to store samples
 * @param max_samples Maximum number of samples to read
 * @param num_read Pointer to store actual number of samples read
 * @return 0 on success, negative error code on failure
 */
int max30102_read_samples(struct max30102_device *dev,
                          struct max30102_sample *samples,
                          uint8_t max_samples,
                          uint8_t *num_read);

/**
 * @brief Read a single sample (blocking until available)
 * 
 * @param dev Pointer to initialized device handle
 * @param sample Pointer to store the sample
 * @return 0 on success, negative error code on failure
 */
int max30102_read_sample(struct max30102_device *dev,
                         struct max30102_sample *sample);

/**
 * @brief Get FIFO status
 * 
 * @param dev Pointer to initialized device handle
 * @param status Pointer to store FIFO status
 * @return 0 on success, negative error code on failure
 */
int max30102_get_fifo_status(struct max30102_device *dev,
                             struct max30102_fifo_status *status);

/**
 * @brief Clear the FIFO
 * 
 * @param dev Pointer to initialized device handle
 * @return 0 on success, negative error code on failure
 */
int max30102_clear_fifo(struct max30102_device *dev);

/**
 * @brief Read die temperature
 * 
 * @param dev Pointer to initialized device handle
 * @param temperature Pointer to store temperature in °C
 * @return 0 on success, negative error code on failure
 */
int max30102_read_temperature(struct max30102_device *dev, float *temperature);

/**
 * @brief Put device into shutdown mode (low power)
 * 
 * @param dev Pointer to initialized device handle
 * @return 0 on success, negative error code on failure
 */
int max30102_shutdown(struct max30102_device *dev);

/**
 * @brief Wake device from shutdown mode
 * 
 * @param dev Pointer to initialized device handle
 * @return 0 on success, negative error code on failure
 */
int max30102_wakeup(struct max30102_device *dev);

/**
 * @brief Perform software reset
 * 
 * @param dev Pointer to initialized device handle
 * @return 0 on success, negative error code on failure
 */
int max30102_reset(struct max30102_device *dev);

/**
 * @brief Set LED current
 * 
 * @param dev Pointer to initialized device handle
 * @param led1_ma Red LED current in mA (0-51)
 * @param led2_ma IR LED current in mA (0-51)
 * @return 0 on success, negative error code on failure
 */
int max30102_set_led_current(struct max30102_device *dev,
                             float led1_ma, float led2_ma);

#ifdef __cplusplus
}
#endif

#endif /* MAX30102_H */