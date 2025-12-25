/**
 * @file main.c
 * @brief MAX30102 PPG Sensor Demo Application
 * 
 * This demo application demonstrates:
 * 1. Initializing the MAX30102 sensor
 * 2. Configuring for heart rate measurement
 * 3. Reading raw PPG data from the FIFO
 * 4. Basic signal visualization via logging
 * 5. Simple peak detection for heart rate estimation
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "max30102.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/*
 * ============================================================================
 * Configuration
 * ============================================================================
 */

/* I2C device */
#define I2C_NODE DT_NODELABEL(i2c0)

/* LED for heartbeat visualization */
#define LED0_NODE DT_ALIAS(led0)

/* Sample buffer size */
#define SAMPLE_BUFFER_SIZE      200

/* Minimum expected heart rate (BPM) */
#define MIN_HR_BPM              30

/* Maximum expected heart rate (BPM) */
#define MAX_HR_BPM              220

/*
 * ============================================================================
 * Global Variables
 * ============================================================================
 */

/* Device handles */
static struct max30102_device ppg_sensor;
static const struct device *i2c_dev;
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* Sample buffer for analysis */
static struct max30102_sample sample_buffer[SAMPLE_BUFFER_SIZE];
static uint16_t sample_count = 0;

/* Heart rate calculation variables */
static int64_t last_peak_time = 0;
static uint32_t last_peak_value = 0;
static float current_hr_bpm = 0.0f;
static bool peak_detected = false;

/*
 * ============================================================================
 * Signal Processing Functions
 * ============================================================================
 */

/**
 * @brief Simple moving average filter
 * 
 * This helps smooth out noise in the PPG signal.
 * 
 * @param buffer Array of values
 * @param len Number of values
 * @param window Window size for averaging
 * @return Averaged value
 */
static uint32_t moving_average(uint32_t *buffer, int len, int window)
{
    if (len < window) {
        window = len;
    }
    
    uint64_t sum = 0;
    int start = len - window;
    
    for (int i = start; i < len; i++) {
        sum += buffer[i];
    }
    
    return (uint32_t)(sum / window);
}

/**
 * @brief Detect peaks in the IR signal for heart rate calculation
 * 
 * This is a simple threshold-based peak detector.
 * For production, you'd want a more robust algorithm.
 * 
 * @param sample Current sample
 * @param threshold Adaptive threshold
 * @return true if peak detected
 */
static bool detect_peak(struct max30102_sample *sample, uint32_t threshold)
{
    static uint32_t prev_value = 0;
    static uint32_t prev_prev_value = 0;
    static bool rising = false;
    
    uint32_t current = sample->ir;
    bool is_peak = false;
    
    /* Detect local maximum:
     * - Previous value was higher than both current and the one before
     * - Previous value is above threshold
     */
    if (prev_value > prev_prev_value && prev_value > current) {
        /* We were rising, now falling = peak */
        if (rising && prev_value > threshold) {
            is_peak = true;
        }
        rising = false;
    } else if (current > prev_value) {
        rising = true;
    }
    
    prev_prev_value = prev_value;
    prev_value = current;
    
    return is_peak;
}

/**
 * @brief Calculate heart rate from inter-beat interval
 * 
 * @param ibi_ms Inter-beat interval in milliseconds
 * @return Heart rate in BPM, or 0 if invalid
 */
static float calculate_hr_from_ibi(int64_t ibi_ms)
{
    if (ibi_ms <= 0) {
        return 0.0f;
    }
    
    /* HR (BPM) = 60000 ms / IBI (ms) */
    float hr = 60000.0f / (float)ibi_ms;
    
    /* Validate range */
    if (hr < MIN_HR_BPM || hr > MAX_HR_BPM) {
        return 0.0f;  /* Invalid */
    }
    
    return hr;
}

/**
 * @brief Process a PPG sample and detect heartbeats
 * 
 * @param sample Pointer to the sample
 */
static void process_sample(struct max30102_sample *sample)
{
    static uint32_t ir_buffer[32];
    static uint8_t ir_index = 0;
    static uint8_t ir_count = 0;
    static uint32_t baseline = 0;
    
    /* Add to circular buffer */
    ir_buffer[ir_index] = sample->ir;
    ir_index = (ir_index + 1) % 32;
    if (ir_count < 32) ir_count++;
    
    /* Calculate baseline (moving average) */
    if (ir_count >= 8) {
        baseline = moving_average(ir_buffer, ir_count, 8);
    }
    
    /* Calculate adaptive threshold (slightly above baseline) */
    uint32_t threshold = baseline + (baseline / 50);  /* 2% above baseline */
    
    /* Detect peak */
    if (detect_peak(sample, threshold)) {
        int64_t now = sample->timestamp;
        
        if (last_peak_time > 0) {
            int64_t ibi = now - last_peak_time;
            
            /* Calculate HR if IBI is reasonable (300-2000ms = 30-200 BPM) */
            if (ibi > 300 && ibi < 2000) {
                float new_hr = calculate_hr_from_ibi(ibi);
                
                if (new_hr > 0) {
                    /* Simple low-pass filter on HR */
                    if (current_hr_bpm == 0) {
                        current_hr_bpm = new_hr;
                    } else {
                        current_hr_bpm = 0.8f * current_hr_bpm + 0.2f * new_hr;
                    }
                    
                    /* Toggle LED on heartbeat */
                    gpio_pin_toggle_dt(&led);
                    
                    LOG_INF("♥ BEAT | IBI: %lld ms | HR: %.1f BPM", 
                            ibi, current_hr_bpm);
                }
            }
        }
        
        last_peak_time = now;
        last_peak_value = sample->ir;
        peak_detected = true;
    } else {
        peak_detected = false;
    }
}

/**
 * @brief Print a simple ASCII visualization of the PPG signal
 * 
 * @param sample The sample to visualize
 */
static void visualize_sample(struct max30102_sample *sample)
{
    /* Scale the IR value to a bar chart (0-50 characters) */
    /* Typical IR values range from ~50000 to ~150000 depending on setup */
    
    static uint32_t min_val = UINT32_MAX;
    static uint32_t max_val = 0;
    
    /* Track min/max for auto-scaling */
    if (sample->ir < min_val && sample->ir > 0) min_val = sample->ir;
    if (sample->ir > max_val) max_val = sample->ir;
    
    /* Calculate bar length */
    uint32_t range = max_val - min_val;
    if (range < 1000) range = 1000;  /* Minimum range */
    
    int bar_len = ((sample->ir - min_val) * 50) / range;
    if (bar_len < 0) bar_len = 0;
    if (bar_len > 50) bar_len = 50;
    
    /* Build the bar string */
    char bar[52];
    for (int i = 0; i < 50; i++) {
        if (i < bar_len) {
            bar[i] = peak_detected ? '*' : '=';
        } else {
            bar[i] = ' ';
        }
    }
    bar[50] = '|';
    bar[51] = '\0';
    
    /* Print with values */
    LOG_INF("IR:%6u |%s HR:%.0f", 
            sample->ir, bar, current_hr_bpm);
}

/*
 * ============================================================================
 * Initialization Functions
 * ============================================================================
 */

/**
 * @brief Initialize all hardware
 */
static int hardware_init(void)
{
    int ret;
    
    LOG_INF("╔════════════════════════════════════════╗");
    LOG_INF("║       MAX30102 PPG SENSOR DEMO         ║");
    LOG_INF("╚════════════════════════════════════════╝");
    
    /* Initialize LED */
    if (!gpio_is_ready_dt(&led)) {
        LOG_ERR("LED device not ready");
        return -ENODEV;
    }
    
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED: %d", ret);
        return ret;
    }
    LOG_INF("LED initialized (will blink on heartbeat)");
    
    /* Initialize I2C */
    i2c_dev = DEVICE_DT_GET(I2C_NODE);
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }
    LOG_INF("I2C bus ready");
    
    /* Initialize MAX30102 */
    ret = max30102_init(&ppg_sensor, i2c_dev);
    if (ret != 0) {
        LOG_ERR("MAX30102 initialization failed: %d", ret);
        return ret;
    }
    
    /* Configure with default settings */
    ret = max30102_configure_default(&ppg_sensor);
    if (ret != 0) {
        LOG_ERR("MAX30102 configuration failed: %d", ret);
        return ret;
    }
    
    /* Read and display temperature */
    float temp;
    ret = max30102_read_temperature(&ppg_sensor, &temp);
    if (ret == 0) {
        LOG_INF("Sensor temperature: %.2f °C", temp);
    }
    
    LOG_INF("Hardware initialization complete");
    LOG_INF("");
    
    return 0;
}

/*
 * ============================================================================
 * Main Application
 * ============================================================================
 */

/**
 * @brief Main entry point
 */
int main(void)
{
    int ret;
    struct max30102_sample samples[16];
    uint8_t num_samples;
    uint32_t total_samples = 0;
    int64_t last_status_time = 0;
    
    /* Initialize hardware */
    ret = hardware_init();
    if (ret != 0) {
        LOG_ERR("Initialization failed!");
        
        /* Blink LED rapidly to indicate error */
        while (1) {
            gpio_pin_toggle_dt(&led);
            k_sleep(K_MSEC(100));
        }
    }
    
    LOG_INF("╔════════════════════════════════════════╗");
    LOG_INF("║          STARTING DATA CAPTURE         ║");
    LOG_INF("╠════════════════════════════════════════╣");
    LOG_INF("║  Place your finger on the sensor...    ║");
    LOG_INF("║  LED will blink with each heartbeat    ║");
    LOG_INF("╚════════════════════════════════════════╝");
    LOG_INF("");
    
    /* Give user time to place finger */
    k_sleep(K_SECONDS(2));
    
    /* Clear FIFO before starting */
    max30102_clear_fifo(&ppg_sensor);
    
    /* Main sampling loop */
    while (1) {
        /* Read available samples from FIFO */
        ret = max30102_read_samples(&ppg_sensor, samples, 16, &num_samples);
        
        if (ret != 0) {
            LOG_ERR("Failed to read samples: %d", ret);
            k_sleep(K_MSEC(100));
            continue;
        }
        
        if (num_samples == 0) {
            /* No samples available, wait a bit */
            k_sleep(K_MSEC(10));
            continue;
        }
        
        /* Process each sample */
        for (uint8_t i = 0; i < num_samples; i++) {
            total_samples++;
            
            /* Check for valid signal (finger present) */
            if (samples[i].ir < 10000) {
                /* Signal too low - finger probably not on sensor */
                if (total_samples % 100 == 0) {
                    LOG_WRN("Low signal - please place finger on sensor");
                }
                continue;
            }
            
            /* Process the sample for heart rate detection */
            process_sample(&samples[i]);
            
            /* Visualize every 4th sample (reduce log spam) */
            if (total_samples % 4 == 0) {
                visualize_sample(&samples[i]);
            }
        }
        
        /* Print periodic status */
        int64_t now = k_uptime_get();
        if (now - last_status_time > 10000) {  /* Every 10 seconds */
            LOG_INF("────────────────────────────────────────");
            LOG_INF("Status: %u total samples, Current HR: %.1f BPM",
                    total_samples, current_hr_bpm);
            
            /* Read temperature periodically */
            float temp;
            if (max30102_read_temperature(&ppg_sensor, &temp) == 0) {
                LOG_INF("Sensor temperature: %.2f °C", temp);
            }
            
            /* Get FIFO status */
            struct max30102_fifo_status fifo_status;
            if (max30102_get_fifo_status(&ppg_sensor, &fifo_status) == 0) {
                if (fifo_status.overflow_cnt > 0) {
                    LOG_WRN("FIFO overflow count: %d", fifo_status.overflow_cnt);
                }
            }
            
            LOG_INF("────────────────────────────────────────");
            last_status_time = now;
        }
        
        /* Small delay to prevent tight loop */
        k_sleep(K_MSEC(5));
    }
    
    return 0;
}