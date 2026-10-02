/*
  Self-Driving RC Car - pure C, ESP-IDF, for the Arduino Nano ESP32 (ESP32-S3)
  LIDAR VERSION: three VL53L1X time-of-flight sensors on one shared I2C
  bus, replacing the three HY-SRF05 ultrasonic sensors from the previous
  version. Wiring matches the diagram discussed earlier:

    Arduino label -> ESP32-S3 GPIO      Used for
    A4  -> GPIO11                       I2C SDA (shared bus, all 3 sensors)
    A5  -> GPIO12                       I2C SCL (shared bus, all 3 sensors)
    D2  -> GPIO5                        Sensor 1 (Left)  XSHUT
    D4  -> GPIO7                        Sensor 2 (Mid)   XSHUT
    D8  -> GPIO17                       Sensor 3 (Right) XSHUT
    D9  -> GPIO18                       Steering servo (PWM)
    D3  -> GPIO6                        Drive motor DRV8871 IN1 (PWM)
    D11 -> GPIO38                       Drive motor DRV8871 IN2 (PWM)
    A0  -> GPIO1                        Start trigger (digital in)
    A1  -> GPIO2                        Battery sense (analog in)
    D6  -> GPIO9                        Status LED

  D10/D7/D5 (the old ECHO pins) are no longer used at all - freed up.

  *** THIS FILE NEEDS ONE MORE THING FROM YOU BEFORE IT BUILDS ***
  This deliberately does NOT reimplement ST's VL53L1X_api.c/.h - that file
  contains a large (~135-register) proprietary default-configuration
  table that the sensor needs for correct calibration, and reconstructing
  that from memory risks getting it subtly wrong in something that's
  actively steering a car near walls. Get the official, unmodified
  VL53L1X_api.c and VL53L1X_api.h from ST's STSW-IMG009 package (the
  "VL53L1X ULD API") on st.com, and drop both files into this same
  folder unchanged. This file only supplies:
    - vl53l1x_platform.c/.h - the I2C glue those two files call into
    - main.c (this file) - the XSHUT bring-up sequence and the same
      drive state machine as the ultrasonic version, just calling the
      ULD API's high-level functions (VL53L1X_SensorInit,
      VL53L1X_StartRanging, VL53L1X_CheckForDataReady, VL53L1X_GetDistance,
      VL53L1X_ClearInterrupt) instead of the old ultrasonic trigger/echo
      code.

  Everything below this point - PID, turn-direction learning, kickstart,
  recovery, stuck detection - is UNCHANGED from the updated AVR/ultrasonic
  version. Only the sensor-reading layer and pin setup changed.

  One deliberate change: the old code added three fixed 10ms delays
  between ultrasonic pings to let each HY-SRF05's echo die down before
  firing the next one (avoiding acoustic cross-talk). VL53L1X sensors
  communicate over I2C, not sound, so there's no cross-talk to wait out -
  those delays are removed here. Each sensor also free-runs its own
  ranging cycle continuously once VL53L1X_StartRanging() is called, so
  polling all three back-to-back with no delay lets the loop run close to
  each sensor's own ~20ms ranging period rather than the sum of all three.
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_log.h"

#define TAG "rc_car"

#include "vl53l1x_platform.h"
#include "VL53L1X_api.h" // from ST's official STSW-IMG009 package - see the note above

// ---------- Pin assignments ----------
#define PIN_I2C_SDA       11  // A4
#define PIN_I2C_SCL       12  // A5

#define PIN_XSHUT_LEFT    5   // D2
#define PIN_XSHUT_MID     7   // D4
#define PIN_XSHUT_RIGHT   17  // D8

#define PIN_SERVO         18  // D9

#define PIN_MOTOR_IN1     6   // D3
#define PIN_MOTOR_IN2     38  // D11

#define PIN_START_TRIGGER 1   // A0
#define PIN_LED           9   // D6

#define BATTERY_ADC_UNIT     ADC_UNIT_1
#define BATTERY_ADC_CHANNEL  ADC_CHANNEL_1 // GPIO2 (A1) on ESP32-S3's ADC1

// ---------- I2C addresses assigned during bring-up ----------
// The VL53L1X's factory-default address is 0x29; each sensor is moved to
// one of these unique addresses (one at a time, while the other two are
// still held in reset via XSHUT) before any of them range simultaneously.
#define VL53L1X_DEFAULT_ADDR 0x29
#define VL53L1X_ADDR_LEFT    0x30
#define VL53L1X_ADDR_MID     0x31
#define VL53L1X_ADDR_RIGHT   0x32

#define VL53L1X_DISTANCE_MODE_SHORT 1
#define VL53L1X_TIMING_BUDGET_MS    20
#define VL53L1X_INTER_MEASUREMENT_MS 20

// How long to wait for a "data ready" flag before giving up on a single
// read (treated the same as an ultrasonic timeout was: fall back to
// MAX_DISTANCE_CM, i.e. "clear"). Set well above the sensor's own ~20ms
// ranging period so a normal cycle never trips it.
#define VL53L1X_DATA_READY_TIMEOUT_US 60000UL

// ---------- LEDC (PWM) assignments ----------
#define LEDC_MODE            LEDC_LOW_SPEED_MODE // only mode available on ESP32-S3

#define SERVO_LEDC_TIMER      LEDC_TIMER_0
#define SERVO_LEDC_CHANNEL    LEDC_CHANNEL_0
#define SERVO_LEDC_FREQ_HZ    50
#define SERVO_LEDC_RES_BITS   LEDC_TIMER_14_BIT           // ESP32-S3 LEDC max
#define SERVO_LEDC_DUTY_MAX   ((1u << SERVO_LEDC_RES_BITS) - 1u) // derived, not hardcoded
#define SERVO_PERIOD_US       20000u // 1 / 50Hz

#define MOTOR_LEDC_TIMER      LEDC_TIMER_1
#define MOTOR_LEDC_CHANNEL_1  LEDC_CHANNEL_1 // IN1
#define MOTOR_LEDC_CHANNEL_2  LEDC_CHANNEL_2 // IN2
#define MOTOR_LEDC_FREQ_HZ    1000
#define MOTOR_LEDC_RES_BITS   LEDC_TIMER_8_BIT // 0-255 duty, same scale as the original

// ---------- Tuning constants (unchanged from the updated AVR version) ----------
#define MAX_DISTANCE_CM        150u
#define STOP_DISTANCE_CM       10u
#define CORNER_SLOW_DISTANCE_CM 80u
#define CORNER_TRIGGER_DISTANCE_CM 60u

#define TURN_SPEED              80u
#define TURN_DURATION_MS        600u
#define TURN_MAX_EXTRA_MS       800u

#define SERVO_MIN_US      1000u
#define SERVO_MAX_US      2000u
#define SERVO_CENTER_DEG  90u
#define SERVO_MIN_DEG     45u
#define SERVO_MAX_DEG     135u

#define SERVO_INVERT      0u // servo direction non-inverted

#define DRIVE_SPEED       255u
#define MIN_SPEED         200u
#define TURN_SPEED_REDUCTION 50u
#define REVERSE_SPEED     100u

#define KICKSTART_SPEED    255u
#define KICKSTART_MS       120u
#define RECOVERY_SETTLE_MS 100u

#define REVERSE_TIME_MS        600u
#define WIGGLE_HALF_PERIOD_MS  200u

#define STUCK_CHECK_WINDOW_MS   1000u
#define STUCK_DISTANCE_DELTA_CM 3u

#define BATTERY_LOW_THRESHOLD 860

// ---------- micros() ----------
static inline uint32_t micros(void) {
    return (uint32_t)esp_timer_get_time();
}

static void delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// ---------- GPIO setup ----------
static void gpio_setup(void) {
    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL << PIN_XSHUT_LEFT) | (1ULL << PIN_XSHUT_MID) |
                         (1ULL << PIN_XSHUT_RIGHT) | (1ULL << PIN_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_conf);

    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << PIN_START_TRIGGER),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in_conf);

    // Hold every sensor in reset until the bring-up sequence wakes them
    // one at a time.
    gpio_set_level(PIN_XSHUT_LEFT, 0);
    gpio_set_level(PIN_XSHUT_MID, 0);
    gpio_set_level(PIN_XSHUT_RIGHT, 0);
    gpio_set_level(PIN_LED, 0);
}

// ---------- I2C bus setup ----------
static i2c_master_bus_handle_t i2c_bus_setup(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true, // testing with these on in case external pull-ups aren't reaching the bus
    };
    i2c_master_bus_handle_t bus = NULL;
    i2c_new_master_bus(&bus_config, &bus);
    vl53l1x_platform_init_bus(bus);
    return bus;
}

// ---------- VL53L1X bring-up: wake, re-address, and start each sensor ----------
// Only one sensor is released from reset (XSHUT high) at a time, so each
// one is uniquely reachable at the factory-default address (0x29) while
// it's given its own permanent address - this is the standard way to run
// several I2C devices that all share one fixed default address.
static void vl53l1x_bring_up_one(gpio_num_t xshut_pin, uint8_t new_addr) {
    gpio_set_level(xshut_pin, 1);
    delay_ms(2); // sensor boot time after leaving reset

    VL53L1X_ERROR status = vl53l1x_platform_set_address(VL53L1X_DEFAULT_ADDR, new_addr);
    ESP_LOGI(TAG, "0x%02X: set_address status=%d", new_addr, status);
    delay_ms(2);

    status = VL53L1X_SensorInit(new_addr);
    ESP_LOGI(TAG, "0x%02X: SensorInit status=%d", new_addr, status);

    status = VL53L1X_SetDistanceMode(new_addr, VL53L1X_DISTANCE_MODE_SHORT);
    ESP_LOGI(TAG, "0x%02X: SetDistanceMode status=%d", new_addr, status);

    status = VL53L1X_SetTimingBudgetInMs(new_addr, VL53L1X_TIMING_BUDGET_MS);
    ESP_LOGI(TAG, "0x%02X: SetTimingBudgetInMs status=%d", new_addr, status);

    status = VL53L1X_SetInterMeasurementInMs(new_addr, VL53L1X_INTER_MEASUREMENT_MS);
    ESP_LOGI(TAG, "0x%02X: SetInterMeasurementInMs status=%d", new_addr, status);

    status = VL53L1X_StartRanging(new_addr);
    ESP_LOGI(TAG, "0x%02X: StartRanging status=%d", new_addr, status);
}

static void vl53l1x_bring_up_all(void) {
    vl53l1x_bring_up_one(PIN_XSHUT_LEFT, VL53L1X_ADDR_LEFT);
    vl53l1x_bring_up_one(PIN_XSHUT_MID, VL53L1X_ADDR_MID);
    vl53l1x_bring_up_one(PIN_XSHUT_RIGHT, VL53L1X_ADDR_RIGHT);
}

// ---------- LEDC (servo + motor PWM) setup ----------
static void ledc_setup(void) {
    ledc_timer_config_t servo_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = SERVO_LEDC_TIMER,
        .duty_resolution = SERVO_LEDC_RES_BITS,
        .freq_hz = SERVO_LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&servo_timer);

    ledc_channel_config_t servo_channel = {
        .speed_mode = LEDC_MODE,
        .channel = SERVO_LEDC_CHANNEL,
        .timer_sel = SERVO_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_SERVO,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&servo_channel);

    ledc_timer_config_t motor_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = MOTOR_LEDC_TIMER,
        .duty_resolution = MOTOR_LEDC_RES_BITS,
        .freq_hz = MOTOR_LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&motor_timer);

    ledc_channel_config_t motor_channel_1 = {
        .speed_mode = LEDC_MODE,
        .channel = MOTOR_LEDC_CHANNEL_1,
        .timer_sel = MOTOR_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_MOTOR_IN1,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&motor_channel_1);

    ledc_channel_config_t motor_channel_2 = {
        .speed_mode = LEDC_MODE,
        .channel = MOTOR_LEDC_CHANNEL_2,
        .timer_sel = MOTOR_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_MOTOR_IN2,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&motor_channel_2);
}

// ---------- ADC setup (battery sense) ----------
static adc_oneshot_unit_handle_t battery_adc_handle;

static void adc_setup(void) {
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = BATTERY_ADC_UNIT,
    };
    adc_oneshot_new_unit(&unit_cfg, &battery_adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12, // ESP32-S3's ADC only supports 12-bit - unlike the classic ESP32, it can't be configured down to 10-bit
        .atten = ADC_ATTEN_DB_12,
    };
    adc_oneshot_config_channel(battery_adc_handle, BATTERY_ADC_CHANNEL, &chan_cfg);
}

// ---------- Servo ----------
static void set_servo_angle(uint8_t angle_deg) {
    if (angle_deg < SERVO_MIN_DEG) angle_deg = SERVO_MIN_DEG;
    if (angle_deg > SERVO_MAX_DEG) angle_deg = SERVO_MAX_DEG;

    uint32_t span_deg = SERVO_MAX_DEG - SERVO_MIN_DEG;
    uint32_t span_us  = SERVO_MAX_US - SERVO_MIN_US;
    uint32_t offset_us = (uint32_t)(angle_deg - SERVO_MIN_DEG) * span_us / span_deg;

#if SERVO_INVERT
    uint32_t pulse_us = SERVO_MAX_US - offset_us;
#else
    uint32_t pulse_us = SERVO_MIN_US + offset_us;
#endif

    uint32_t duty = (uint32_t)((uint64_t)pulse_us * SERVO_LEDC_DUTY_MAX / SERVO_PERIOD_US);
    ledc_set_duty(LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, SERVO_LEDC_CHANNEL);
}

// ---------- Motor (DRV8871 via LEDC PWM) ----------
static uint8_t kickstart_needed = 1;

static inline void motor_pwm(uint8_t in1, uint8_t in2) {
    ledc_set_duty(LEDC_MODE, MOTOR_LEDC_CHANNEL_1, in1);
    ledc_update_duty(LEDC_MODE, MOTOR_LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_MODE, MOTOR_LEDC_CHANNEL_2, in2);
    ledc_update_duty(LEDC_MODE, MOTOR_LEDC_CHANNEL_2);
}

static void drive_forward(uint8_t speed) {
    if (kickstart_needed && speed > 0) {
        motor_pwm(KICKSTART_SPEED, 0);
        delay_ms(KICKSTART_MS);
        kickstart_needed = 0;
    }
    motor_pwm(speed, 0);
}

static void drive_reverse(uint8_t speed) {
    if (kickstart_needed && speed > 0) {
        motor_pwm(0, KICKSTART_SPEED);
        delay_ms(KICKSTART_MS);
        kickstart_needed = 0;
    }
    motor_pwm(0, speed);
}

static void motor_stop(void) {
    motor_pwm(0, 0);
    kickstart_needed = 1;
}

// ---------- Distance reading (VL53L1X, replaces the old ultrasonic code) ----------
// Polls "data ready" on one sensor, reads its distance in mm, converts to
// cm (everything downstream - PID, thresholds - is tuned in cm and stays
// unchanged), and clears the interrupt so that sensor's next continuous
// measurement can complete. Falls back to MAX_DISTANCE_CM on timeout,
// matching how the old ultrasonic code treated "no echo".
static long read_distance_cm(uint16_t dev_addr) {
    uint8_t ready = 0;
    uint32_t t0 = micros();

    do {
        VL53L1X_CheckForDataReady(dev_addr, &ready);
        if (ready) break;
        if (micros() - t0 > VL53L1X_DATA_READY_TIMEOUT_US) {
            ESP_LOGI(TAG, "0x%02X: TIMED OUT waiting for data ready", dev_addr);
            return MAX_DISTANCE_CM;
        }
    } while (1);

    uint16_t distance_mm = 0;
    VL53L1X_GetDistance(dev_addr, &distance_mm);
    VL53L1X_ClearInterrupt(dev_addr);

    long distance_cm = (long)(distance_mm / 10);
    if (distance_cm > (long)MAX_DISTANCE_CM) distance_cm = MAX_DISTANCE_CM;
    ESP_LOGI(TAG, "0x%02X: %ld cm", dev_addr, distance_cm);
    return distance_cm;
}

// ---------- small helpers (map / constrain) ----------
static long constrain_long(long x, long lo, long hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static long map_long(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static float constrain_float(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// ---------- Steering PID ----------
#define STEER_KP  0.2f
#define STEER_KI  0.02f
#define STEER_KD  0.0f
#define STEER_INTEGRAL_LIMIT 150.0f

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
} SteeringPID;

static SteeringPID steer_pid = {
    .kp = STEER_KP, .ki = STEER_KI, .kd = STEER_KD,
    .integral = 0.0f, .prev_error = 0.0f
};

static void pid_reset(SteeringPID *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

static float pid_update(SteeringPID *pid, float error, float dt) {
    if (dt <= 0.0f) dt = 0.001f;
    pid->integral += error * dt;
    pid->integral = constrain_float(pid->integral, -STEER_INTEGRAL_LIMIT, STEER_INTEGRAL_LIMIT);
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}

// ---------- Sensor smoothing ----------
#define SENSOR_FILTER_ALPHA 1.0f // no filtering, matching the updated AVR version

static float filtered_diff = 0.0f;
static uint8_t filtered_diff_valid = 0;

static float filter_update(float raw_value) {
    if (!filtered_diff_valid) {
        filtered_diff = raw_value;
        filtered_diff_valid = 1;
    } else {
        filtered_diff = (SENSOR_FILTER_ALPHA * raw_value) + ((1.0f - SENSOR_FILTER_ALPHA) * filtered_diff);
    }
    return filtered_diff;
}

// ---------- Turn PID (with direction learning) ----------
#define TURN_KP  0.5f
#define TURN_KI  0.0f
#define TURN_KD  0.0f
#define TURN_DIRECTION_LOCK_THRESHOLD_CM 150L

static SteeringPID turn_pid = {
    .kp = TURN_KP, .ki = TURN_KI, .kd = TURN_KD,
    .integral = 0.0f, .prev_error = 0.0f
};

static uint8_t track_direction_known = 0;
static uint8_t track_turn_right = 0;
static long turn_direction_accum = 0;

static void reset_steering(void) {
    pid_reset(&steer_pid);
    pid_reset(&turn_pid);
    filtered_diff_valid = 0;
}

// ---------- Stuck detector ----------
static uint32_t stuck_window_start_us = 0;
static long stuck_ref_left = 0, stuck_ref_mid = 0, stuck_ref_right = 0;
static uint8_t stuck_window_valid = 0;

static void reset_stuck_detector(void) {
    stuck_window_valid = 0;
}

static uint8_t stuck_check_and_update(uint32_t now_us, long dist_left, long dist_mid, long dist_right) {
    if (!stuck_window_valid) {
        stuck_window_start_us = now_us;
        stuck_ref_left = dist_left;
        stuck_ref_mid = dist_mid;
        stuck_ref_right = dist_right;
        stuck_window_valid = 1;
        return 0;
    }

    if ((now_us - stuck_window_start_us) < ((uint32_t)STUCK_CHECK_WINDOW_MS * 1000UL)) {
        return 0;
    }

    long total_change = labs(dist_left - stuck_ref_left)
                       + labs(dist_mid  - stuck_ref_mid)
                       + labs(dist_right - stuck_ref_right);

    stuck_window_start_us = now_us;
    stuck_ref_left = dist_left;
    stuck_ref_mid = dist_mid;
    stuck_ref_right = dist_right;

    return (total_change < (long)STUCK_DISTANCE_DELTA_CM) ? 1 : 0;
}

// ---------- Recovery: reverse while wiggling the steering ----------
static void recover_reverse_and_wiggle(void) {
    motor_stop();
    drive_reverse(REVERSE_SPEED);

    uint32_t recover_start_us = micros();
    uint8_t steer_right = 1;

    while ((micros() - recover_start_us) < ((uint32_t)REVERSE_TIME_MS * 1000UL)) {
        set_servo_angle(steer_right ? SERVO_MAX_DEG : SERVO_MIN_DEG);
        delay_ms(WIGGLE_HALF_PERIOD_MS);
        steer_right = !steer_right;
    }

    motor_stop();
    delay_ms(RECOVERY_SETTLE_MS);
    set_servo_angle(SERVO_CENTER_DEG);

    reset_steering();
    reset_stuck_detector();
}

// ---------- Drive state machine ----------
typedef enum {
    STATE_DRIVE,
    STATE_TURN
} DriveState;

static uint16_t read_battery_adc(void) {
    int raw = 0;
    adc_oneshot_read(battery_adc_handle, BATTERY_ADC_CHANNEL, &raw);
    return (uint16_t)(raw >> 2); // 12-bit raw reading scaled down to the 10-bit (0-1023) range BATTERY_LOW_THRESHOLD was calibrated against
}

static void battery_low_warning(void) {
    if (read_battery_adc() < BATTERY_LOW_THRESHOLD) {
        gpio_set_level(PIN_LED, 1);
        delay_ms(250);
        gpio_set_level(PIN_LED, 0);
        delay_ms(250);
    } else {
        gpio_set_level(PIN_LED, 0);
    }
}

void app_main(void) {
    gpio_setup();
    i2c_bus_setup();
    vl53l1x_bring_up_all();
    ledc_setup();
    adc_setup();

    motor_stop();
    set_servo_angle(SERVO_CENTER_DEG);

    while (gpio_get_level(PIN_START_TRIGGER) == 0) {
        delay_ms(20);
        battery_low_warning();
    }

    uint32_t last_pid_us = micros();
    DriveState state = STATE_DRIVE;
    uint32_t turn_start_us = 0;
    reset_stuck_detector();

    while (1) {
        if (gpio_get_level(PIN_START_TRIGGER) == 0) {
            motor_stop();
            set_servo_angle(SERVO_CENTER_DEG);
            reset_steering();
            reset_stuck_detector();
            state = STATE_DRIVE;

            while (gpio_get_level(PIN_START_TRIGGER) == 0) {
                delay_ms(20);
                battery_low_warning();
            }

            last_pid_us = micros();
            continue;
        }

        // No inter-sensor delay needed here - see the note at the top of
        // this file about why the ultrasonic version's cross-talk delays
        // don't apply to I2C sensors.
        long dist_left  = read_distance_cm(VL53L1X_ADDR_LEFT);
        long dist_mid   = read_distance_cm(VL53L1X_ADDR_MID);
        long dist_right = read_distance_cm(VL53L1X_ADDR_RIGHT);

        if (dist_mid < STOP_DISTANCE_CM) {
            recover_reverse_and_wiggle();
            last_pid_us = micros();
            state = STATE_DRIVE;
            continue;
        }

        if (state == STATE_DRIVE) {
            if (stuck_check_and_update(micros(), dist_left, dist_mid, dist_right)) {
                recover_reverse_and_wiggle();
                last_pid_us = micros();
                continue;
            }

            if (dist_mid < CORNER_TRIGGER_DISTANCE_CM) {
                state = STATE_TURN;
                turn_start_us = micros();
                reset_steering();
                reset_stuck_detector();
                turn_direction_accum = 0;
                continue;
            }

            uint32_t now = micros();
            float dt = (float)(now - last_pid_us) / 1000000.0f;
            last_pid_us = now;

            long diff = constrain_long(dist_right - dist_left, -(long)MAX_DISTANCE_CM, (long)MAX_DISTANCE_CM);
            float smoothed_diff = filter_update((float)diff);
            float offset = pid_update(&steer_pid, smoothed_diff, dt);

            int angle = (int)SERVO_CENTER_DEG + (int)offset;
            if (angle < (int)SERVO_MIN_DEG) angle = (int)SERVO_MIN_DEG;
            if (angle > (int)SERVO_MAX_DEG) angle = (int)SERVO_MAX_DEG;
            set_servo_angle((uint8_t)angle);

            long steer_amount = (offset < 0.0f) ? (long)(-offset) : (long)offset;
            long turn_speed = (long)DRIVE_SPEED - map_long(
                constrain_long(steer_amount, 0, (long)(SERVO_MAX_DEG - SERVO_CENTER_DEG)),
                0, (long)(SERVO_MAX_DEG - SERVO_CENTER_DEG),
                0, (long)TURN_SPEED_REDUCTION);

            long front_speed = (dist_mid < CORNER_SLOW_DISTANCE_CM)
                ? map_long(constrain_long(dist_mid, STOP_DISTANCE_CM, CORNER_SLOW_DISTANCE_CM),
                           STOP_DISTANCE_CM, CORNER_SLOW_DISTANCE_CM,
                           MIN_SPEED, DRIVE_SPEED)
                : (long)DRIVE_SPEED;

            long speed = (turn_speed < front_speed) ? turn_speed : front_speed;
            if (speed < MIN_SPEED) speed = MIN_SPEED;

            drive_forward((uint8_t)speed);

        } else { // STATE_TURN
            uint32_t now = micros();
            float dt = (float)(now - last_pid_us) / 1000000.0f;
            last_pid_us = now;

            long diff = constrain_long(dist_right - dist_left, -(long)MAX_DISTANCE_CM, (long)MAX_DISTANCE_CM);
            float offset = pid_update(&turn_pid, (float)diff, dt);

            if (!track_direction_known) {
                turn_direction_accum += diff;
            } else {
                float magnitude = (offset < 0.0f) ? -offset : offset;
                offset = track_turn_right ? magnitude : -magnitude;
            }

            int angle = (int)SERVO_CENTER_DEG + (int)offset;
            if (angle < (int)SERVO_MIN_DEG) angle = (int)SERVO_MIN_DEG;
            if (angle > (int)SERVO_MAX_DEG) angle = (int)SERVO_MAX_DEG;
            set_servo_angle((uint8_t)angle);

            drive_forward(TURN_SPEED);

            uint32_t elapsed_ms = (micros() - turn_start_us) / 1000UL;

            if (elapsed_ms >= TURN_DURATION_MS) {
                if (dist_mid >= CORNER_TRIGGER_DISTANCE_CM ||
                    elapsed_ms >= (TURN_DURATION_MS + TURN_MAX_EXTRA_MS)) {
                    if (!track_direction_known &&
                        labs(turn_direction_accum) >= TURN_DIRECTION_LOCK_THRESHOLD_CM) {
                        track_turn_right = (turn_direction_accum >= 0);
                        track_direction_known = 1;
                    }
                    state = STATE_DRIVE;
                    reset_steering();
                    reset_stuck_detector();
                    last_pid_us = micros();
                }
            }
        }
    }
}
