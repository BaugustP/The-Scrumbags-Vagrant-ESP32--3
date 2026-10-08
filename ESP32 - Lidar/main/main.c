/*
  Self-Driving RC Car - pure C, ESP-IDF, for the Arduino Nano ESP32 (ESP32-S3)
  LIDAR VERSION: three VL53L1X time-of-flight sensors on one shared I2C
  bus, replacing the three HY-SRF05 ultrasonic sensors.

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

  *** THIS FILE NEEDS ST'S OFFICIAL VL53L1X ULD API ***
  Get the unmodified VL53L1X_api.c / VL53L1X_api.h from ST's STSW-IMG009
  package and put them in this folder, together with your existing
  vl53l1x_platform.c/.h (the I2C glue). This file only supplies the XSHUT
  bring-up sequence and the drive logic.

  Earlier changes (unchanged in this revision):

  1. Range validation. Every reading is checked with VL53L1X_GetRangeStatus().
     A reading whose status is not 0 ("valid") is NOT used as a distance.
     The last good value is held for up to LIDAR_MAX_BAD_READS reads in a
     row, after which the sensor reports "clear" (MAX_DISTANCE_CM).
  2. Stuck detector uses the FRONT sensor only, and only while a real wall
     is in view.
  3. Per-reading ESP_LOGI calls removed from the control loop. Set
     LIDAR_DEBUG to 1 for a throttled status line instead.
  4. Inter-measurement period is 25 ms (must be >= the 20 ms timing budget).
  5. Tuning constants and turn logic follow the working AVR version. Turn
     direction is decided per corner (see the TURN section below).

  This revision:

  A. Sensor bring-up is verified (review item 5). After XSHUT is released
     the code polls VL53L1X_BootState() instead of a fixed 2 ms wait, checks
     the return status of every setup call, and confirms the sensor actually
     produces a first measurement. A failed sensor is reset and retried
     (VL53L1X_BRINGUP_ATTEMPTS). If one still fails the car refuses to drive:
     it stops and blinks the status LED N times per second, where
     N = 1 (left), 2 (middle), 3 (right) for the first sensor that failed.
     Power-cycle to try again.

  B. Side-sensor dropouts no longer look like open space (item 6). For
     steering, a side sensor keeps using its last VALID reading for up to
     LIDAR_SIDE_HOLD_READS invalid reads in a row (longer than the front
     sensor's hold). If either side has no trustworthy value the steering PID
     is skipped and the wheels go straight until both sides are valid again,
     instead of steering hard toward a phantom 200 cm gap. The TURN logic
     still sees a genuinely open side as "clear", which is what it needs to
     pick a direction.

  C. TURN steers from the first loop (item 7). The seed reading that
     triggered the corner gives a provisional direction immediately; it can
     still flip while evidence accumulates and locks as before
     (TURN_DECIDE_THRESHOLD_CM / TURN_DECIDE_MS). If there is essentially no
     evidence yet (|sum| < TURN_PROVISIONAL_MIN_CM) the wheels stay straight.

  D. Stale readings are flushed (item 8). The kickstart, recovery, pause and
     the initial start-wait all block for a long time without servicing the
     sensors. A VL53L1X holds its last result until the interrupt is cleared,
     so the first read afterwards used to return a value from BEFORE the
     delay (for example the <10 cm reading that caused a recovery, which could
     trigger another one). After any long block, lidar_flush_all() clears
     each sensor, discards one fresh measurement, and marks the readings
     "no data yet" until a new valid one arrives.

  E. Recovery reverses for REVERSE_TIME_MS in total (item 9). The timer now
     starts before the kickstart (so the kickstart counts toward the time),
     and the last wiggle step is shortened to fit instead of always running a
     full WIGGLE_HALF_PERIOD_MS.

  Behavior:
    DRIVE - centers between the two side boards with a PID on the
    left/right difference and slows down as the front wall gets close.
    TURN  - triggered when the front sensor sees a wall inside
    CORNER_TRIGGER_DISTANCE_CM. Reduced fixed speed, held for
    TURN_DURATION_MS (extended up to TURN_MAX_EXTRA_MS if still blocked).
    Recovery (reverse + wiggle) - front emergency stop (inside
    STOP_DISTANCE_CM) or stuck detection.
    Kickstart - every stopped -> moving transition gets a short full-power
    burst before dropping to the requested speed.

  No delay is needed between sensor reads: I2C sensors have no acoustic
  cross-talk, and each one free-runs its own ranging cycle.
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

// Set to 1 for a throttled "L / M / R" status line (cm, raw mm, range status)
// about every 200 ms. Leave at 0 for driving.
#define LIDAR_DEBUG 0

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
#define VL53L1X_INTER_MEASUREMENT_MS 25 // must be >= the timing budget

// How long to wait for a "data ready" flag before giving up on a single
// read. Well above the sensor's own ranging period so a normal cycle never
// trips it.
#define VL53L1X_DATA_READY_TIMEOUT_US 60000UL

// Bring-up robustness.
#define VL53L1X_BOOT_TIMEOUT_MS       100u  // max wait for the sensor firmware to boot after XSHUT goes high
#define VL53L1X_FIRST_RANGE_TIMEOUT_MS 150u // max wait for the first measurement after StartRanging
#define VL53L1X_BRINGUP_ATTEMPTS      3     // tries per sensor before giving up

// How many consecutive invalid readings from one sensor are covered by
// holding its last good value, before it is reported as "clear" instead.
// (Used for the front sensor and for TURN.)
#define LIDAR_MAX_BAD_READS 2

// Side sensors, when used for STEERING, keep using their last valid reading
// for this many invalid reads in a row (about 8 x 25-30 ms = ~200 ms). Past
// that the steering PID is skipped rather than fed a fake "wide open" side.
#define LIDAR_SIDE_HOLD_READS 8

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
#define MOTOR_LEDC_RES_BITS   LEDC_TIMER_8_BIT // 0-255 duty

// ---------- Tuning constants (taken from the working AVR version) ----------
#define MAX_DISTANCE_CM        150u    // ignore/clamp anything farther than this
#define STOP_DISTANCE_CM       10u     // genuine imminent collision - stop/reverse
#define CORNER_SLOW_DISTANCE_CM 120u    // front wall closer than this -> start slowing
#define CORNER_TRIGGER_DISTANCE_CM 80u // front wall closer than this -> commit to a TURN

#define TURN_SPEED              70u    // fixed, slow speed while executing a turn
#define TURN_DURATION_MS        400u   // how long to hold the turn through a corner - THE main knob to tune
#define TURN_MAX_EXTRA_MS       200u   // if still blocked after TURN_DURATION_MS, keep turning up to this much longer

#define SERVO_MIN_US      1000u
#define SERVO_MAX_US      2000u
#define SERVO_CENTER_DEG  90u
#define SERVO_MIN_DEG     45u
#define SERVO_MAX_DEG     135u

#define SERVO_INVERT      0u // servo direction non-inverted

#define DRIVE_SPEED       100u    // 0-255 forward PWM speed on a clear straight
#define MIN_SPEED         70u    // speed floor so the car doesn't stall approaching a corner
#define TURN_SPEED_REDUCTION 50u // max PWM cut for hard PID steering corrections on a straight
#define REVERSE_SPEED     80u    // 0-255 reverse PWM speed used during recovery

#define KICKSTART_SPEED    255u
#define KICKSTART_MS       120u
#define RECOVERY_SETTLE_MS 100u

#define REVERSE_TIME_MS        300u   // TOTAL time spent reversing, kickstart included
#define WIGGLE_HALF_PERIOD_MS  200u

// Stuck: the FRONT sensor must stay within STUCK_DISTANCE_DELTA_CM for this
// long, with a real wall in view, while in DRIVE.
#define STUCK_CHECK_WINDOW_MS   2000u
#define STUCK_DISTANCE_DELTA_CM 3u

#define BATTERY_LOW_THRESHOLD 860

// ---------- micros() ----------
static inline uint32_t micros(void) {
    return (uint32_t)esp_timer_get_time();
}

static void delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// Set whenever the code has just blocked for a long time without servicing the
// sensors (kickstart, recovery, pause, start-wait). The main loop then calls
// lidar_flush_all() before trusting any reading. Starts set: the bring-up and
// start-wait happen before the first loop iteration.
static uint8_t sensors_stale = 1;

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

// ---------- VL53L1X bring-up: wake, re-address, verify, and start each sensor ----------
// Only one sensor is released from reset (XSHUT high) at a time, so each
// one is uniquely reachable at the factory-default address (0x29) while
// it's given its own permanent address.

// Runs one setup call and bails out of the enclosing function with `false`
// (and a log line saying which call) if it returns a nonzero status.
#define BRINGUP_STEP(call, name)                                                   \
    do {                                                                           \
        status = (call);                                                           \
        if (status != 0) {                                                         \
            ESP_LOGE(TAG, "0x%02X: %s failed, status=%d", new_addr, name, status); \
            return false;                                                          \
        }                                                                          \
    } while (0)

static bool vl53l1x_bring_up_once(gpio_num_t xshut_pin, uint8_t new_addr) {
    VL53L1X_ERROR status;

    // Always start from a clean reset so a half-initialised sensor from a
    // previous failed attempt is back at the factory-default address.
    gpio_set_level(xshut_pin, 0);
    delay_ms(10);
    gpio_set_level(xshut_pin, 1);

    // Wait for the sensor firmware to finish booting (instead of a fixed delay).
    uint8_t booted = 0;
    uint32_t t0 = micros();
    while (!booted) {
        VL53L1X_BootState(VL53L1X_DEFAULT_ADDR, &booted);
        if (booted) break;
        if (micros() - t0 > (uint32_t)VL53L1X_BOOT_TIMEOUT_MS * 1000UL) {
            ESP_LOGE(TAG, "0x%02X: sensor did not boot within %u ms", new_addr, (unsigned)VL53L1X_BOOT_TIMEOUT_MS);
            return false;
        }
        delay_ms(2);
    }

    BRINGUP_STEP(vl53l1x_platform_set_address(VL53L1X_DEFAULT_ADDR, new_addr), "set_address");
    delay_ms(2);

    BRINGUP_STEP(VL53L1X_SensorInit(new_addr), "SensorInit");
    BRINGUP_STEP(VL53L1X_SetDistanceMode(new_addr, VL53L1X_DISTANCE_MODE_SHORT), "SetDistanceMode");
    BRINGUP_STEP(VL53L1X_SetTimingBudgetInMs(new_addr, VL53L1X_TIMING_BUDGET_MS), "SetTimingBudgetInMs");
    BRINGUP_STEP(VL53L1X_SetInterMeasurementInMs(new_addr, VL53L1X_INTER_MEASUREMENT_MS), "SetInterMeasurementInMs");
    BRINGUP_STEP(VL53L1X_StartRanging(new_addr), "StartRanging");

    // Prove the sensor really produces data: wait for its first measurement,
    // then clear it so continuous ranging carries on.
    uint8_t ready = 0;
    t0 = micros();
    while (!ready) {
        VL53L1X_CheckForDataReady(new_addr, &ready);
        if (ready) break;
        if (micros() - t0 > (uint32_t)VL53L1X_FIRST_RANGE_TIMEOUT_MS * 1000UL) {
            ESP_LOGE(TAG, "0x%02X: no first measurement within %u ms", new_addr, (unsigned)VL53L1X_FIRST_RANGE_TIMEOUT_MS);
            return false;
        }
        delay_ms(5);
    }
    VL53L1X_ClearInterrupt(new_addr);

    ESP_LOGI(TAG, "0x%02X: sensor ready", new_addr);
    return true;
}

static bool vl53l1x_bring_up_sensor(gpio_num_t xshut_pin, uint8_t new_addr) {
    for (int attempt = 1; attempt <= VL53L1X_BRINGUP_ATTEMPTS; attempt++) {
        if (vl53l1x_bring_up_once(xshut_pin, new_addr)) return true;
        ESP_LOGW(TAG, "0x%02X: bring-up attempt %d/%d failed", new_addr, attempt, VL53L1X_BRINGUP_ATTEMPTS);
    }
    // Leave a sensor that will not come up in reset so it can't disturb the bus.
    gpio_set_level(xshut_pin, 0);
    return false;
}

// Returns 0 if all three sensors came up, otherwise 1/2/3 for the FIRST one
// (left/middle/right) that failed. All three are always attempted so the
// log shows every failure.
static uint8_t vl53l1x_bring_up_all(void) {
    uint8_t first_failed = 0;
    if (!vl53l1x_bring_up_sensor(PIN_XSHUT_LEFT,  VL53L1X_ADDR_LEFT))  first_failed = 1;
    if (!vl53l1x_bring_up_sensor(PIN_XSHUT_MID,   VL53L1X_ADDR_MID)  && !first_failed) first_failed = 2;
    if (!vl53l1x_bring_up_sensor(PIN_XSHUT_RIGHT, VL53L1X_ADDR_RIGHT) && !first_failed) first_failed = 3;
    return first_failed;
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
        .bitwidth = ADC_BITWIDTH_12, // ESP32-S3's ADC only supports 12-bit
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
        sensors_stale = 1; // the sensors were not serviced during the kickstart
    }
    motor_pwm(speed, 0);
}

static void drive_reverse(uint8_t speed) {
    if (kickstart_needed && speed > 0) {
        motor_pwm(0, KICKSTART_SPEED);
        delay_ms(KICKSTART_MS);
        kickstart_needed = 0;
        sensors_stale = 1;
    }
    motor_pwm(0, speed);
}

static void motor_stop(void) {
    motor_pwm(0, 0);
    kickstart_needed = 1;
}

// ---------- Distance reading (VL53L1X) with range validation ----------
typedef struct {
    uint16_t addr;
    long     last_good_cm;  // last reading that passed the range-status check
    uint8_t  bad_reads;     // consecutive invalid readings so far (255 = "no data yet")
    uint16_t raw_mm;        // last raw distance (debug only)
    uint8_t  raw_status;    // last raw range status (debug only; 255 = timed out)
} Lidar;

static Lidar lidar_left  = { VL53L1X_ADDR_LEFT,  MAX_DISTANCE_CM, 0, 0, 0 };
static Lidar lidar_mid   = { VL53L1X_ADDR_MID,   MAX_DISTANCE_CM, 0, 0, 0 };
static Lidar lidar_right = { VL53L1X_ADDR_RIGHT, MAX_DISTANCE_CM, 0, 0, 0 };

// An invalid reading: hold the last good value for a couple of reads (so a
// single dropout can't fake a wall or an opening), then report "clear".
static long lidar_bad_read(Lidar *s) {
    if (s->bad_reads < 255) s->bad_reads++;
    return (s->bad_reads <= LIDAR_MAX_BAD_READS) ? s->last_good_cm : (long)MAX_DISTANCE_CM;
}

// Mark a sensor as having no usable data: it reports "clear" and is not
// usable for steering until a fresh VALID reading arrives.
static void lidar_invalidate(Lidar *s) {
    s->last_good_cm = MAX_DISTANCE_CM;
    s->bad_reads = 255;
}

// Steering-grade side reading: the last VALID distance, as long as the sensor
// has not been invalid for more than LIDAR_SIDE_HOLD_READS reads in a row.
// Returns 0 (and leaves *cm untouched) if there is no trustworthy value.
static uint8_t lidar_side_cm(const Lidar *s, long *cm) {
    if (s->bad_reads > LIDAR_SIDE_HOLD_READS) return 0;
    *cm = s->last_good_cm;
    return 1;
}

// Polls "data ready" on one sensor, checks the range status, converts mm to
// cm (everything downstream is tuned in cm), and clears the interrupt so
// the next continuous measurement can complete.
static long read_distance_cm(Lidar *s) {
    uint8_t ready = 0;
    uint32_t t0 = micros();

    do {
        VL53L1X_CheckForDataReady(s->addr, &ready);
        if (ready) break;
        if (micros() - t0 > VL53L1X_DATA_READY_TIMEOUT_US) {
            s->raw_mm = 0;
            s->raw_status = 255;
            return lidar_bad_read(s);
        }
    } while (1);

    uint8_t range_status = 255;
    uint16_t distance_mm = 0;
    VL53L1X_GetRangeStatus(s->addr, &range_status);
    VL53L1X_GetDistance(s->addr, &distance_mm);
    VL53L1X_ClearInterrupt(s->addr);

    s->raw_mm = distance_mm;
    s->raw_status = range_status;

    if (range_status != 0) { // 0 = valid measurement
        return lidar_bad_read(s);
    }

    long cm = (long)(distance_mm / 10);
    if (cm > (long)MAX_DISTANCE_CM) cm = MAX_DISTANCE_CM;
    s->last_good_cm = cm;
    s->bad_reads = 0;
    return cm;
}

// After a long block (kickstart, recovery, pause, start-wait) every sensor is
// still holding a result from BEFORE the delay. Clear it, discard the next
// measurement too (it may have been partly taken before the clear), and mark
// the readings "no data yet" so nothing stale is used.
static void lidar_flush_all(void) {
    Lidar *all[3] = { &lidar_left, &lidar_mid, &lidar_right };

    for (int i = 0; i < 3; i++) {
        VL53L1X_ClearInterrupt(all[i]->addr);
    }
    for (int i = 0; i < 3; i++) {
        uint8_t ready = 0;
        uint32_t t0 = micros();
        do {
            VL53L1X_CheckForDataReady(all[i]->addr, &ready);
            if (ready) break;
        } while (micros() - t0 < VL53L1X_DATA_READY_TIMEOUT_US);
        if (ready) VL53L1X_ClearInterrupt(all[i]->addr);
        lidar_invalidate(all[i]);
    }
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

// ---------- Steering PID (gains from the working AVR version) ----------
// Error = (right distance) - (left distance), in cm.
// Positive error => more room on the right => steer right.
#define STEER_KP  0.2f   // degrees of steering per cm of left/right imbalance
#define STEER_KI  0.01f  // corrects any steady drift/bias - keep small
#define STEER_KD  0.05f  // damps oscillation
#define STEER_INTEGRAL_LIMIT 150.0f // anti-windup clamp (cm*s)

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
// Exponential filter on the left/right difference for the straight-line
// PID. Lower alpha = smoother but slower; 1.0 = no filtering.
#define SENSOR_FILTER_ALPHA 0.8f

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

// ---------- Turn PID ----------
// Aggressive PID used only during STATE_TURN, on the RAW (unfiltered) diff.
// Only its MAGNITUDE is used for steering; the direction is decided once
// per corner (see below).
#define TURN_KP  0.5f
#define TURN_KI  0.0f
#define TURN_KD  0.0f

static SteeringPID turn_pid = {
    .kp = TURN_KP, .ki = TURN_KI, .kd = TURN_KD,
    .integral = 0.0f, .prev_error = 0.0f
};

// ---------- Per-corner turn direction (not hardcoded, no learning) ----------
// When a corner starts, (right - left) is summed each loop. Positive sum =
// more room on the right = turn right. The car steers from the very first
// loop using the current sign of that sum (provisional), which can still flip
// while evidence accumulates. The direction LOCKS as soon as |sum| reaches
// TURN_DECIDE_THRESHOLD_CM, or after TURN_DECIDE_MS at the latest, and is then
// kept for that corner only.
#define TURN_DECIDE_MS            200u  // latest the direction is locked after a corner starts
#define TURN_DECIDE_THRESHOLD_CM  70L   // lock early once |sum of (right-left)| reaches this
#define TURN_PROVISIONAL_MIN_CM   10L   // below this there is no real evidence yet: wheels stay straight
#define TURN_MIN_STEER_DEG        20u   // minimum steering away from center once steering (0 = PID magnitude only)

static long    turn_diff_accum = 0;
static uint8_t turn_decided    = 0;
static uint8_t turn_right      = 0;

static void reset_steering(void) {
    pid_reset(&steer_pid);
    pid_reset(&turn_pid);
    filtered_diff_valid = 0;
}

// ---------- Stuck detector (front sensor only) ----------
static uint32_t stuck_window_start_us = 0;
static long stuck_ref_mid = 0;
static uint8_t stuck_window_valid = 0;

static void reset_stuck_detector(void) {
    stuck_window_valid = 0;
}

static uint8_t stuck_check_and_update(uint32_t now_us, long dist_mid) {
    // Nothing in front (clamped / "clear") is open road, never "stuck".
    if (dist_mid >= (long)MAX_DISTANCE_CM) {
        stuck_window_valid = 0;
        return 0;
    }

    if (!stuck_window_valid) {
        stuck_window_start_us = now_us;
        stuck_ref_mid = dist_mid;
        stuck_window_valid = 1;
        return 0;
    }

    if ((now_us - stuck_window_start_us) < ((uint32_t)STUCK_CHECK_WINDOW_MS * 1000UL)) {
        return 0;
    }

    long change = labs(dist_mid - stuck_ref_mid);

    stuck_window_start_us = now_us;
    stuck_ref_mid = dist_mid;

    return (change < (long)STUCK_DISTANCE_DELTA_CM) ? 1 : 0;
}

// ---------- Recovery: reverse while wiggling the steering ----------
// Reverses for REVERSE_TIME_MS IN TOTAL. The clock starts before the
// kickstart (which therefore counts toward the time), the first wiggle side is
// applied before the kickstart, and the last wiggle step is shortened to fit.
static void recover_reverse_and_wiggle(void) {
    const uint32_t total_us = (uint32_t)REVERSE_TIME_MS * 1000UL;
    uint32_t recover_start_us = micros();
    uint8_t steer_right = 0; // the side AFTER the first one applied below

    set_servo_angle(SERVO_MAX_DEG); // first wiggle side, applied before the kickstart

    motor_stop();
    drive_reverse(REVERSE_SPEED); // includes the kickstart burst

    while (1) {
        uint32_t elapsed_us = micros() - recover_start_us;
        if (elapsed_us >= total_us) break;

        uint32_t remaining_ms = (total_us - elapsed_us) / 1000UL;
        delay_ms(remaining_ms < WIGGLE_HALF_PERIOD_MS ? remaining_ms : WIGGLE_HALF_PERIOD_MS);

        if ((micros() - recover_start_us) >= total_us) break;
        set_servo_angle(steer_right ? SERVO_MAX_DEG : SERVO_MIN_DEG);
        steer_right = !steer_right;
    }

    motor_stop();
    delay_ms(RECOVERY_SETTLE_MS);
    set_servo_angle(SERVO_CENTER_DEG);

    reset_steering();
    reset_stuck_detector();
    sensors_stale = 1; // nothing was read during the reverse
}

// ---------- Drive state machine ----------
typedef enum {
    STATE_DRIVE,
    STATE_TURN
} DriveState;

static uint16_t read_battery_adc(void) {
    int raw = 0;
    adc_oneshot_read(battery_adc_handle, BATTERY_ADC_CHANNEL, &raw);
    return (uint16_t)(raw >> 2); // 12-bit reading scaled to the 10-bit (0-1023) range BATTERY_LOW_THRESHOLD was calibrated against
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

// A sensor would not come up: stay stopped and blink the LED N times per
// second (1 = left, 2 = middle, 3 = right). Power-cycle to try again.
static void sensor_failure_halt(uint8_t failed_sensor) {
    motor_stop();
    set_servo_angle(SERVO_CENTER_DEG);
    ESP_LOGE(TAG, "Sensor %u failed to start - not driving. Power-cycle to retry.", (unsigned)failed_sensor);

    while (1) {
        for (uint8_t i = 0; i < failed_sensor; i++) {
            gpio_set_level(PIN_LED, 1);
            delay_ms(200);
            gpio_set_level(PIN_LED, 0);
            delay_ms(200);
        }
        delay_ms(1000);
    }
}

void app_main(void) {
    gpio_setup();
    i2c_bus_setup();
    uint8_t failed_sensor = vl53l1x_bring_up_all();
    ledc_setup();
    adc_setup();

    motor_stop();
    set_servo_angle(SERVO_CENTER_DEG);

    if (failed_sensor != 0) {
        sensor_failure_halt(failed_sensor); // never returns
    }

    while (gpio_get_level(PIN_START_TRIGGER) != 0) {
        delay_ms(20);
        battery_low_warning();
    }
    sensors_stale = 1; // the wait above may have been long

    uint32_t last_pid_us = micros();
    DriveState state = STATE_DRIVE;
    uint32_t turn_start_us = 0;
#if LIDAR_DEBUG
    uint32_t last_debug_us = 0;
#endif
    reset_stuck_detector();

    while (1) {
        if (gpio_get_level(PIN_START_TRIGGER) != 0) {
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
            sensors_stale = 1; // sensors were not read while paused
            continue;
        }

        // After any long block, drop the stale results before reading.
        if (sensors_stale) {
            lidar_flush_all();
            sensors_stale = 0;
            last_pid_us = micros();
        }

        // No inter-sensor delay needed: I2C sensors have no acoustic cross-talk.
        long dist_left  = read_distance_cm(&lidar_left);
        long dist_mid   = read_distance_cm(&lidar_mid);
        long dist_right = read_distance_cm(&lidar_right);

#if LIDAR_DEBUG
        if (micros() - last_debug_us > 200000UL) {
            last_debug_us = micros();
            ESP_LOGI(TAG, "L=%ld(%umm s%u)  M=%ld(%umm s%u)  R=%ld(%umm s%u)  state=%d",
                     dist_left,  (unsigned)lidar_left.raw_mm,  (unsigned)lidar_left.raw_status,
                     dist_mid,   (unsigned)lidar_mid.raw_mm,   (unsigned)lidar_mid.raw_status,
                     dist_right, (unsigned)lidar_right.raw_mm, (unsigned)lidar_right.raw_status,
                     (int)state);
        }
#endif

        if (dist_mid < STOP_DISTANCE_CM) {
            recover_reverse_and_wiggle();
            last_pid_us = micros();
            state = STATE_DRIVE;
            continue;
        }

        if (state == STATE_DRIVE) {
            if (stuck_check_and_update(micros(), dist_mid)) {
                recover_reverse_and_wiggle();
                last_pid_us = micros();
                continue;
            }

            if (dist_mid < CORNER_TRIGGER_DISTANCE_CM) {
                state = STATE_TURN;
                turn_start_us = micros();
                reset_steering();
                reset_stuck_detector();

                // Fresh direction decision for this corner, seeded with the
                // reading that triggered it.
                turn_diff_accum = constrain_long(dist_right - dist_left,
                                                 -(long)MAX_DISTANCE_CM, (long)MAX_DISTANCE_CM);
                turn_decided = 0;
                continue;
            }

            uint32_t now = micros();
            float dt = (float)(now - last_pid_us) / 1000000.0f;
            last_pid_us = now;

            // Steer from the sides' last VALID readings. If either side has no
            // trustworthy value (invalid for too long), do not feed the PID a
            // fake "wide open" gap: go straight and wait for valid data.
            float offset = 0.0f;
            long left_cm, right_cm;
            if (lidar_side_cm(&lidar_left, &left_cm) && lidar_side_cm(&lidar_right, &right_cm)) {
                long diff = constrain_long(right_cm - left_cm, -(long)MAX_DISTANCE_CM, (long)MAX_DISTANCE_CM);
                float smoothed_diff = filter_update((float)diff);
                offset = pid_update(&steer_pid, smoothed_diff, dt);

                int angle = (int)SERVO_CENTER_DEG + (int)offset;
                if (angle < (int)SERVO_MIN_DEG) angle = (int)SERVO_MIN_DEG;
                if (angle > (int)SERVO_MAX_DEG) angle = (int)SERVO_MAX_DEG;
                set_servo_angle((uint8_t)angle);
            } else {
                set_servo_angle(SERVO_CENTER_DEG);
                reset_steering(); // don't carry a stale integral/filter across the gap
            }

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

            uint32_t since_turn_start_ms = (now - turn_start_us) / 1000UL;

            long diff = constrain_long(dist_right - dist_left, -(long)MAX_DISTANCE_CM, (long)MAX_DISTANCE_CM);
            float offset = pid_update(&turn_pid, (float)diff, dt);

            // Direction: provisional from the first loop, locked as soon as the
            // evidence is strong or after TURN_DECIDE_MS at the latest.
            if (!turn_decided) {
                turn_diff_accum += diff;
                if (labs(turn_diff_accum) >= TURN_DECIDE_THRESHOLD_CM ||
                    since_turn_start_ms >= TURN_DECIDE_MS) {
                    turn_right = (turn_diff_accum >= 0); // more room on the right -> turn right
                    turn_decided = 1;
                } else if (labs(turn_diff_accum) >= TURN_PROVISIONAL_MIN_CM) {
                    turn_right = (turn_diff_accum >= 0); // provisional - may still flip before the lock
                }
            }

            if (turn_decided || labs(turn_diff_accum) >= TURN_PROVISIONAL_MIN_CM) {
                // Only the PID's magnitude is used; the sign comes from the direction above.
                float magnitude = (offset < 0.0f) ? -offset : offset;
                if (magnitude < (float)TURN_MIN_STEER_DEG) magnitude = (float)TURN_MIN_STEER_DEG;
                offset = turn_right ? magnitude : -magnitude;

                int angle = (int)SERVO_CENTER_DEG + (int)offset;
                if (angle < (int)SERVO_MIN_DEG) angle = (int)SERVO_MIN_DEG;
                if (angle > (int)SERVO_MAX_DEG) angle = (int)SERVO_MAX_DEG;
                set_servo_angle((uint8_t)angle);
            } else {
                // No real evidence yet: wheels straight rather than guess.
                set_servo_angle(SERVO_CENTER_DEG);
            }

            drive_forward(TURN_SPEED);

            uint32_t elapsed_ms = (micros() - turn_start_us) / 1000UL;

            if (elapsed_ms >= TURN_DURATION_MS) {
                // Nominal turn time is up. If the front is clear again the
                // corner's behind us; if it's still blocked keep turning a
                // bit longer rather than driving straight into the wall.
                if (dist_mid >= CORNER_TRIGGER_DISTANCE_CM ||
                    elapsed_ms >= (TURN_DURATION_MS + TURN_MAX_EXTRA_MS)) {
                    state = STATE_DRIVE;
                    reset_steering();
                    reset_stuck_detector();
                    last_pid_us = micros();
                }
            }
        }
    }
}