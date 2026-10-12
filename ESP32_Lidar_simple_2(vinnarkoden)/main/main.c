/*
  Self-Driving RC Car - SIMPLE version
  ESP-IDF, Arduino Nano ESP32 (ESP32-S3), three VL53L1X lidars on one I2C bus.

  Goal: get around a small walled track (about 4 m, 90-degree corners,
  ~15 cm walls) reliably. No PID, no state machine - one rule every loop:

    1. Steer away from the closer side wall   (gentle, capped).
    2. The closer the front wall, the harder we turn toward the open side
       (from 0 at CORNER_CM to full lock at CORNER_FULL_CM).
    3. Slow down as the front wall gets close and while steering hard;
       speed up only gradually (no burst when the front opens mid-corner).
    4. Front wall too close  -> reverse with the wheels turned the other way,
       then carry on (rule 2 then turns the car away from the wall).
    5. Not moving for a while -> one full-power push forward; if that does not
       help, the same reverse as in 4.

  Wiring (Arduino label -> GPIO):
    A4 -> 11  I2C SDA            D9  -> 18  Steering servo
    A5 -> 12  I2C SCL            D3  -> 6   DRV8871 IN1
    D2 -> 5   Left  lidar XSHUT  D11 -> 38  DRV8871 IN2
    D4 -> 7   Front lidar XSHUT  A0  -> 1   Start trigger (HIGH = run)
    D8 -> 17  Right lidar XSHUT  D6  -> 9   Status LED

  Needs ST's VL53L1X ULD API (VL53L1X_api.c/.h from STSW-IMG009) and your
  vl53l1x_platform.c/.h I2C glue in the same folder.

  LED: on = driving, off = waiting for the start trigger,
       blinking 1/2/3 times = left/front/right sensor failed to start.
*/

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "vl53l1x_platform.h"
#include "VL53L1X_api.h"

#define TAG "rc_car"

// 1 = print "L M R steer speed" about 5 times a second. Use it on the bench.
#define LIDAR_DEBUG 0

// =====================================================================
//  SETTINGS - everything you should need to tune is here
// =====================================================================

// Which way the corners go, used when the sensors don't show an opening
// yet. A loop driven in one direction usually turns the same way at every
// corner: set +1 (right) or -1 (left). 0 = remember the last opening seen.
#define TRACK_TURN_DIR    0

// Distances (cm). Short distance mode reaches about 130 cm.
#define MAX_CM            130L  // "nothing in range" is reported as this
#define STOP_CM           10L   // front closer than this -> reverse
#define SLOW_CM           120L  // front closer than this -> start slowing down
#define CORNER_CM         90L   // front closer than this -> start turning
#define CORNER_FULL_CM    35L   // front closer than this -> full steering lock
#define OPENING_CM        40L   // |right - left| >= this = one side is open (a corner)
#define ESCAPE_DIFF_CM    15L   // |right - left| >= this picks the reverse/escape side

// Steering (degrees away from center; the servo is limited to +-45).
#define KP_DEG_PER_CM     0.f  // straight: degrees per cm of left/right difference
#define STRAIGHT_MAX_DEG  15    // straight: never steer harder than this
#define FULL_STEER_DEG    45    // corners and reversing
#define SERVO_INVERT      0     // 0: +deg = right. Bench test: hand near LEFT sensor -> wheels turn RIGHT.

// Motor PWM (0-255).
#define DRIVE_SPEED       80    // straight
#define CORNER_SPEED      70    // at the front wall
#define REVERSE_SPEED     90
#define KICK_SPEED        255   // full-power burst whenever the car starts from standstill
#define KICK_MS           200
#define SPEED_RAMP_UP     2     // max PWM increase per loop (~30 ms): 2 = 60 -> 80 in about 0.3 s

// Recovery.
#define REVERSE_MS        400   // total reverse time (kick included)
#define SETTLE_MS         150   // pause after reversing (servo swings to the escape side)
#define STOP_CONFIRM      2     // front must be < STOP_CM this many reads in a row

// Stuck = front reading changes less than STUCK_DELTA_CM for STUCK_MS
// (STUCK_CLEAR_MS when the front sees nothing, e.g. nose up on a bump).
#define STUCK_DELTA_CM    5L
#define STUCK_MS          1500u
#define STUCK_CLEAR_MS    3000u

// Sensor handling.
#define HOLD_READS        4     // a few bad readings keep the last good value
#define NEAR_CM           15L   // lost the target while it was this close -> it is too close to see (0 cm)
#define SENSOR_ROI        6     // field of view, 4..16 SPADs (16 = full 27 deg cone). Smaller sees less floor.

// Start trigger: pause only after this many LOW reads in a row (~150 ms).
#define START_OFF_READS   5

// =====================================================================
//  Hardware
// =====================================================================

#define PIN_I2C_SDA       11
#define PIN_I2C_SCL       12
#define PIN_XSHUT_LEFT    5
#define PIN_XSHUT_MID     7
#define PIN_XSHUT_RIGHT   17
#define PIN_SERVO         18
#define PIN_MOTOR_IN1     6
#define PIN_MOTOR_IN2     38
#define PIN_START_TRIGGER 1
#define PIN_LED           9

#define ADDR_DEFAULT      0x29
#define ADDR_LEFT         0x30
#define ADDR_MID          0x31
#define ADDR_RIGHT        0x32

#define LEDC_MODE         LEDC_LOW_SPEED_MODE
#define SERVO_CH          LEDC_CHANNEL_0
#define MOTOR_CH1         LEDC_CHANNEL_1
#define MOTOR_CH2         LEDC_CHANNEL_2
#define SERVO_RES_BITS    14
#define SERVO_PERIOD_US   20000u

#define READ_TIMEOUT_US   60000u

static inline uint32_t micros(void) { return (uint32_t)esp_timer_get_time(); }
static void delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

static long clampl(long x, long lo, long hi) { return x < lo ? lo : (x > hi ? hi : x); }

// Linear map, like Arduino's map().
static long mapl(long x, long in_a, long in_b, long out_a, long out_b) {
    return (x - in_a) * (out_b - out_a) / (in_b - in_a) + out_a;
}

static bool sensors_stale = true; // set after any long blocking delay

// ---------- Setup ----------
static void gpio_setup(void) {
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_XSHUT_LEFT) | (1ULL << PIN_XSHUT_MID) |
                        (1ULL << PIN_XSHUT_RIGHT) | (1ULL << PIN_LED),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&out);

    gpio_config_t in = {
        .pin_bit_mask = 1ULL << PIN_START_TRIGGER,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, // loose wire reads LOW = stop 
    };
    gpio_config(&in);

    gpio_set_level(PIN_XSHUT_LEFT, 0);
    gpio_set_level(PIN_XSHUT_MID, 0);
    gpio_set_level(PIN_XSHUT_RIGHT, 0);
    gpio_set_level(PIN_LED, 0);
}

static void i2c_setup(void) {
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_new_master_bus(&cfg, &bus);
    if (err != ESP_OK) ESP_LOGE(TAG, "I2C bus setup failed: %s", esp_err_to_name(err));
    vl53l1x_platform_init_bus(bus);
}

static void pwm_setup(void) {
    ledc_timer_config_t servo_timer = {
        .speed_mode = LEDC_MODE, .timer_num = LEDC_TIMER_0,
        .duty_resolution = SERVO_RES_BITS, .freq_hz = 50, .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&servo_timer);

    ledc_timer_config_t motor_timer = {
        .speed_mode = LEDC_MODE, .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_8_BIT, .freq_hz = 1000, .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&motor_timer);

    const struct { ledc_channel_t ch; ledc_timer_t timer; int pin; } chans[] = {
        { SERVO_CH,  LEDC_TIMER_0, PIN_SERVO },
        { MOTOR_CH1, LEDC_TIMER_1, PIN_MOTOR_IN1 },
        { MOTOR_CH2, LEDC_TIMER_1, PIN_MOTOR_IN2 },
    };
    for (int i = 0; i < 3; i++) {
        ledc_channel_config_t c = {
            .speed_mode = LEDC_MODE, .channel = chans[i].ch, .timer_sel = chans[i].timer,
            .intr_type = LEDC_INTR_DISABLE, .gpio_num = chans[i].pin, .duty = 0, .hpoint = 0,
        };
        ledc_channel_config(&c);
    }
}

// ---------- Steering ----------
// deg: 0 = straight, positive = right, negative = left (limited to +-45).
static int steer_now = 0;

static void steer(int deg) {
    if (deg > 45) deg = 45;
    if (deg < -45) deg = -45;
    steer_now = deg;
#if SERVO_INVERT
    deg = -deg;
#endif
    uint32_t pulse_us = 1500 + (deg * 500) / 45; // 1000..2000 us
    uint32_t duty = (uint32_t)((uint64_t)pulse_us * ((1u << SERVO_RES_BITS) - 1) / SERVO_PERIOD_US);
    ledc_set_duty(LEDC_MODE, SERVO_CH, duty);
    ledc_update_duty(LEDC_MODE, SERVO_CH);
}

// ---------- Motor ----------
static bool motor_stopped = true; // next start gets a full-power kick

static void motor_pwm(uint8_t in1, uint8_t in2) {
    ledc_set_duty(LEDC_MODE, MOTOR_CH1, in1);
    ledc_update_duty(LEDC_MODE, MOTOR_CH1);
    ledc_set_duty(LEDC_MODE, MOTOR_CH2, in2);
    ledc_update_duty(LEDC_MODE, MOTOR_CH2);
}

static void motor_stop(void) {
    motor_pwm(0, 0);
    motor_stopped = true;
}

static void motor_forward(uint8_t speed) {
    if (motor_stopped) {
        motor_pwm(KICK_SPEED, 0);
        delay_ms(KICK_MS);
        motor_stopped = false;
        sensors_stale = true;
    }
    motor_pwm(speed, 0);
}

// ---------- Lidar ----------
typedef struct {
    uint16_t addr;
    long     cm;      // last GOOD reading
    uint8_t  bad;     // bad readings in a row
    uint8_t  status;  // last range status (255 = timeout / I2C error)
} Lidar;

static Lidar lidar_left  = { ADDR_LEFT,  MAX_CM, 0, 0 };
static Lidar lidar_mid   = { ADDR_MID,   MAX_CM, 0, 0 };
static Lidar lidar_right = { ADDR_RIGHT, MAX_CM, 0, 0 };

// One reading in cm. Bad readings: hold the last good value for a few reads;
// after that the target is either too close to see (last seen within
// NEAR_CM -> 0) or out of range (-> MAX_CM).
static long lidar_read(Lidar *s) {
    uint8_t ready = 0, range_status = 255;
    uint16_t mm = 0;
    uint32_t t0 = micros();

    while (!ready && (micros() - t0) < READ_TIMEOUT_US) {
        if (VL53L1X_CheckForDataReady(s->addr, &ready) != 0) ready = 0;
    }

    bool ok = false;
    if (ready) {
        ok = VL53L1X_GetRangeStatus(s->addr, &range_status) == 0 &&
             VL53L1X_GetDistance(s->addr, &mm) == 0 &&
             range_status == 0;
        VL53L1X_ClearInterrupt(s->addr);
    }
    s->status = ready ? range_status : 255;

    if (ok) {
        s->cm = clampl(mm / 10, 0, MAX_CM);
        s->bad = 0;
        return s->cm;
    }
    if (s->bad < 255) s->bad++;
    if (s->bad <= HOLD_READS) return s->cm;
    return (s->cm <= NEAR_CM) ? 0 : MAX_CM;
}

// Throw away results measured during a long delay (kick, reverse, pause).
static void lidar_flush(void) {
    Lidar *all[3] = { &lidar_left, &lidar_mid, &lidar_right };
    for (int i = 0; i < 3; i++) VL53L1X_ClearInterrupt(all[i]->addr);
    for (int i = 0; i < 3; i++) {
        uint8_t ready = 0;
        uint32_t t0 = micros();
        while (!ready && (micros() - t0) < READ_TIMEOUT_US) {
            if (VL53L1X_CheckForDataReady(all[i]->addr, &ready) != 0) ready = 0;
        }
        if (ready) VL53L1X_ClearInterrupt(all[i]->addr);
    }
}

// Wake one sensor (others stay in reset), give it its own address, configure
// and start it, and wait for a first measurement. Returns false on any error.
static bool lidar_start_once(gpio_num_t xshut, uint8_t addr) {
    gpio_set_level(xshut, 0);
    delay_ms(10);
    gpio_set_level(xshut, 1);

    uint8_t booted = 0;
    uint32_t t0 = micros();
    while (!booted) {
        VL53L1X_BootState(ADDR_DEFAULT, &booted);
        if (booted) break;
        if (micros() - t0 > 100000u) return false;
        delay_ms(2);
    }

    if (vl53l1x_platform_set_address(ADDR_DEFAULT, addr) != 0) return false;
    delay_ms(2);
    if (VL53L1X_SensorInit(addr) != 0) return false;
    if (VL53L1X_SetDistanceMode(addr, 1) != 0) return false;          // 1 = short (~130 cm)
    if (VL53L1X_SetTimingBudgetInMs(addr, 20) != 0) return false;
    if (VL53L1X_SetInterMeasurementInMs(addr, 25) != 0) return false;
    if (VL53L1X_SetROI(addr, SENSOR_ROI, SENSOR_ROI) != 0) return false;
    if (VL53L1X_StartRanging(addr) != 0) return false;

    uint8_t ready = 0;
    t0 = micros();
    while (!ready) {
        VL53L1X_CheckForDataReady(addr, &ready);
        if (ready) break;
        if (micros() - t0 > 150000u) return false;
        delay_ms(5);
    }
    VL53L1X_ClearInterrupt(addr);
    return true;
}

static bool lidar_start(gpio_num_t xshut, uint8_t addr) {
    for (int attempt = 1; attempt <= 3; attempt++) {
        if (lidar_start_once(xshut, addr)) {
            ESP_LOGI(TAG, "lidar 0x%02X ready", (unsigned)addr);
            return true;
        }
        ESP_LOGW(TAG, "lidar 0x%02X: start attempt %d failed", (unsigned)addr, attempt);
    }
    gpio_set_level(xshut, 0); // keep a dead sensor off the bus
    return false;
}

// Sensor failed: stay stopped, blink the LED n times every couple of seconds.
static void halt_blinking(int n) {
    motor_stop();
    steer(0);
    ESP_LOGE(TAG, "sensor %d failed - not driving, power-cycle to retry", n);
    while (1) {
        for (int i = 0; i < n; i++) {
            gpio_set_level(PIN_LED, 1); delay_ms(200);
            gpio_set_level(PIN_LED, 0); delay_ms(200);
        }
        delay_ms(1000);
    }
}

// ---------- Start trigger ----------
static bool start_trigger_off(void) {
    static int low_reads = 0;
    if (gpio_get_level(PIN_START_TRIGGER)) { low_reads = 0; return false; }
    if (low_reads < START_OFF_READS) low_reads++;
    return low_reads >= START_OFF_READS;
}

static void wait_for_start(void) {
    motor_stop();
    steer(0);
    gpio_set_level(PIN_LED, 0);
    while (gpio_get_level(PIN_START_TRIGGER) == 0) delay_ms(20);
    gpio_set_level(PIN_LED, 1);
    sensors_stale = true;
}

// ---------- Corner direction ----------
// +1 = right, -1 = left. Updated whenever one side is clearly open.
static int turn_dir = (TRACK_TURN_DIR != 0) ? TRACK_TURN_DIR : 1;

// Side to turn toward: the clearly more open side if there is one,
// otherwise the corner direction we expect.
static int pick_dir(long diff, long min_diff) {
    if (diff >= min_diff)  return 1;
    if (diff <= -min_diff) return -1;
    return (TRACK_TURN_DIR != 0) ? TRACK_TURN_DIR : turn_dir;
}

// ---------- Recovery ----------
// Reverse with the wheels turned AWAY from the escape side (that swings the
// nose toward it), then point the wheels to the escape side.
static void reverse_and_escape(int dir) {
    ESP_LOGI(TAG, "reverse, escape %s", dir > 0 ? "right" : "left");
    motor_stop();
    steer(-dir * FULL_STEER_DEG);

    motor_pwm(0, KICK_SPEED);
    delay_ms(KICK_MS < REVERSE_MS ? KICK_MS : REVERSE_MS);
    if (REVERSE_MS > KICK_MS) {
        motor_pwm(0, REVERSE_SPEED);
        delay_ms(REVERSE_MS - KICK_MS);
    }

    motor_stop();
    steer(dir * FULL_STEER_DEG);
    delay_ms(SETTLE_MS);
    sensors_stale = true;
}

// ---------- Stuck detector ----------
static long     stuck_ref = -1;
static uint32_t stuck_since_us = 0;
static int      stuck_tries = 0; // 0 = next time push forward, 1 = next time reverse

static void stuck_reset(void) { stuck_ref = -1; }

static bool stuck_check(long front, uint32_t now) {
    if (stuck_ref < 0 || labs(front - stuck_ref) >= STUCK_DELTA_CM) {
        if (stuck_ref >= 0) stuck_tries = 0; // it moved
        stuck_ref = front;
        stuck_since_us = now;
        return false;
    }
    uint32_t limit_ms = (front >= MAX_CM) ? STUCK_CLEAR_MS : STUCK_MS;
    return (now - stuck_since_us) >= limit_ms * 1000u;
}

// =====================================================================
//  Main
// =====================================================================
void app_main(void) {
    gpio_setup();
    pwm_setup();
    motor_stop();
    steer(0);
    i2c_setup();

    if (!lidar_start(PIN_XSHUT_LEFT,  ADDR_LEFT))  halt_blinking(1);
    if (!lidar_start(PIN_XSHUT_MID,   ADDR_MID))   halt_blinking(2);
    if (!lidar_start(PIN_XSHUT_RIGHT, ADDR_RIGHT)) halt_blinking(3);

    wait_for_start();

    int stop_hits = 0;
    long last_speed = CORNER_SPEED;
#if LIDAR_DEBUG
    uint32_t last_print_us = 0;
#endif

    while (1) {
        if (start_trigger_off()) {
            ESP_LOGW(TAG, "start trigger LOW - paused");
            wait_for_start();
            stop_hits = 0;
            stuck_reset();
            continue;
        }

        if (sensors_stale) {
            lidar_flush();
            sensors_stale = false;
        }

        long left  = lidar_read(&lidar_left);
        long front = lidar_read(&lidar_mid);
        long right = lidar_read(&lidar_right);

        // Positive diff = more room on the right.
        long diff = clampl(right - left, -100, 100);
        if (diff >= OPENING_CM)  turn_dir = 1;
        if (diff <= -OPENING_CM) turn_dir = -1;

        // 1. Too close to the front wall: reverse and turn away.
        if (front < STOP_CM) {
            if (++stop_hits >= STOP_CONFIRM) {
                reverse_and_escape(pick_dir(diff, ESCAPE_DIFF_CM));
                stop_hits = 0;
                stuck_reset();
                continue;
            }
        } else {
            stop_hits = 0;
        }

        // 2. Not moving: push forward once, then reverse.
        if (stuck_check(front, micros())) {
            stuck_reset();
            if (stuck_tries == 0) {
                ESP_LOGW(TAG, "stuck - push forward");
                stuck_tries = 1;
                motor_stop(); // the next motor_forward() gives the full-power kick
            } else {
                ESP_LOGW(TAG, "still stuck - reverse");
                stuck_tries = 0;
                reverse_and_escape(pick_dir(diff, ESCAPE_DIFF_CM));
                continue;
            }
        }

        // 3. Steering. Straight: gently away from the closer wall.
        int deg = (int)(KP_DEG_PER_CM * (float)diff);
        if (deg >  STRAIGHT_MAX_DEG) deg =  STRAIGHT_MAX_DEG;
        if (deg < -STRAIGHT_MAX_DEG) deg = -STRAIGHT_MAX_DEG;

        //    Front wall ahead: turn toward the open side, harder as it gets closer.
        if (front < CORNER_CM) {
            int dir = pick_dir(diff, OPENING_CM);
            int corner_deg = (int)mapl(clampl(front, CORNER_FULL_CM, CORNER_CM),
                                       CORNER_CM, CORNER_FULL_CM, 0, FULL_STEER_DEG);
            int same_way = deg * dir; // straight steering that already goes that way
            deg = dir * (same_way > corner_deg ? same_way : corner_deg);
        }
        steer(deg);

        // 4. Speed: slow down toward the front wall AND while steering hard,
        //    and only speed up gradually - no burst when the front opens mid-corner.
        long speed = DRIVE_SPEED;
        if (front < SLOW_CM) {
            speed = mapl(clampl(front, STOP_CM, SLOW_CM), STOP_CM, SLOW_CM, CORNER_SPEED, DRIVE_SPEED);
        }
        long steer_speed = mapl(clampl(labs(steer_now), STRAIGHT_MAX_DEG, FULL_STEER_DEG),
                                STRAIGHT_MAX_DEG, FULL_STEER_DEG, DRIVE_SPEED, CORNER_SPEED);
        if (steer_speed < speed) speed = steer_speed;           // hard steering -> corner speed
        if (motor_stopped) last_speed = CORNER_SPEED;           // after a reverse/pause: ramp from corner speed
        if (speed > last_speed + SPEED_RAMP_UP) speed = last_speed + SPEED_RAMP_UP;
        last_speed = speed;
        motor_forward((uint8_t)clampl(speed, 0, 255));

#if LIDAR_DEBUG
        if (micros() - last_print_us > 200000u) {
            last_print_us = micros();
            ESP_LOGI(TAG, "L=%3ld(s%u) F=%3ld(s%u) R=%3ld(s%u)  steer=%+d speed=%ld",
                     left, (unsigned)lidar_left.status, front, (unsigned)lidar_mid.status,
                     right, (unsigned)lidar_right.status, steer_now, speed);
        }
#endif
    }
}
