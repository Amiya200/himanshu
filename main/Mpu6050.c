/*
 * mpu6050.c  —  MPU-6050 / ICM-20600 I2C driver
 *
 * FIX vs previous: WHO_AM_I=0x70 is ICM-20600, which is register-
 * compatible with MPU-6050.  Previously logged as "unexpected" and
 * continued, but s_mpu_ready was still set — so the sensor WAS
 * working all along. Now 0x70 is explicitly accepted with a clear
 * info message so the log no longer causes confusion.
 *
 * Accepted WHO_AM_I values:
 *   0x68 — MPU-6050
 *   0x70 — ICM-20600  (same register map, pin-compatible)
 *   0x72 — MPU-6500   (pin/register-compatible)
 *   0x19 — MPU-6050 variant (some clones)
 *   anything else — warn but continue if I2C responded OK
 *
 * Also added: mpu_print_status() called from the task every
 * MPU_STATUS_PRINT_INTERVAL samples so values appear on serial
 * without needing to type STATUS.
 */

#include "mpu6050.h"
#include "config.h"
#include "motor.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "MPU6050";

/* Print a live status line every N poll cycles (~5 s at 50 Hz) */
#define MPU_STATUS_PRINT_INTERVAL   250

/* ─── MPU-6050 register map ───────────────────────────────── */
#define REG_PWR_MGMT_1      0x6B
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_TEMP_OUT_H      0x41
#define REG_GYRO_XOUT_H     0x43
#define REG_WHO_AM_I        0x75

/* ─── state ───────────────────────────────────────────────── */
static mpu_data_t        s_data;
static SemaphoreHandle_t s_mutex          = NULL;
static volatile float    s_heading        = 0.0f;
static volatile bool     s_accident_fired = false;
static float             s_gyro_z_bias    = 0.0f;
static volatile bool     s_mpu_ready      = false;
static uint8_t           s_who_am_i       = 0x00;  /* store for logging */

/* ─── I2C helpers ─────────────────────────────────────────── */

static esp_err_t i2c_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(
               MPU_I2C_PORT, MPU6050_ADDR, buf, 2,
               pdMS_TO_TICKS(20));
}

static esp_err_t i2c_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
               MPU_I2C_PORT, MPU6050_ADDR,
               &reg, 1, data, len,
               pdMS_TO_TICKS(20));
}

static inline int16_t to_int16(uint8_t hi, uint8_t lo)
{
    return (int16_t)((uint16_t)(hi << 8) | lo);
}

/* ─── I2C bus scan (debug only) ───────────────────────────── */

#if CONFIG_LOG_DEFAULT_LEVEL >= 3
static void i2c_scan(void)
{
    ESP_LOGI(TAG, "I2C scan (SDA=%d SCL=%d port=%d)",
             (int)PIN_MPU_SDA, (int)PIN_MPU_SCL, (int)MPU_I2C_PORT);
    bool found = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(MPU_I2C_PORT, cmd,
                                              pdMS_TO_TICKS(30));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  Found I2C device at 0x%02X", (unsigned)addr);
            found = true;
        }
    }
    if (!found) ESP_LOGW(TAG, "  No I2C devices found -- check wiring!");
}
#else
static void i2c_scan(void) {}
#endif

/* ─── WHO_AM_I identification ─────────────────────────────── */

static const char *identify_who_am_i(uint8_t who)
{
    switch (who) {
        case 0x68: return "MPU-6050";
        case 0x70: return "ICM-20600";   /* FIX: was logged as unexpected */
        case 0x72: return "MPU-6500";
        case 0x19: return "MPU-6050 clone";
        case 0x71: return "MPU-9250";
        default:   return "Unknown (compatible)";
    }
}

/* Returns true if the WHO_AM_I value means the bus is broken */
static bool who_am_i_is_bus_fault(uint8_t who)
{
    return (who == 0x00 || who == 0xFF);
}

/* ─── gyro Z bias calibration ─────────────────────────────── */

static void calibrate_gyro_z(int samples)
{
    float   sum  = 0.0f;
    int     good = 0;
    uint8_t buf[2];

    ESP_LOGI(TAG, "Calibrating gyro Z bias (%d samples, keep still)...",
             samples);

    for (int i = 0; i < samples; i++) {
        if (i2c_read_reg(REG_GYRO_XOUT_H + 4, buf, 2) == ESP_OK) {
            sum += (float)to_int16(buf[0], buf[1]) / GYRO_SENSITIVITY;
            good++;
        }
        vTaskDelay(pdMS_TO_TICKS(MPU_POLL_INTERVAL_MS));
    }
    s_gyro_z_bias = (good > 0) ? (sum / (float)good) : 0.0f;
    ESP_LOGI(TAG, "Gyro Z bias=%.4f deg/s  (%d/%d samples OK)",
             (double)s_gyro_z_bias, good, samples);
}

/* ─── MPU task ────────────────────────────────────────────── */

#if MPU_ENABLED

static void mpu_task(void *arg)
{
    (void)arg;
    const float dt  = MPU_POLL_INTERVAL_MS / 1000.0f;
    int vib_cnt     = 0;
    int print_cnt   = 0;

    vTaskDelay(pdMS_TO_TICKS(500));
    calibrate_gyro_z(100);
    ESP_LOGI(TAG, "MPU task running  chip=%s(0x%02X)  dt=%.3f s",
             identify_who_am_i(s_who_am_i), (unsigned)s_who_am_i,
             (double)dt);

    /* Print initial live values immediately so user can see sensor works */
    ESP_LOGI(TAG, "--- LIVE MPU DATA (prints every %d ms) ---",
             MPU_STATUS_PRINT_INTERVAL * MPU_POLL_INTERVAL_MS);

    while (1) {
        uint8_t raw[14];
        if (i2c_read_reg(REG_ACCEL_XOUT_H, raw, 14) != ESP_OK) {
            ESP_LOGW(TAG, "I2C read fail -- skipping sample");
            vTaskDelay(pdMS_TO_TICKS(MPU_POLL_INTERVAL_MS));
            continue;
        }

        /* Accelerometer (g) */
        float ax = (float)to_int16(raw[0],  raw[1])  / ACCEL_SENSITIVITY;
        float ay = (float)to_int16(raw[2],  raw[3])  / ACCEL_SENSITIVITY;
        float az = (float)to_int16(raw[4],  raw[5])  / ACCEL_SENSITIVITY;

        /* Temperature (C) */
        int16_t raw_temp = to_int16(raw[6], raw[7]);
        int     temp_c   = (int)(raw_temp / 340.0f + 36.53f);

        /* Gyroscope (deg/s) */
        float gx = (float)to_int16(raw[8],  raw[9])  / GYRO_SENSITIVITY;
        float gy = (float)to_int16(raw[10], raw[11]) / GYRO_SENSITIVITY;
        float gz = (float)to_int16(raw[12], raw[13]) / GYRO_SENSITIVITY
                   - s_gyro_z_bias;

        /* Integrate yaw heading */
        float new_heading = fmodf(s_heading + gz * dt + 360.0f, 360.0f);
        s_heading = new_heading;

        /* Vibration (deviation from 1 g) */
        float mag = sqrtf(ax*ax + ay*ay + az*az);
        float vib = fabsf(mag - 1.0f);

        /* Debounced accident detection */
        if (vib > MPU_VIB_THRESHOLD_G) {
            vib_cnt++;
        } else {
            vib_cnt = 0;
        }

        bool accident_now = (vib_cnt >= MPU_VIB_DEBOUNCE);
        if (accident_now && !s_accident_fired && !motor_get_accident()) {
            s_accident_fired = true;
            ESP_LOGW(TAG, "VIBRATION ACCIDENT  mag=%.3f vib=%.3f",
                     (double)mag, (double)vib);
            motor_set_accident(true);
        }

        /* Publish under mutex */
        mpu_data_t d = {
            .accel_x     = ax,   .accel_y = ay,   .accel_z = az,
            .gyro_x      = gx,   .gyro_y  = gy,   .gyro_z  = gz,
            .heading_deg = s_heading,
            .vib_mag     = vib,
            .accident    = accident_now,
            .temp_c      = temp_c,
        };
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            s_data = d;
            xSemaphoreGive(s_mutex);
        }

        /*
         * Periodic serial status print so you can see sensor values
         * without typing STATUS.  Prints every ~5 s.
         */
        print_cnt++;
        if (print_cnt >= MPU_STATUS_PRINT_INTERVAL) {
            print_cnt = 0;
            ESP_LOGI(TAG,
                     "[MPU] Ax=%.3f Ay=%.3f Az=%.3f g | "
                     "Gz=%.2f deg/s | Hdg=%.1f deg | "
                     "Vib=%.3f g | Temp=%d C | Acc=%d",
                     (double)ax, (double)ay, (double)az,
                     (double)gz, (double)s_heading,
                     (double)vib, temp_c,
                     (int)motor_get_accident());
        }

        vTaskDelay(pdMS_TO_TICKS(MPU_POLL_INTERVAL_MS));
    }
}

#endif /* MPU_ENABLED */

/* ─── public API ──────────────────────────────────────────── */

void mpu_init(void)
{
#if MPU_ENABLED
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex);

    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_MPU_SDA,
        .scl_io_num       = PIN_MPU_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MPU_I2C_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(MPU_I2C_PORT, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return;
    }

    err = i2c_driver_install(MPU_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    i2c_scan();

    /* Wake sensor */
    err = i2c_write_reg(REG_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "Sensor not responding  SDA=%d SCL=%d ADDR=0x%02X  %s",
                 (int)PIN_MPU_SDA, (int)PIN_MPU_SCL,
                 (unsigned)MPU6050_ADDR, esp_err_to_name(err));
        ESP_LOGE(TAG, "Check: wiring, 4.7k pull-ups, AD0 pin, 3.3V supply");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Configure */
    i2c_write_reg(REG_SMPLRT_DIV,  0x04);  /* 200 Hz */
    i2c_write_reg(REG_CONFIG,       0x03);  /* DLPF ~44 Hz */
    i2c_write_reg(REG_GYRO_CONFIG,  0x00);  /* +/-250 dps */
    i2c_write_reg(REG_ACCEL_CONFIG, 0x00);  /* +/-2 g */

    /* WHO_AM_I check */
    i2c_read_reg(REG_WHO_AM_I, &s_who_am_i, 1);

    if (who_am_i_is_bus_fault(s_who_am_i)) {
        ESP_LOGE(TAG,
                 "WHO_AM_I=0x%02X -- bus fault (all zeros/ones). "
                 "Check SDA/SCL wiring and pull-up resistors.",
                 (unsigned)s_who_am_i);
        return;
    }

    /*
     * FIX: 0x70 = ICM-20600, fully register-compatible with MPU-6050.
     * Previously logged as "unexpected" which was misleading.
     * All known-good values are accepted without warning.
     */
    const char *chip_name = identify_who_am_i(s_who_am_i);
    if (s_who_am_i == 0x68 || s_who_am_i == 0x70 ||
        s_who_am_i == 0x72 || s_who_am_i == 0x19 || s_who_am_i == 0x71) {
        ESP_LOGI(TAG, "Sensor identified: %s  (WHO_AM_I=0x%02X)  SDA=%d SCL=%d",
                 chip_name, (unsigned)s_who_am_i,
                 (int)PIN_MPU_SDA, (int)PIN_MPU_SCL);
    } else {
        ESP_LOGW(TAG,
                 "WHO_AM_I=0x%02X (%s) -- unrecognised but I2C responded OK. "
                 "Sensor likely compatible, continuing.",
                 (unsigned)s_who_am_i, chip_name);
    }

    s_mpu_ready = true;
    xTaskCreate(mpu_task, "mpu_task", 3072, NULL, 7, NULL);

#else
    ESP_LOGW(TAG, "MPU disabled (MPU_ENABLED=0 in config.h)");
#endif
}

mpu_data_t mpu_get(void)
{
    mpu_data_t d = {0};
#if MPU_ENABLED
    if (s_mutex &&
        xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        d = s_data;
        xSemaphoreGive(s_mutex);
    }
#endif
    return d;
}

void mpu_reset_heading(void)
{
    s_heading = 0.0f;
    ESP_LOGI(TAG, "Heading reset to 0");
}

void mpu_clear_accident(void)
{
    s_accident_fired = false;
}

float mpu_heading(void)
{
    return s_heading;
}

bool mpu_is_ready(void)
{
    return s_mpu_ready;
}