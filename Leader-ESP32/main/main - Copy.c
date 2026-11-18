#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <math.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"

#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"


/* ============================================================
   USER SETTINGS
   ============================================================ */

/*
 * Sensor calculation rate
 */
#define SAMPLE_PERIOD_MS           10          // 100 Hz

/*
 * Terminal print rate
 *
 * 100 Hz / 10 = 10 Hz
 */
#define TERMINAL_PRINT_DIV         10

/*
 * Bluetooth send rate
 *
 * 100 Hz / 2 = 50 Hz
 */
#define BT_SEND_DIV                2


/* ============================================================
   KNEE CONTROL SETTINGS
   ============================================================ */

/*
 * Maximum angle that may be sent to the robotic leg.
 *
 * CHANGE THIS VALUE if the mechanical knee is later allowed
 * to move further.
 */
#define MAX_KNEE_ANGLE_DEG         100.0f


/*
 * Movement threshold.
 *
 * Robotic leg will only start following after the measured
 * knee angle exceeds 5 degrees.
 */
#define MOVE_START_ANGLE_DEG       5.0f


/*
 * Once movement has started, it stops following when the
 * knee returns below 3 degrees.
 *
 * This creates hysteresis and prevents shaking around 5 deg.
 */
#define MOVE_STOP_ANGLE_DEG        3.0f


/*
 * Ignore very small changes in command angle.
 */
#define COMMAND_DEADBAND_DEG       0.5f


/* ============================================================
   IMU ORIENTATION SETTINGS
   ============================================================ */

/*
 * Your two IMUs are physically mounted differently.
 *
 * Therefore each sensor gets its own:
 *
 *     axis selection
 *     sign
 *
 *
 * USE_PITCH:
 *
 *     1 = use pitch
 *     0 = use roll
 *
 *
 * AXIS_SIGN:
 *
 *     +1.0 = normal
 *     -1.0 = invert sensor direction
 */


/*
 * UPPER THIGH IMU
 *
 * GY-87 mounted on main PCB.
 */
#define THIGH_USE_PITCH            1
#define THIGH_AXIS_SIGN            1.0f


/*
 * LOWER LEG IMU
 *
 * MPU6050 is mounted flipped relative to the thigh sensor,
 * therefore start with an inverted sign.
 */
#define SHIN_USE_PITCH             1
#define SHIN_AXIS_SIGN            -1.0f


/*
 * Overall knee direction.
 *
 * If bending still produces negative RAW KNEE values,
 * change this from +1.0f to -1.0f.
 */
#define KNEE_DIRECTION             1.0f


/* ============================================================
   FILTER / CALIBRATION
   ============================================================ */

#define GYRO_CAL_SAMPLES           2000

/*
 * Let complementary filters settle before defining straight
 * leg as zero.
 */
#define FILTER_SETTLE_MS           2000

/*
 * Number of samples used to define straight leg = 0 degrees.
 *
 * 100 samples at 100 Hz ≈ 1 second.
 */
#define ZERO_CAL_SAMPLES           100


/*
 * Complementary filter:
 *
 * 98% gyro
 *  2% accelerometer
 */
#define FILTER_ALPHA               0.98f


/* ============================================================
   BLUETOOTH
   ============================================================ */

#define BT_DEVICE_NAME             "KNEE_LEADER"
#define BT_SERVER_NAME             "KNEE_ANGLE"


/* ============================================================
   I2C
   ============================================================ */

#define I2C_SPEED_HZ               400000
#define I2C_TIMEOUT_MS             100


/*
 * THIGH IMU
 *
 * SDA = GPIO32
 * SCL = GPIO33
 */
#define THIGH_I2C_PORT             I2C_NUM_0
#define THIGH_SDA                  32
#define THIGH_SCL                  33
#define THIGH_ADDR                 0x68


/*
 * SHIN IMU
 *
 * SDA = GPIO25
 * SCL = GPIO26
 */
#define SHIN_I2C_PORT              I2C_NUM_1
#define SHIN_SDA                   25
#define SHIN_SCL                   26
#define SHIN_ADDR                  0x68


/* ============================================================
   MPU6050 REGISTERS
   ============================================================ */

#define REG_PWR_MGMT_1             0x6B
#define REG_ACCEL_CONFIG           0x1C
#define REG_GYRO_CONFIG            0x1B
#define REG_ACCEL_XOUT_H           0x3B
#define REG_WHO_AM_I               0x75


/* ============================================================
   MPU6050 SCALE FACTORS
   ============================================================ */

#define ACCEL_SCALE                16384.0f
#define GYRO_SCALE                 131.0f

#define RAD_TO_DEG                 57.2957795f


static const char *TAG = "KNEE_LEADER";


/* ============================================================
   DATA STRUCTURES
   ============================================================ */

typedef struct
{
    float roll;
    float pitch;

    bool initialized;

} complementary_filter_t;


typedef struct
{
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;

    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;

    complementary_filter_t filter;

} imu_t;


typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

} imu_raw_t;


/* ============================================================
   BLUETOOTH STATE
   ============================================================ */

static volatile bool bt_connected = false;
static volatile bool bt_tx_ready = true;
static volatile bool bt_congested = false;

static uint32_t bt_handle = 0;


/* ============================================================
   BASIC FUNCTIONS
   ============================================================ */

static inline int16_t be16(const uint8_t *p)
{
    return (int16_t)((p[0] << 8) | p[1]);
}


/* ============================================================
   WRAP ANGLE TO -180 ... +180
   ============================================================ */

static float wrap_angle_deg(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}


/* ============================================================
   CLAMP FLOAT
   ============================================================ */

static float clamp_float(
        float value,
        float minimum,
        float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}


/* ============================================================
   CREATE I2C BUS + MPU6050 DEVICE
   ============================================================ */

static esp_err_t add_i2c_bus_and_device(
        i2c_port_num_t port,
        int sda,
        int scl,
        uint8_t address,
        i2c_master_bus_handle_t *out_bus,
        i2c_master_dev_handle_t *out_dev)
{
    i2c_master_bus_config_t bus_config =
    {
        .i2c_port = port,
        .sda_io_num = sda,
        .scl_io_num = scl,

        .clk_source =
                I2C_CLK_SRC_DEFAULT,

        .glitch_ignore_cnt = 7,

        .flags.enable_internal_pullup = true,
    };


    esp_err_t err =
            i2c_new_master_bus(
                    &bus_config,
                    out_bus);


    if (err != ESP_OK)
    {
        return err;
    }


    i2c_device_config_t device_config =
    {
        .dev_addr_length =
                I2C_ADDR_BIT_LEN_7,

        .device_address =
                address,

        .scl_speed_hz =
                I2C_SPEED_HZ,
    };


    return i2c_master_bus_add_device(
            *out_bus,
            &device_config,
            out_dev);
}


/* ============================================================
   MPU6050 WRITE REGISTER
   ============================================================ */

static esp_err_t mpu_write_reg(
        i2c_master_dev_handle_t dev,
        uint8_t reg,
        uint8_t value)
{
    uint8_t data[2] =
    {
        reg,
        value
    };


    return i2c_master_transmit(
            dev,
            data,
            sizeof(data),
            I2C_TIMEOUT_MS);
}


/* ============================================================
   MPU6050 READ
   ============================================================ */

static esp_err_t mpu_read(
        i2c_master_dev_handle_t dev,
        uint8_t reg,
        uint8_t *data,
        size_t length)
{
    return i2c_master_transmit_receive(
            dev,
            &reg,
            1,
            data,
            length,
            I2C_TIMEOUT_MS);
}


/* ============================================================
   MPU6050 INITIALISATION
   ============================================================ */

static bool mpu_init(
        i2c_master_dev_handle_t dev,
        const char *name)
{
    uint8_t who = 0;


    if (mpu_read(
            dev,
            REG_WHO_AM_I,
            &who,
            1) != ESP_OK)
    {
        ESP_LOGE(
                TAG,
                "%s not responding",
                name);

        return false;
    }


    ESP_LOGI(
            TAG,
            "%s WHO_AM_I = 0x%02X",
            name,
            who);


    /*
     * Wake MPU6050
     */

    ESP_ERROR_CHECK(
            mpu_write_reg(
                    dev,
                    REG_PWR_MGMT_1,
                    0x00));


    vTaskDelay(
            pdMS_TO_TICKS(100));


    /*
     * Accelerometer:
     *
     * +/- 2g
     */

    ESP_ERROR_CHECK(
            mpu_write_reg(
                    dev,
                    REG_ACCEL_CONFIG,
                    0x00));


    /*
     * Gyroscope:
     *
     * +/-250 deg/s
     */

    ESP_ERROR_CHECK(
            mpu_write_reg(
                    dev,
                    REG_GYRO_CONFIG,
                    0x00));


    ESP_LOGI(
            TAG,
            "%s initialized",
            name);


    return true;
}


/* ============================================================
   READ RAW MPU6050 DATA
   ============================================================ */

static bool mpu_read_raw(
        i2c_master_dev_handle_t dev,
        imu_raw_t *raw)
{
    uint8_t data[14];


    if (mpu_read(
            dev,
            REG_ACCEL_XOUT_H,
            data,
            sizeof(data)) != ESP_OK)
    {
        return false;
    }


    raw->ax = be16(&data[0]);
    raw->ay = be16(&data[2]);
    raw->az = be16(&data[4]);

    raw->gx = be16(&data[8]);
    raw->gy = be16(&data[10]);
    raw->gz = be16(&data[12]);


    return true;
}


/* ============================================================
   ACCELEROMETER ROLL + PITCH
   ============================================================ */

static void calculate_roll_pitch(
        float ax,
        float ay,
        float az,
        float *roll,
        float *pitch)
{
    /*
     * Roll = rotation around X axis
     */

    *roll =
            atan2f(
                    ay,
                    az)
            * RAD_TO_DEG;


    /*
     * Pitch = rotation around Y axis
     */

    *pitch =
            atan2f(
                    -ax,
                    sqrtf(
                            ay * ay +
                            az * az))
            * RAD_TO_DEG;
}


/* ============================================================
   COMPLEMENTARY FILTER
   ============================================================ */

static void complementary_filter_update(
        complementary_filter_t *filter,
        float accel_roll,
        float accel_pitch,
        float gyro_x,
        float gyro_y,
        float dt)
{
    /*
     * First sample:
     *
     * Initialize using accelerometer.
     */

    if (!filter->initialized)
    {
        filter->roll =
                accel_roll;

        filter->pitch =
                accel_pitch;

        filter->initialized =
                true;

        return;
    }


    /*
     * Gyroscope prediction
     */

    float predicted_roll =
            wrap_angle_deg(
                    filter->roll +
                    gyro_x * dt);


    float predicted_pitch =
            wrap_angle_deg(
                    filter->pitch +
                    gyro_y * dt);


    /*
     * Find shortest angle error.
     */

    float roll_error =
            wrap_angle_deg(
                    accel_roll -
                    predicted_roll);


    float pitch_error =
            wrap_angle_deg(
                    accel_pitch -
                    predicted_pitch);


    /*
     * Complementary filter
     */

    filter->roll =
            wrap_angle_deg(
                    predicted_roll +
                    (1.0f - FILTER_ALPHA) *
                    roll_error);


    filter->pitch =
            wrap_angle_deg(
                    predicted_pitch +
                    (1.0f - FILTER_ALPHA) *
                    pitch_error);
}


/* ============================================================
   UPDATE ONE IMU
   ============================================================ */

static bool update_imu(
        imu_t *imu,
        float dt)
{
    imu_raw_t raw;


    if (!mpu_read_raw(
            imu->dev,
            &raw))
    {
        return false;
    }


    /*
     * Convert accelerometer data to g.
     */

    float ax =
            raw.ax /
            ACCEL_SCALE;


    float ay =
            raw.ay /
            ACCEL_SCALE;


    float az =
            raw.az /
            ACCEL_SCALE;


    float accel_roll;
    float accel_pitch;


    calculate_roll_pitch(
            ax,
            ay,
            az,
            &accel_roll,
            &accel_pitch);


    /*
     * Correct gyro offsets.
     */

    float gx =
            (raw.gx -
             imu->gyro_bias_x) /
            GYRO_SCALE;


    float gy =
            (raw.gy -
             imu->gyro_bias_y) /
            GYRO_SCALE;


    /*
     * Update filter.
     */

    complementary_filter_update(
            &imu->filter,

            accel_roll,
            accel_pitch,

            gx,
            gy,

            dt);


    return true;
}


/* ============================================================
   CALIBRATE BOTH GYROSCOPES
   ============================================================ */

static bool calibrate_gyros(
        imu_t *thigh,
        imu_t *shin)
{
    int64_t thigh_gx = 0;
    int64_t thigh_gy = 0;
    int64_t thigh_gz = 0;

    int64_t shin_gx = 0;
    int64_t shin_gy = 0;
    int64_t shin_gz = 0;

    int valid = 0;


    ESP_LOGI(TAG, "");
    ESP_LOGI(
            TAG,
            "======================================");

    ESP_LOGI(
            TAG,
            "GYRO CALIBRATION");

    ESP_LOGI(
            TAG,
            "KEEP BOTH LEGS/IMUs COMPLETELY STILL");

    ESP_LOGI(
            TAG,
            "======================================");


    /*
     * Give sensors time to settle.
     */

    vTaskDelay(
            pdMS_TO_TICKS(2000));


    for (int i = 0;
         i < GYRO_CAL_SAMPLES;
         i++)
    {
        imu_raw_t thigh_raw;
        imu_raw_t shin_raw;


        if (mpu_read_raw(
                thigh->dev,
                &thigh_raw) &&
            mpu_read_raw(
                shin->dev,
                &shin_raw))
        {
            thigh_gx +=
                    thigh_raw.gx;

            thigh_gy +=
                    thigh_raw.gy;

            thigh_gz +=
                    thigh_raw.gz;


            shin_gx +=
                    shin_raw.gx;

            shin_gy +=
                    shin_raw.gy;

            shin_gz +=
                    shin_raw.gz;


            valid++;
        }


        esp_rom_delay_us(1000);
    }


    if (valid == 0)
    {
        ESP_LOGE(
                TAG,
                "Gyro calibration failed");

        return false;
    }


    thigh->gyro_bias_x =
            (float)thigh_gx /
            valid;

    thigh->gyro_bias_y =
            (float)thigh_gy /
            valid;

    thigh->gyro_bias_z =
            (float)thigh_gz /
            valid;


    shin->gyro_bias_x =
            (float)shin_gx /
            valid;

    shin->gyro_bias_y =
            (float)shin_gy /
            valid;

    shin->gyro_bias_z =
            (float)shin_gz /
            valid;


    ESP_LOGI(
            TAG,
            "Gyro calibration complete");


    return true;
}


/* ============================================================
   GET THIGH KNEE AXIS
   ============================================================ */

static float get_thigh_angle(
        const imu_t *imu)
{
#if THIGH_USE_PITCH

    return
            THIGH_AXIS_SIGN *
            imu->filter.pitch;

#else

    return
            THIGH_AXIS_SIGN *
            imu->filter.roll;

#endif
}


/* ============================================================
   GET SHIN KNEE AXIS
   ============================================================ */

static float get_shin_angle(
        const imu_t *imu)
{
#if SHIN_USE_PITCH

    return
            SHIN_AXIS_SIGN *
            imu->filter.pitch;

#else

    return
            SHIN_AXIS_SIGN *
            imu->filter.roll;

#endif
}


/* ============================================================
   FILTER SETTLING

   Run both complementary filters before defining knee zero.
   ============================================================ */

static bool settle_filters(
        imu_t *thigh,
        imu_t *shin)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(
            TAG,
            "FILTER SETTLING");

    ESP_LOGI(
            TAG,
            "KEEP LEG STRAIGHT AND STILL");


    int samples =
            FILTER_SETTLE_MS /
            SAMPLE_PERIOD_MS;


    int64_t previous_time =
            esp_timer_get_time();


    for (int i = 0;
         i < samples;
         i++)
    {
        vTaskDelay(
                pdMS_TO_TICKS(
                        SAMPLE_PERIOD_MS));


        int64_t now =
                esp_timer_get_time();


        float dt =
                (now -
                 previous_time) /
                1000000.0f;


        previous_time =
                now;


        if (dt <= 0.0f ||
            dt > 0.1f)
        {
            dt = 0.01f;
        }


        if (!update_imu(
                thigh,
                dt))
        {
            return false;
        }


        if (!update_imu(
                shin,
                dt))
        {
            return false;
        }
    }


    ESP_LOGI(
            TAG,
            "Filters settled");


    return true;
}


/* ============================================================
   CALIBRATE STRAIGHT LEG ZERO
   ============================================================ */

static bool calibrate_knee_zero(
        imu_t *thigh,
        imu_t *shin,
        float *zero_reference)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(
            TAG,
            "======================================");

    ESP_LOGI(
            TAG,
            "STRAIGHT LEG ZERO CALIBRATION");

    ESP_LOGI(
            TAG,
            "KEEP LEG COMPLETELY STRAIGHT");

    ESP_LOGI(
            TAG,
            "======================================");


    float zero_sum = 0.0f;

    int valid_samples = 0;


    int64_t previous_time =
            esp_timer_get_time();


    for (int i = 0;
         i < ZERO_CAL_SAMPLES;
         i++)
    {
        vTaskDelay(
                pdMS_TO_TICKS(
                        SAMPLE_PERIOD_MS));


        int64_t now =
                esp_timer_get_time();


        float dt =
                (now -
                 previous_time) /
                1000000.0f;


        previous_time =
                now;


        if (dt <= 0.0f ||
            dt > 0.1f)
        {
            dt = 0.01f;
        }


        if (!update_imu(
                thigh,
                dt))
        {
            continue;
        }


        if (!update_imu(
                shin,
                dt))
        {
            continue;
        }


        float thigh_angle =
                get_thigh_angle(
                        thigh);


        float shin_angle =
                get_shin_angle(
                        shin);


        float relative_angle =
                wrap_angle_deg(
                        shin_angle -
                        thigh_angle);


        zero_sum +=
                relative_angle;


        valid_samples++;
    }


    if (valid_samples == 0)
    {
        ESP_LOGE(
                TAG,
                "Knee zero calibration failed");

        return false;
    }


    *zero_reference =
            zero_sum /
            valid_samples;


    ESP_LOGI(TAG, "");
    ESP_LOGI(
            TAG,
            "======================================");

    ESP_LOGI(
            TAG,
            "KNEE ZERO CALIBRATED");

    ESP_LOGI(
            TAG,
            "Zero reference = %.2f deg",
            *zero_reference);

    ESP_LOGI(
            TAG,
            "YOU MAY NOW MOVE YOUR LEG");

    ESP_LOGI(
            TAG,
            "======================================");


    return true;
}


/* ============================================================
   SAFE COMMAND ANGLE
   ============================================================ */

static float update_command_angle(
        float measured_angle)
{
    static bool tracking = false;

    static float command_angle =
            0.0f;


    /*
     * Start / stop hysteresis
     */

    if (!tracking)
    {
        if (measured_angle >=
            MOVE_START_ANGLE_DEG)
        {
            tracking = true;
        }
    }
    else
    {
        if (measured_angle <=
            MOVE_STOP_ANGLE_DEG)
        {
            tracking = false;
        }
    }


    /*
     * When standing straight:
     *
     * command robotic knee to 0 degrees.
     */

    if (!tracking)
    {
        command_angle =
                0.0f;

        return
                command_angle;
    }


    /*
     * Apply maximum-angle failsafe.
     */

    float desired =
            clamp_float(
                    measured_angle,
                    0.0f,
                    MAX_KNEE_ANGLE_DEG);


    /*
     * Ignore tiny command changes.
     */

    if (fabsf(
            desired -
            command_angle) >=
        COMMAND_DEADBAND_DEG)
    {
        command_angle =
                desired;
    }


    /*
     * Maximum clamp always has priority.
     */

    if (desired >=
        MAX_KNEE_ANGLE_DEG)
    {
        command_angle =
                MAX_KNEE_ANGLE_DEG;
    }


    return
            command_angle;
}


/* ============================================================
   BLUETOOTH CALLBACK
   ============================================================ */

static void spp_callback(
        esp_spp_cb_event_t event,
        esp_spp_cb_param_t *param)
{
    switch (event)
    {
        case ESP_SPP_INIT_EVT:
        {
            ESP_LOGI(
                    TAG,
                    "Bluetooth SPP initialized");


            ESP_ERROR_CHECK(
                    esp_spp_start_srv(
                            ESP_SPP_SEC_NONE,
                            ESP_SPP_ROLE_SLAVE,
                            0,
                            BT_SERVER_NAME));


            ESP_ERROR_CHECK(
                    esp_bt_gap_set_scan_mode(
                            ESP_BT_CONNECTABLE,
                            ESP_BT_GENERAL_DISCOVERABLE));

            break;
        }


        case ESP_SPP_START_EVT:
        {
            ESP_LOGI(
                    TAG,
                    "Bluetooth server started");

            ESP_LOGI(
                    TAG,
                    "Bluetooth name: %s",
                    BT_DEVICE_NAME);

            ESP_LOGI(
                    TAG,
                    "Waiting for follower...");

            break;
        }


        case ESP_SPP_SRV_OPEN_EVT:
        {
            bt_handle =
                    param->srv_open.handle;

            bt_connected =
                    true;

            bt_tx_ready =
                    true;

            bt_congested =
                    false;


            ESP_LOGI(
                    TAG,
                    "FOLLOWER CONNECTED");

            break;
        }


        case ESP_SPP_CLOSE_EVT:
        {
            bt_connected =
                    false;

            bt_tx_ready =
                    true;

            bt_congested =
                    false;


            ESP_LOGW(
                    TAG,
                    "FOLLOWER DISCONNECTED");


            esp_bt_gap_set_scan_mode(
                    ESP_BT_CONNECTABLE,
                    ESP_BT_GENERAL_DISCOVERABLE);

            break;
        }


        case ESP_SPP_CONG_EVT:
        {
            bt_congested =
                    param->cong.cong;

            break;
        }


        case ESP_SPP_WRITE_EVT:
        {
            bt_congested =
                    param->write.cong;

            bt_tx_ready =
                    true;

            break;
        }


        default:
        {
            break;
        }
    }
}


/* ============================================================
   INITIALISE BLUETOOTH
   ============================================================ */

static void bluetooth_init(void)
{
    /*
     * NVS initialization.
     */

    esp_err_t ret =
            nvs_flash_init();


    if (ret ==
            ESP_ERR_NVS_NO_FREE_PAGES ||
        ret ==
            ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(
                nvs_flash_erase());


        ESP_ERROR_CHECK(
                nvs_flash_init());
    }
    else
    {
        ESP_ERROR_CHECK(
                ret);
    }


    /*
     * Bluetooth controller configuration.
     *
     * sdkconfig must be configured for:
     *
     *     Classic Bluetooth
     *     BR/EDR Only
     *     SPP enabled
     */

    esp_bt_controller_config_t bt_cfg =
            BT_CONTROLLER_INIT_CONFIG_DEFAULT();


    ESP_ERROR_CHECK(
            esp_bt_controller_init(
                    &bt_cfg));


    ESP_ERROR_CHECK(
            esp_bt_controller_enable(
                    ESP_BT_MODE_CLASSIC_BT));


    /*
     * Initialize Bluedroid.
     */

    ESP_ERROR_CHECK(
            esp_bluedroid_init());


    ESP_ERROR_CHECK(
            esp_bluedroid_enable());


    /*
     * Set visible Bluetooth name.
     */

    ESP_ERROR_CHECK(
            esp_bt_gap_set_device_name(
                    BT_DEVICE_NAME));


    /*
     * Register SPP callback.
     */

    ESP_ERROR_CHECK(
            esp_spp_register_callback(
                    spp_callback));


    /*
     * ESP-IDF 5.3 SPP initialization.
     */

    esp_spp_cfg_t spp_cfg =
    {
        .mode =
                ESP_SPP_MODE_CB,

        .enable_l2cap_ertm =
                true,

        .tx_buffer_size =
                0
    };


    ESP_ERROR_CHECK(
            esp_spp_enhanced_init(
                    &spp_cfg));
}


/* ============================================================
   SEND KNEE ANGLE OVER BLUETOOTH
   ============================================================ */

static void bluetooth_send_angle(
        float angle)
{
    /*
     * Do nothing if follower is not connected.
     */

    if (!bt_connected ||
        !bt_tx_ready ||
        bt_congested)
    {
        return;
    }


    /*
     * Message example:
     *
     *     37.52\n
     */

    char message[24];


    int length =
            snprintf(
                    message,
                    sizeof(message),
                    "%.2f\n",
                    angle);


    if (length <= 0)
    {
        return;
    }


    bt_tx_ready =
            false;


    esp_err_t err =
            esp_spp_write(
                    bt_handle,
                    length,
                    (uint8_t *)message);


    if (err != ESP_OK)
    {
        bt_tx_ready =
                true;
    }
}


/* ============================================================
   MAIN
   ============================================================ */

void app_main(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(
            TAG,
            "======================================");

    ESP_LOGI(
            TAG,
            "LEADER KNEE ANGLE SYSTEM");

    ESP_LOGI(
            TAG,
            "Dual MPU6050 + Bluetooth");

    ESP_LOGI(
            TAG,
            "Maximum command = %.1f deg",
            MAX_KNEE_ANGLE_DEG);

    ESP_LOGI(
            TAG,
            "======================================");


    /* ========================================================
       IMU OBJECTS
       ======================================================== */

    imu_t thigh = {0};

    imu_t shin = {0};


    /* ========================================================
       THIGH IMU
       SDA 32
       SCL 33
       ======================================================== */

    ESP_ERROR_CHECK(
            add_i2c_bus_and_device(
                    THIGH_I2C_PORT,
                    THIGH_SDA,
                    THIGH_SCL,
                    THIGH_ADDR,

                    &thigh.bus,
                    &thigh.dev));


    /* ========================================================
       SHIN IMU
       SDA 25
       SCL 26
       ======================================================== */

    ESP_ERROR_CHECK(
            add_i2c_bus_and_device(
                    SHIN_I2C_PORT,
                    SHIN_SDA,
                    SHIN_SCL,
                    SHIN_ADDR,

                    &shin.bus,
                    &shin.dev));


    /* ========================================================
       INITIALISE IMUs
       ======================================================== */

    if (!mpu_init(
            thigh.dev,
            "THIGH IMU"))
    {
        return;
    }


    if (!mpu_init(
            shin.dev,
            "SHIN IMU"))
    {
        return;
    }


    /* ========================================================
       GYRO CALIBRATION
       ======================================================== */

    if (!calibrate_gyros(
            &thigh,
            &shin))
    {
        return;
    }


    /* ========================================================
       COMPLEMENTARY FILTER SETTLING
       ======================================================== */

    if (!settle_filters(
            &thigh,
            &shin))
    {
        ESP_LOGE(
                TAG,
                "Filter settling failed");

        return;
    }


    /* ========================================================
       STRAIGHT-LEG ZERO CALIBRATION
       ======================================================== */

    float zero_reference =
            0.0f;


    if (!calibrate_knee_zero(
            &thigh,
            &shin,
            &zero_reference))
    {
        return;
    }


    /* ========================================================
       START BLUETOOTH
       ======================================================== */

    bluetooth_init();


    /* ========================================================
       MAIN LOOP TIMING
       ======================================================== */

    int64_t previous_time =
            esp_timer_get_time();


    uint32_t sample_count =
            0;


    /* ========================================================
       MAIN LOOP
       ======================================================== */

    while (1)
    {
        vTaskDelay(
                pdMS_TO_TICKS(
                        SAMPLE_PERIOD_MS));


        int64_t now =
                esp_timer_get_time();


        float dt =
                (now -
                 previous_time) /
                1000000.0f;


        previous_time =
                now;


        /*
         * Protect filter from abnormal dt.
         */

        if (dt <= 0.0f ||
            dt > 0.1f)
        {
            dt =
                    0.01f;
        }


        /* ====================================================
           UPDATE BOTH IMUs
           ==================================================== */

        if (!update_imu(
                &thigh,
                dt))
        {
            ESP_LOGE(
                    TAG,
                    "THIGH IMU read failed");

            continue;
        }


        if (!update_imu(
                &shin,
                dt))
        {
            ESP_LOGE(
                    TAG,
                    "SHIN IMU read failed");

            continue;
        }


        /* ====================================================
           GET KNEE AXIS FROM EACH IMU
           ==================================================== */

        float thigh_angle =
                get_thigh_angle(
                        &thigh);


        float shin_angle =
                get_shin_angle(
                        &shin);


        /* ====================================================
           RELATIVE THIGH-SHIN ANGLE
           ==================================================== */

        float relative_angle =
                wrap_angle_deg(
                        shin_angle -
                        thigh_angle);


        /* ====================================================
           RAW KNEE ANGLE

           IMPORTANT:

           This value is NOT clamped to zero.

           Therefore if orientation is wrong we will see a
           negative number instead of hiding the problem.
           ==================================================== */

        float raw_knee_angle =
                KNEE_DIRECTION *
                wrap_angle_deg(
                        relative_angle -
                        zero_reference);


        /* ====================================================
           MEASURED KNEE ANGLE

           Once the orientation is correct, negative values
           represent tiny extension/noise and are treated as 0.
           ==================================================== */

        float knee_angle =
                raw_knee_angle;


        if (knee_angle < 0.0f)
        {
            knee_angle =
                    0.0f;
        }


        /* ====================================================
           COMMAND ANGLE
           ==================================================== */

        float command_angle =
                update_command_angle(
                        knee_angle);


        sample_count++;


        /* ====================================================
           TERMINAL OUTPUT

           Shows BOTH roll and pitch.

           This lets us confirm which physical MPU axis is
           actually following your knee.
           ==================================================== */

        if ((sample_count %
             TERMINAL_PRINT_DIV) == 0)
        {
            printf(
                "KNEE=%6.2f | "
                "RAW=%+7.2f | "
                "CMD=%6.2f | "
                "THIGH R=%+7.2f P=%+7.2f | "
                "SHIN R=%+7.2f P=%+7.2f | "
                "BT=%s\n",

                knee_angle,

                raw_knee_angle,

                command_angle,

                thigh.filter.roll,
                thigh.filter.pitch,

                shin.filter.roll,
                shin.filter.pitch,

                bt_connected
                    ? "CONNECTED"
                    : "WAITING");
        }


        /* ====================================================
           SEND COMMAND TO FOLLOWER

           Approximately 50 Hz
           ==================================================== */

        if ((sample_count %
             BT_SEND_DIV) == 0)
        {
            bluetooth_send_angle(
                    command_angle);
        }
    }
}