#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
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

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"

/* ===================== USER SETTINGS ===================== */

#define SAMPLE_PERIOD_MS 10
#define TERMINAL_PRINT_DIV 10
#define BT_SEND_DIV 2
#define WS_SEND_DIV 2
#define DISPLAY_LPF_ALPHA 0.2f

/* ===================== KNEE CONTROL ===================== */

#define MIN_KNEE_ANGLE_DEG 0.0f
#define MAX_KNEE_ANGLE_DEG 100.0f
#define KNEE_ZERO_DEADBAND_DEG 1.0f

#define MOVE_START_ANGLE_DEG 5.0f
#define MOVE_STOP_ANGLE_DEG 3.0f
#define COMMAND_DEADBAND_DEG 0.5f

/* ===================== IMU ORIENTATION ===================== */

/*
 * THIGH IMU:
 * X points right
 * Y points toward ESP32
 *
 * SHIN IMU:
 * X points downward
 * Y points left
 *
 * Z axes point in opposite directions.
 */

#define THIGH_AXIS_SIGN 1.0f
#define SHIN_AXIS_SIGN -1.0f
#define KNEE_DIRECTION -1.0f

/* ===================== FILTER / CALIBRATION ===================== */

#define GYRO_CAL_SAMPLES 2000
#define FILTER_SETTLE_MS 2000
#define ZERO_CAL_SAMPLES 100

#define FILTER_ALPHA 0.98f
#define ACCEL_XY_MIN_G 0.25f

/* ===================== BLUETOOTH ===================== */

#define BT_DEVICE_NAME "KNEE_LEADER"
#define BT_SERVER_NAME "KNEE_ANGLE"

/*
 * Follower sends:
 *
 * M,37.52\n
 *
 * If no feedback arrives within this period,
 * the follower measurement is considered stale.
 */
#define FOLLOWER_FEEDBACK_TIMEOUT_MS 500

/* ===================== WIFI ===================== */

#define WIFI_AP_SSID "KNEE_LEADER_WIFI"
#define WIFI_AP_PASS "kneeangle123"
#define WIFI_AP_CHANNEL 1
#define WIFI_AP_MAX_CONN 4

#define WS_URI "/ws"

/* ===================== I2C ===================== */

#define I2C_SPEED_HZ 400000
#define I2C_TIMEOUT_MS 100

#define THIGH_I2C_PORT I2C_NUM_0
#define THIGH_SDA 32
#define THIGH_SCL 33
#define THIGH_ADDR 0x68

#define SHIN_I2C_PORT I2C_NUM_1
#define SHIN_SDA 25
#define SHIN_SCL 26
#define SHIN_ADDR 0x68

/* ===================== MPU6050 ===================== */

#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_WHO_AM_I 0x75

#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE 131.0f
#define RAD_TO_DEG 57.2957795f

static const char *TAG="KNEE_LEADER";

/* ===================== DATA TYPES ===================== */

typedef struct{
    float flexion_angle;
    bool initialized;
}complementary_filter_t;

typedef struct{
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    float gyro_bias_z;
    complementary_filter_t filter;
}imu_t;

typedef struct{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
}imu_raw_t;

/* ===================== BLUETOOTH STATE ===================== */

static volatile bool bt_connected=false;
static volatile bool bt_tx_ready=true;
static volatile bool bt_congested=false;

static uint32_t bt_handle=0;

/* ===================== FOLLOWER FEEDBACK ===================== */

static volatile float follower_motor_angle=0.0f;
static volatile bool follower_angle_valid=false;
static volatile int64_t follower_last_rx_us=0;

static char follower_rx_buffer[32];
static int follower_rx_position=0;
static bool follower_rx_discarding=false;

/* ===================== WEBSOCKET STATE ===================== */

static httpd_handle_t ws_server=NULL;
static int ws_client_fd=-1;
static volatile bool ws_connected=false;

/* ===================== HELPERS ===================== */

static inline int16_t be16(const uint8_t *p){
    return (int16_t)((p[0]<<8)|p[1]);
}

static float wrap_angle_deg(float angle){
    while(angle>180.0f)
        angle-=360.0f;

    while(angle<-180.0f)
        angle+=360.0f;

    return angle;
}

static float clamp_float(float value,float minimum,float maximum){
    if(value<minimum)
        return minimum;

    if(value>maximum)
        return maximum;

    return value;
}

/* ===================== I2C ===================== */

static esp_err_t add_i2c_bus_and_device(
    i2c_port_num_t port,
    int sda,
    int scl,
    uint8_t address,
    i2c_master_bus_handle_t *out_bus,
    i2c_master_dev_handle_t *out_dev){

    i2c_master_bus_config_t bus_config={
        .i2c_port=port,
        .sda_io_num=sda,
        .scl_io_num=scl,
        .clk_source=I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt=7,
        .flags.enable_internal_pullup=true
    };

    esp_err_t err=i2c_new_master_bus(
        &bus_config,
        out_bus);

    if(err!=ESP_OK)
        return err;

    i2c_device_config_t device_config={
        .dev_addr_length=I2C_ADDR_BIT_LEN_7,
        .device_address=address,
        .scl_speed_hz=I2C_SPEED_HZ
    };

    return i2c_master_bus_add_device(
        *out_bus,
        &device_config,
        out_dev);
}

/* ===================== MPU6050 ===================== */

static esp_err_t mpu_write_reg(
    i2c_master_dev_handle_t dev,
    uint8_t reg,
    uint8_t value){

    uint8_t data[2]={
        reg,
        value
    };

    return i2c_master_transmit(
        dev,
        data,
        sizeof(data),
        I2C_TIMEOUT_MS);
}

static esp_err_t mpu_read(
    i2c_master_dev_handle_t dev,
    uint8_t reg,
    uint8_t *data,
    size_t length){

    return i2c_master_transmit_receive(
        dev,
        &reg,
        1,
        data,
        length,
        I2C_TIMEOUT_MS);
}

static bool mpu_init(
    i2c_master_dev_handle_t dev,
    const char *name){

    uint8_t who=0;

    if(mpu_read(
        dev,
        REG_WHO_AM_I,
        &who,
        1)!=ESP_OK){

        ESP_LOGE(TAG,"%s not responding",name);
        return false;
    }

    ESP_LOGI(
        TAG,
        "%s WHO_AM_I = 0x%02X",
        name,
        who);

    ESP_ERROR_CHECK(
        mpu_write_reg(
            dev,
            REG_PWR_MGMT_1,
            0x00));

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_ERROR_CHECK(
        mpu_write_reg(
            dev,
            REG_ACCEL_CONFIG,
            0x00));

    ESP_ERROR_CHECK(
        mpu_write_reg(
            dev,
            REG_GYRO_CONFIG,
            0x00));

    ESP_LOGI(TAG,"%s initialized",name);

    return true;
}

static bool mpu_read_raw(
    i2c_master_dev_handle_t dev,
    imu_raw_t *raw){

    uint8_t data[14];

    if(mpu_read(
        dev,
        REG_ACCEL_XOUT_H,
        data,
        sizeof(data))!=ESP_OK){

        return false;
    }

    raw->ax=be16(&data[0]);
    raw->ay=be16(&data[2]);
    raw->az=be16(&data[4]);

    raw->gx=be16(&data[8]);
    raw->gy=be16(&data[10]);
    raw->gz=be16(&data[12]);

    return true;
}

/* ===================== ACCELEROMETER Z ANGLE ===================== */

static bool calculate_accel_z_angle(
    float ax,
    float ay,
    float *angle){

    float xy_magnitude=
        sqrtf(
            ax*ax+
            ay*ay);

    if(xy_magnitude<ACCEL_XY_MIN_G)
        return false;

    *angle=
        wrap_angle_deg(
            -atan2f(
                ay,
                ax)*
            RAD_TO_DEG);

    return true;
}

/* ===================== COMPLEMENTARY FILTER ===================== */

static void complementary_filter_update(
    complementary_filter_t *filter,
    float accel_angle,
    bool accel_valid,
    float gyro_z,
    float dt){

    if(!filter->initialized){

        if(accel_valid)
            filter->flexion_angle=accel_angle;
        else
            filter->flexion_angle=0.0f;

        filter->initialized=true;
        return;
    }

    float predicted_angle=
        wrap_angle_deg(
            filter->flexion_angle+
            gyro_z*dt);

    if(accel_valid){

        float error=
            wrap_angle_deg(
                accel_angle-
                predicted_angle);

        filter->flexion_angle=
            wrap_angle_deg(
                predicted_angle+
                (1.0f-FILTER_ALPHA)*
                error);
    }
    else{
        filter->flexion_angle=
            predicted_angle;
    }
}

static bool update_imu(
    imu_t *imu,
    float dt){

    imu_raw_t raw;

    if(!mpu_read_raw(
        imu->dev,
        &raw)){

        return false;
    }

    float ax=
        raw.ax/
        ACCEL_SCALE;

    float ay=
        raw.ay/
        ACCEL_SCALE;

    float accel_angle=0.0f;

    bool accel_valid=
        calculate_accel_z_angle(
            ax,
            ay,
            &accel_angle);

    float gyro_z=
        (raw.gz-
         imu->gyro_bias_z)/
        GYRO_SCALE;

    complementary_filter_update(
        &imu->filter,
        accel_angle,
        accel_valid,
        gyro_z,
        dt);

    return true;
}

/* ===================== GYRO CALIBRATION ===================== */

static bool calibrate_gyros(
    imu_t *thigh,
    imu_t *shin){

    int64_t thigh_gz=0;
    int64_t shin_gz=0;
    int valid=0;

    ESP_LOGI(TAG,"");
    ESP_LOGI(TAG,"======================================");
    ESP_LOGI(TAG,"GYRO CALIBRATION");
    ESP_LOGI(TAG,"KEEP BOTH LEGS/IMUs COMPLETELY STILL");
    ESP_LOGI(TAG,"======================================");

    vTaskDelay(pdMS_TO_TICKS(2000));

    for(int i=0;i<GYRO_CAL_SAMPLES;i++){

        imu_raw_t thigh_raw;
        imu_raw_t shin_raw;

        if(mpu_read_raw(
                thigh->dev,
                &thigh_raw)&&
           mpu_read_raw(
                shin->dev,
                &shin_raw)){

            thigh_gz+=thigh_raw.gz;
            shin_gz+=shin_raw.gz;
            valid++;
        }

        esp_rom_delay_us(1000);
    }

    if(valid==0){

        ESP_LOGE(TAG,"Gyro calibration failed");
        return false;
    }

    thigh->gyro_bias_z=
        (float)thigh_gz/
        valid;

    shin->gyro_bias_z=
        (float)shin_gz/
        valid;

    ESP_LOGI(
        TAG,
        "Thigh gyro-Z bias = %.2f",
        thigh->gyro_bias_z);

    ESP_LOGI(
        TAG,
        "Shin gyro-Z bias = %.2f",
        shin->gyro_bias_z);

    ESP_LOGI(TAG,"Gyro calibration complete");

    return true;
}

/* ===================== IMU ANGLES ===================== */

static float get_thigh_angle(const imu_t *imu){
    return
        THIGH_AXIS_SIGN*
        imu->filter.flexion_angle;
}

static float get_shin_angle(const imu_t *imu){
    return
        SHIN_AXIS_SIGN*
        imu->filter.flexion_angle;
}

/* ===================== FILTER SETTLE ===================== */

static bool settle_filters(
    imu_t *thigh,
    imu_t *shin){

    ESP_LOGI(TAG,"");
    ESP_LOGI(TAG,"FILTER SETTLING");
    ESP_LOGI(TAG,"KEEP LEG STRAIGHT AND STILL");

    int samples=
        FILTER_SETTLE_MS/
        SAMPLE_PERIOD_MS;

    int64_t previous_time=
        esp_timer_get_time();

    for(int i=0;i<samples;i++){

        vTaskDelay(
            pdMS_TO_TICKS(
                SAMPLE_PERIOD_MS));

        int64_t now=
            esp_timer_get_time();

        float dt=
            (now-
             previous_time)/
            1000000.0f;

        previous_time=now;

        if(dt<=0.0f||
           dt>0.1f){

            dt=0.01f;
        }

        if(!update_imu(
            thigh,
            dt)){

            return false;
        }

        if(!update_imu(
            shin,
            dt)){

            return false;
        }
    }

    ESP_LOGI(TAG,"Filters settled");

    return true;
}

/* ===================== KNEE ZERO ===================== */

static bool calibrate_knee_zero(
    imu_t *thigh,
    imu_t *shin,
    float *zero_reference){

    ESP_LOGI(TAG,"");
    ESP_LOGI(TAG,"======================================");
    ESP_LOGI(TAG,"STRAIGHT LEG ZERO CALIBRATION");
    ESP_LOGI(TAG,"KEEP LEG COMPLETELY STRAIGHT");
    ESP_LOGI(TAG,"======================================");

    float zero_sum=0.0f;
    int valid_samples=0;

    int64_t previous_time=
        esp_timer_get_time();

    for(int i=0;i<ZERO_CAL_SAMPLES;i++){

        vTaskDelay(
            pdMS_TO_TICKS(
                SAMPLE_PERIOD_MS));

        int64_t now=
            esp_timer_get_time();

        float dt=
            (now-
             previous_time)/
            1000000.0f;

        previous_time=now;

        if(dt<=0.0f||
           dt>0.1f){

            dt=0.01f;
        }

        if(!update_imu(
            thigh,
            dt)){

            continue;
        }

        if(!update_imu(
            shin,
            dt)){

            continue;
        }

        float thigh_angle=
            get_thigh_angle(
                thigh);

        float shin_angle=
            get_shin_angle(
                shin);

        float relative_angle=
            wrap_angle_deg(
                shin_angle-
                thigh_angle);

        zero_sum+=relative_angle;
        valid_samples++;
    }

    if(valid_samples==0){

        ESP_LOGE(
            TAG,
            "Knee zero calibration failed");

        return false;
    }

    *zero_reference=
        zero_sum/
        valid_samples;

    ESP_LOGI(TAG,"");
    ESP_LOGI(TAG,"======================================");
    ESP_LOGI(TAG,"KNEE ZERO CALIBRATED");

    ESP_LOGI(
        TAG,
        "Zero reference = %.2f deg",
        *zero_reference);

    ESP_LOGI(
        TAG,
        "YOU MAY NOW MOVE YOUR LEG");

    ESP_LOGI(TAG,"======================================");

    return true;
}

/* ===================== SAFE MOTOR COMMAND ===================== */

static float update_command_angle(
    float measured_angle){

    static bool tracking=false;
    static float command_angle=0.0f;

    if(!tracking){

        if(measured_angle>=
           MOVE_START_ANGLE_DEG){

            tracking=true;
        }
    }
    else{

        if(measured_angle<=
           MOVE_STOP_ANGLE_DEG){

            tracking=false;
        }
    }

    if(!tracking){

        command_angle=0.0f;
        return command_angle;
    }

    float desired=
        clamp_float(
            measured_angle,
            MIN_KNEE_ANGLE_DEG,
            MAX_KNEE_ANGLE_DEG);

    if(fabsf(
        desired-
        command_angle)>=
        COMMAND_DEADBAND_DEG){

        command_angle=
            desired;
    }

    if(desired>=
       MAX_KNEE_ANGLE_DEG){

        command_angle=
            MAX_KNEE_ANGLE_DEG;
    }

    return command_angle;
}

/* ============================================================
   FOLLOWER BLUETOOTH FEEDBACK PARSER

   Expected follower packet:

   M,37.52\n
   ============================================================ */

static void follower_parser_reset(void){
    follower_rx_position=0;
    follower_rx_discarding=false;
}

static void process_follower_data(
    const uint8_t *data,
    int length){

    for(int i=0;i<length;i++){

        char c=(char)data[i];

        if(c=='\n'){

            if(!follower_rx_discarding&&
               follower_rx_position>0){

                follower_rx_buffer[
                    follower_rx_position]='\0';

                /*
                 * Packet must start with:
                 * M,
                 */
                if(follower_rx_buffer[0]=='M'&&
                   follower_rx_buffer[1]==','){

                    char *end;

                    float angle=
                        strtof(
                            &follower_rx_buffer[2],
                            &end);

                    while(*end==' '||
                          *end=='\t'){

                        end++;
                    }

                    if(end!=
                           &follower_rx_buffer[2]&&
                       *end=='\0'&&
                       isfinite(angle)&&
                       angle>=0.0f&&
                       angle<=180.0f){

                        follower_motor_angle=
                            angle;

                        follower_angle_valid=
                            true;

                        follower_last_rx_us=
                            esp_timer_get_time();
                    }
                }
            }

            follower_parser_reset();
        }
        else if(c=='\r'){
            /* Ignore carriage return */
        }
        else if(follower_rx_discarding){
            /* Ignore until newline */
        }
        else if(
            follower_rx_position<
            (int)
            sizeof(follower_rx_buffer)-1){

            follower_rx_buffer[
                follower_rx_position++]=
                c;
        }
        else{
            follower_rx_discarding=true;
        }
    }
}

/* ===================== BLUETOOTH CALLBACK ===================== */

static void spp_callback(
    esp_spp_cb_event_t event,
    esp_spp_cb_param_t *param){

    switch(event){

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
            bt_handle=
                param->srv_open.handle;

            bt_connected=true;
            bt_tx_ready=true;
            bt_congested=false;

            follower_angle_valid=false;
            follower_last_rx_us=0;

            follower_parser_reset();

            ESP_LOGI(
                TAG,
                "FOLLOWER CONNECTED");

            break;
        }

        /*
         * NEW:
         * Receive measured motor angle from follower.
         */
        case ESP_SPP_DATA_IND_EVT:
        {
            process_follower_data(
                param->data_ind.data,
                param->data_ind.len);

            break;
        }

        case ESP_SPP_CLOSE_EVT:
        {
            bt_connected=false;
            bt_tx_ready=true;
            bt_congested=false;

            follower_angle_valid=false;
            follower_last_rx_us=0;

            follower_parser_reset();

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
            bt_congested=
                param->cong.cong;

            break;
        }

        case ESP_SPP_WRITE_EVT:
        {
            bt_congested=
                param->write.cong;

            bt_tx_ready=true;

            break;
        }

        default:
        {
            break;
        }
    }
}

/* ===================== SEND COMMAND TO FOLLOWER ===================== */

static void bluetooth_send_angle(float angle){

    if(!bt_connected||
       !bt_tx_ready||
       bt_congested){

        return;
    }

    char message[24];

    int length=
        snprintf(
            message,
            sizeof(message),
            "%.2f\n",
            angle);

    if(length<=0)
        return;

    bt_tx_ready=false;

    esp_err_t err=
        esp_spp_write(
            bt_handle,
            length,
            (uint8_t *)message);

    if(err!=ESP_OK)
        bt_tx_ready=true;
}

/* ===================== BLUETOOTH INIT ===================== */

static void bluetooth_init(void){

    esp_bt_controller_config_t bt_cfg=
        BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(
        esp_bt_controller_init(
            &bt_cfg));

    ESP_ERROR_CHECK(
        esp_bt_controller_enable(
            ESP_BT_MODE_CLASSIC_BT));

    ESP_ERROR_CHECK(
        esp_bluedroid_init());

    ESP_ERROR_CHECK(
        esp_bluedroid_enable());

    ESP_ERROR_CHECK(
        esp_bt_gap_set_device_name(
            BT_DEVICE_NAME));

    ESP_ERROR_CHECK(
        esp_spp_register_callback(
            spp_callback));

    esp_spp_cfg_t spp_cfg={
        .mode=ESP_SPP_MODE_CB,
        .enable_l2cap_ertm=true,
        .tx_buffer_size=0
    };

    ESP_ERROR_CHECK(
        esp_spp_enhanced_init(
            &spp_cfg));
}

/* ===================== WEBSOCKET HANDLER ===================== */

static esp_err_t ws_handler(
    httpd_req_t *req){

    if(req->method==HTTP_GET){

        ws_client_fd=
            httpd_req_to_sockfd(
                req);

        ws_connected=true;

        ESP_LOGI(
            TAG,
            "Laptop app connected over WiFi (fd=%d)",
            ws_client_fd);

        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;

    memset(
        &ws_pkt,
        0,
        sizeof(ws_pkt));

    ws_pkt.type=
        HTTPD_WS_TYPE_TEXT;

    esp_err_t ret=
        httpd_ws_recv_frame(
            req,
            &ws_pkt,
            0);

    if(ret!=ESP_OK){

        ESP_LOGW(
            TAG,
            "httpd_ws_recv_frame failed: %d",
            ret);

        ws_connected=false;

        return ret;
    }

    if(ws_pkt.len>0){

        uint8_t *buf=
            calloc(
                1,
                ws_pkt.len+1);

        if(buf==NULL)
            return ESP_ERR_NO_MEM;

        ws_pkt.payload=buf;

        ret=
            httpd_ws_recv_frame(
                req,
                &ws_pkt,
                ws_pkt.len);

        free(buf);

        if(ret!=ESP_OK){

            ws_connected=false;
            return ret;
        }
    }

    if(ws_pkt.type==
       HTTPD_WS_TYPE_CLOSE){

        ESP_LOGW(
            TAG,
            "Laptop app closed the WebSocket");

        ws_connected=false;
    }

    return ESP_OK;
}

static const httpd_uri_t ws_uri={
    .uri=WS_URI,
    .method=HTTP_GET,
    .handler=ws_handler,
    .user_ctx=NULL,
    .is_websocket=true
};

/* ===================== WEBSOCKET SERVER ===================== */

static void start_ws_server(void){

    httpd_config_t config=
        HTTPD_DEFAULT_CONFIG();

    config.lru_purge_enable=true;

    if(httpd_start(
        &ws_server,
        &config)==ESP_OK){

        httpd_register_uri_handler(
            ws_server,
            &ws_uri);

        ESP_LOGI(
            TAG,
            "WebSocket server ready at ws://192.168.4.1%s",
            WS_URI);
    }
    else{
        ESP_LOGE(
            TAG,
            "Failed to start WebSocket server");
    }
}

/* ===================== WIFI ===================== */

static void wifi_init_softap(void){

    ESP_ERROR_CHECK(
        esp_netif_init());

    ESP_ERROR_CHECK(
        esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg=
        WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(
        esp_wifi_init(
            &cfg));

    wifi_config_t wifi_config={
        .ap={
            .ssid=WIFI_AP_SSID,
            .ssid_len=strlen(WIFI_AP_SSID),
            .channel=WIFI_AP_CHANNEL,
            .max_connection=WIFI_AP_MAX_CONN,
            .authmode=WIFI_AUTH_WPA2_PSK,
            .pmf_cfg={
                .required=false
            }
        }
    };

    strncpy(
        (char *)wifi_config.ap.password,
        WIFI_AP_PASS,
        sizeof(wifi_config.ap.password)-1);

    if(strlen(WIFI_AP_PASS)==0)
        wifi_config.ap.authmode=WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(
        esp_wifi_set_mode(
            WIFI_MODE_AP));

    ESP_ERROR_CHECK(
        esp_wifi_set_config(
            WIFI_IF_AP,
            &wifi_config));

    ESP_ERROR_CHECK(
        esp_wifi_start());

    ESP_LOGI(TAG,"");
    ESP_LOGI(TAG,"======================================");
    ESP_LOGI(TAG,"WIFI ACCESS POINT STARTED");
    ESP_LOGI(TAG,"SSID: %s",WIFI_AP_SSID);

    ESP_LOGI(
        TAG,
        "Password: %s",
        strlen(WIFI_AP_PASS)?
        WIFI_AP_PASS:
        "(open network)");

    ESP_LOGI(
        TAG,
        "WebSocket: ws://192.168.4.1%s",
        WS_URI);

    ESP_LOGI(TAG,"======================================");
}

/* ============================================================
   SEND BOTH ANGLES TO LAPTOP

   Valid follower feedback:
   L,37.52,M,35.91

   No valid follower feedback:
   L,37.52,M,nan
   ============================================================ */

static void ws_send_angles(
    float leader_angle,
    float motor_angle,
    bool motor_valid){

    if(!ws_connected||
       ws_server==NULL||
       ws_client_fd<0){

        return;
    }

    char message[64];
    int length;

    if(motor_valid){

        length=
            snprintf(
                message,
                sizeof(message),
                "L,%.2f,M,%.2f",
                leader_angle,
                motor_angle);
    }
    else{

        length=
            snprintf(
                message,
                sizeof(message),
                "L,%.2f,M,nan",
                leader_angle);
    }

    if(length<=0||
       length>=(int)sizeof(message)){

        return;
    }

    httpd_ws_frame_t ws_pkt;

    memset(
        &ws_pkt,
        0,
        sizeof(ws_pkt));

    ws_pkt.type=
        HTTPD_WS_TYPE_TEXT;

    ws_pkt.payload=
        (uint8_t *)message;

    ws_pkt.len=
        length;

    esp_err_t err=
        httpd_ws_send_frame_async(
            ws_server,
            ws_client_fd,
            &ws_pkt);

    if(err!=ESP_OK)
        ws_connected=false;
}

/* ===================== MAIN ===================== */

void app_main(void){

    ESP_LOGI(TAG,"");
    ESP_LOGI(TAG,"======================================");
    ESP_LOGI(TAG,"LEADER KNEE ANGLE SYSTEM");
    ESP_LOGI(TAG,"Dual MPU6050 - Z-axis knee flexion only");

    ESP_LOGI(
        TAG,
        "Knee range = %.1f to %.1f deg",
        MIN_KNEE_ANGLE_DEG,
        MAX_KNEE_ANGLE_DEG);

    ESP_LOGI(
        TAG,
        "Follower feedback -> Leader -> Laptop enabled");

    ESP_LOGI(TAG,"======================================");

    /* ===================== NVS ===================== */

    esp_err_t nvs_ret=
        nvs_flash_init();

    if(nvs_ret==
           ESP_ERR_NVS_NO_FREE_PAGES||
       nvs_ret==
           ESP_ERR_NVS_NEW_VERSION_FOUND){

        ESP_ERROR_CHECK(
            nvs_flash_erase());

        ESP_ERROR_CHECK(
            nvs_flash_init());
    }
    else{
        ESP_ERROR_CHECK(
            nvs_ret);
    }

    /* ===================== IMUs ===================== */

    imu_t thigh={0};
    imu_t shin={0};

    ESP_ERROR_CHECK(
        add_i2c_bus_and_device(
            THIGH_I2C_PORT,
            THIGH_SDA,
            THIGH_SCL,
            THIGH_ADDR,
            &thigh.bus,
            &thigh.dev));

    ESP_ERROR_CHECK(
        add_i2c_bus_and_device(
            SHIN_I2C_PORT,
            SHIN_SDA,
            SHIN_SCL,
            SHIN_ADDR,
            &shin.bus,
            &shin.dev));

    if(!mpu_init(
        thigh.dev,
        "THIGH IMU")){

        return;
    }

    if(!mpu_init(
        shin.dev,
        "SHIN IMU")){

        return;
    }

    /* ===================== CALIBRATION ===================== */

    if(!calibrate_gyros(
        &thigh,
        &shin)){

        return;
    }

    if(!settle_filters(
        &thigh,
        &shin)){

        ESP_LOGE(
            TAG,
            "Filter settling failed");

        return;
    }

    float zero_reference=0.0f;

    if(!calibrate_knee_zero(
        &thigh,
        &shin,
        &zero_reference)){

        return;
    }

    /* ===================== COMMUNICATION ===================== */

    bluetooth_init();

    wifi_init_softap();

    start_ws_server();

    /* ===================== MAIN LOOP ===================== */

    int64_t previous_time=
        esp_timer_get_time();

    uint32_t sample_count=0;

    float display_angle=0.0f;
    bool display_init=false;

    while(1){

        vTaskDelay(
            pdMS_TO_TICKS(
                SAMPLE_PERIOD_MS));

        int64_t now=
            esp_timer_get_time();

        float dt=
            (now-
             previous_time)/
            1000000.0f;

        previous_time=now;

        if(dt<=0.0f||
           dt>0.1f){

            dt=0.01f;
        }

        /* ===================== UPDATE IMUs ===================== */

        if(!update_imu(
            &thigh,
            dt)){

            ESP_LOGE(
                TAG,
                "THIGH IMU read failed");

            continue;
        }

        if(!update_imu(
            &shin,
            dt)){

            ESP_LOGE(
                TAG,
                "SHIN IMU read failed");

            continue;
        }

        /* ===================== KNEE ANGLE ===================== */

        float thigh_angle=
            get_thigh_angle(
                &thigh);

        float shin_angle=
            get_shin_angle(
                &shin);

        float relative_angle=
            wrap_angle_deg(
                shin_angle-
                thigh_angle);

        float raw_knee_angle=
            KNEE_DIRECTION*
            wrap_angle_deg(
                relative_angle-
                zero_reference);

        float knee_angle=
            raw_knee_angle;

        if(knee_angle<
           KNEE_ZERO_DEADBAND_DEG){

            knee_angle=0.0f;
        }

        knee_angle=
            clamp_float(
                knee_angle,
                MIN_KNEE_ANGLE_DEG,
                MAX_KNEE_ANGLE_DEG);

        /* ===================== DISPLAY FILTER ===================== */

        if(!display_init){

            display_angle=
                knee_angle;

            display_init=true;
        }
        else{

            display_angle+=
                DISPLAY_LPF_ALPHA*
                (knee_angle-
                 display_angle);
        }

        display_angle=
            clamp_float(
                display_angle,
                MIN_KNEE_ANGLE_DEG,
                MAX_KNEE_ANGLE_DEG);

        /* ===================== MOTOR COMMAND ===================== */

        float command_angle=
            update_command_angle(
                knee_angle);

        /* ===================== FOLLOWER FEEDBACK STATUS ===================== */

        bool follower_fresh=false;
        float motor_angle=
            follower_motor_angle;

        if(bt_connected&&
           follower_angle_valid&&
           follower_last_rx_us!=0){

            int64_t feedback_age_us=
                now-
                follower_last_rx_us;

            if(feedback_age_us<
               (int64_t)
               FOLLOWER_FEEDBACK_TIMEOUT_MS*
               1000){

                follower_fresh=true;
            }
        }

        sample_count++;

        /* ===================== TERMINAL ===================== */

        if((sample_count%
            TERMINAL_PRINT_DIV)==0){

            if(follower_fresh){

                printf(
                    "KNEE=%6.2f | "
                    "RAW=%+7.2f | "
                    "CMD=%6.2f | "
                    "MOTOR=%6.2f | "
                    "THIGH_Z=%+7.2f | "
                    "SHIN_Z=%+7.2f | "
                    "BT=%s | "
                    "FB=OK | "
                    "APP=%s\n",

                    knee_angle,
                    raw_knee_angle,
                    command_angle,
                    motor_angle,
                    thigh_angle,
                    shin_angle,

                    bt_connected?
                    "CONNECTED":
                    "WAITING",

                    ws_connected?
                    "CONNECTED":
                    "WAITING");
            }
            else{

                printf(
                    "KNEE=%6.2f | "
                    "RAW=%+7.2f | "
                    "CMD=%6.2f | "
                    "MOTOR=--- | "
                    "THIGH_Z=%+7.2f | "
                    "SHIN_Z=%+7.2f | "
                    "BT=%s | "
                    "FB=WAITING | "
                    "APP=%s\n",

                    knee_angle,
                    raw_knee_angle,
                    command_angle,
                    thigh_angle,
                    shin_angle,

                    bt_connected?
                    "CONNECTED":
                    "WAITING",

                    ws_connected?
                    "CONNECTED":
                    "WAITING");
            }
        }

        /* ===================== SEND TO FOLLOWER ===================== */

        if((sample_count%
            BT_SEND_DIV)==0){

            bluetooth_send_angle(
                command_angle);
        }

        /* ===================== SEND TO LAPTOP ===================== */

        if((sample_count%
            WS_SEND_DIV)==0){

            ws_send_angles(
                display_angle,
                motor_angle,
                follower_fresh);
        }
    }
}