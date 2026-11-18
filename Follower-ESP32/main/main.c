/*
 * KNEE FOLLOWER
 * RMD X6-40 over RS485
 * Leader knee command over Classic Bluetooth SPP
 *
 * Signal path:
 * Leader -> Bluetooth -> Follower -> RS485 -> RMD motor
 *
 * Feedback path:
 * RMD measured angle -> Follower -> Bluetooth -> Leader
 *
 * Follower Bluetooth feedback format:
 * M,37.52\n
 *
 * No WiFi is used on the follower.
 *
 * Power up with the motor/knee at full extension.
 * This position becomes software zero.
 */

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_spp_api.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#if CONFIG_FREERTOS_HZ < 1000
#warning "Set CONFIG_FREERTOS_HZ=1000"
#endif

static const char *TAG="KNEE_FOLLOWER";

#define PI_F 3.14159265358979323846f

/* ===================== LEADER / BLUETOOTH ===================== */

static esp_bd_addr_t leader_address={
    0x68,0xFE,0x71,0x90,0x78,0x4A
};

#define LEADER_TIMEOUT_MS 500
#define LEADER_VALID_MIN_DEG 0.0f
#define LEADER_VALID_MAX_DEG 120.0f
#define BT_RECONNECT_DELAY_MS 1000
#define BT_SCN_FAIL_LIMIT 3

/* Send measured motor angle back to leader at 20 Hz */
#define BT_FEEDBACK_PERIOD_MS 50

/* ===================== JOINT MAPPING ===================== */

#define KNEE_MIN_DEG 0.0f
#define KNEE_MAX_DEG 180.0f
#define FOLLOWER_GAIN 1.0f
#define FOLLOWER_DIRECTION -1.0f

#define MAP_TO_MOTOR(k) ((k)*FOLLOWER_GAIN*FOLLOWER_DIRECTION)

/*
 * Hard cap on the knee command actually driven to the motor.
 */
#define KNEE_COMMAND_MAX_DEG 100.0f

/* ===================== EXTENSION SOFT STOP ===================== */

/*
 * Normal operation will not actively extend the prosthetic
 * knee past 3 degrees.
 *
 * The actual encoder/software zero remains at physical
 * full extension = 0 degrees.
 */
#define SAFE_EXTENSION_DEG 3.0f

/*
 * Start slowing extension when the measured knee angle
 * reaches 10 degrees.
 */
#define EXTENSION_SLOW_ZONE_DEG 10.0f

/*
 * Maximum extension velocity inside the slow zone.
 */
#define EXTENSION_MAX_SPEED_DPS 20.0f

/*
 * Prevent hunting around the 3 degree soft stop.
 */
#define EXTENSION_HOLD_DEADBAND_DEG 0.5f

/* ===================== REFERENCE GENERATOR ===================== */

#define REF_BANDWIDTH_HZ 2.5f
#define REF_ZETA 1.0f
#define REF_VEL_MAX_DPS 90.0f
#define REF_ACC_MAX_DPS2 600.0f

/* ===================== POSITION CONTROLLER ===================== */

#define POS_KP 6.0f
#define MOTOR_MAX_SPEED_DPS 120.0f

/* ===================== TIMING ===================== */

#define CONTROL_PERIOD_MS 20
#define STATUS_EVERY_N_CYCLES 25
#define LOG_PERIOD_MS 200

/* ===================== PROTECTION ===================== */

#define LIMIT_FAULT_MARGIN_DEG 5.0f
#define TRACK_ERR_FAULT_DEG 15.0f
#define TRACK_ERR_FAULT_MS 500
#define TRACK_FAULT_MAX_REF_VEL_DPS 20.0f
#define IQ_FAULT_A 10.0f
#define TEMP_FAULT_C 70
#define COMM_FAIL_FAULT_COUNT 5

/* ===================== RMD RS485 ===================== */

#define RMD_UART UART_NUM_2
#define RMD_TX_PIN GPIO_NUM_17
#define RMD_RX_PIN GPIO_NUM_16
#define RMD_DIR_PIN GPIO_NUM_4
#define RMD_BAUD 115200
#define RMD_ID 0x01
#define RMD_FRAME_SIZE 13
#define RMD_RX_TIMEOUT_US 4000
#define RS485_HEARS_OWN_TX 0

#define CMD_READ_ACCEL 0x42
#define CMD_STOP 0x81
#define CMD_READ_ANGLE 0x92
#define CMD_READ_STATUS1 0x9A
#define CMD_SPEED 0xA2

#define BT_LED_PIN GPIO_NUM_25

/* ===================== TYPES ===================== */

typedef struct{
    float angle_deg;
    int64_t t_us;
}leader_sample_t;

typedef struct{
    int8_t temp_c;
    float iq_a;
    float speed_dps;
}rmd_feedback_t;

typedef enum{
    ST_INIT,
    ST_IDLE,
    ST_RUN,
    ST_STOPPING,
    ST_FAULT
}state_t;

static const char *state_names[]={
    "INIT",
    "IDLE",
    "RUN",
    "STOPPING",
    "FAULT"
};

typedef enum{
    FLT_NONE,
    FLT_COMM,
    FLT_DRIVE_ERROR,
    FLT_OVERTEMP,
    FLT_OVERCURRENT,
    FLT_TRACKING,
    FLT_LIMIT
}fault_t;

static const char *fault_names[]={
    "none",
    "RS485 comms lost",
    "drive error flag",
    "over-temperature",
    "over-current",
    "tracking error too large",
    "joint limit exceeded"
};

typedef struct{
    state_t state;
    fault_t fault;
    uint16_t drive_error;
    float leader_deg;
    float ref_deg;
    float ref_vel;
    float meas_deg;
    float cmd_dps;
    float iq_a;
    float voltage_v;
    int8_t temp_c;
    uint32_t rmd_ok;
    uint32_t rmd_retry;
    uint32_t rmd_fail;
    uint32_t overruns;
    int64_t loop_us_max;
}snapshot_t;

/* ===================== SHARED STATE ===================== */

static snapshot_t snap;

static portMUX_TYPE snap_mux=portMUX_INITIALIZER_UNLOCKED;

static QueueHandle_t leader_mb;
static QueueHandle_t motor_feedback_mb;

static volatile bool bt_connected=false;
static volatile bool bt_tx_ready=true;
static volatile bool bt_congested=false;

static uint32_t bt_handle=0;

static volatile uint32_t parse_errors=0;
static volatile uint32_t bt_rx_count=0;
static volatile uint32_t bt_tx_count=0;
static volatile uint32_t bt_tx_skipped=0;
static volatile uint32_t bt_disconnects=0;

static volatile int64_t bt_last_rx_us=0;
static volatile int64_t bt_max_gap_us=0;

static esp_timer_handle_t reconnect_timer;

static uint8_t leader_scn=0;
static bool have_scn=false;
static int scn_fail_count=0;

static uint32_t rmd_ok=0;
static uint32_t rmd_retry=0;
static uint32_t rmd_fail=0;

static float motor_min_deg;
static float motor_max_deg;

/* ===================== HELPERS ===================== */

static float clampf(float v,float lo,float hi){
    return v<lo?lo:(v>hi?hi:v);
}

static void set_fault(fault_t *fault,fault_t reason){
    if(*fault==FLT_NONE)
        *fault=reason;
}

/* ===================== RMD CRC ===================== */

static uint16_t crc16_modbus(const uint8_t *data,size_t length){
    uint16_t crc=0xFFFF;

    for(size_t i=0;i<length;i++){
        crc^=data[i];

        for(int bit=0;bit<8;bit++){
            crc=(crc&1)?
                (crc>>1)^0xA001:
                (crc>>1);
        }
    }

    return crc;
}

static void rmd_build_frame(
    const uint8_t data[8],
    uint8_t frame[13]){

    frame[0]=0x3E;
    frame[1]=RMD_ID;
    frame[2]=0x08;

    memcpy(&frame[3],data,8);

    uint16_t crc=
        crc16_modbus(
            frame,
            11);

    frame[11]=(uint8_t)(crc&0xFF);
    frame[12]=(uint8_t)(crc>>8);
}

static bool rmd_valid(
    const uint8_t frame[13]){

    if(frame[0]!=0x3E||
       frame[1]!=RMD_ID||
       frame[2]!=0x08){

        return false;
    }

    uint16_t rx=
        (uint16_t)frame[11]|
        ((uint16_t)frame[12]<<8);

    return rx==
        crc16_modbus(
            frame,
            11);
}

/* ===================== RMD TRANSACTION ===================== */

static bool rmd_transaction(
    const uint8_t data[8],
    uint8_t response[13]){

    uint8_t tx[RMD_FRAME_SIZE];

    rmd_build_frame(
        data,
        tx);

    for(int attempt=0;
        attempt<2;
        attempt++){

        if(attempt>0)
            rmd_retry++;

        uart_flush_input(
            RMD_UART);

        if(uart_write_bytes(
            RMD_UART,
            (const char *)tx,
            RMD_FRAME_SIZE)!=RMD_FRAME_SIZE){

            continue;
        }

        if(uart_wait_tx_done(
            RMD_UART,
            pdMS_TO_TICKS(5))!=ESP_OK){

            continue;
        }

        int skip=
            RS485_HEARS_OWN_TX?
            RMD_FRAME_SIZE:
            0;

        int pos=0;

        int64_t deadline=
            esp_timer_get_time()+
            RMD_RX_TIMEOUT_US;

        while(esp_timer_get_time()<
              deadline){

            uint8_t buffer[RMD_FRAME_SIZE];

            int count=
                uart_read_bytes(
                    RMD_UART,
                    buffer,
                    sizeof(buffer),
                    pdMS_TO_TICKS(1));

            for(int i=0;
                i<count;
                i++){

                if(skip>0){
                    skip--;
                    continue;
                }

                if(pos==0&&
                   buffer[i]!=0x3E){

                    continue;
                }

                response[pos++]=
                    buffer[i];

                if(pos==RMD_FRAME_SIZE){

                    if(rmd_valid(response)&&
                       response[3]==data[0]){

                        rmd_ok++;

                        return true;
                    }

                    pos=0;
                }
            }
        }
    }

    rmd_fail++;

    return false;
}

/* ===================== RMD COMMANDS ===================== */

static bool rmd_simple(
    uint8_t command_byte){

    uint8_t command[8]={
        command_byte
    };

    uint8_t response[13];

    return rmd_transaction(
        command,
        response);
}

static bool rmd_read_planning_value(
    uint8_t index,
    uint32_t *value){

    uint8_t command[8]={
        CMD_READ_ACCEL,
        index
    };

    uint8_t response[13];

    if(!rmd_transaction(
            command,
            response)||
       response[4]!=index){

        return false;
    }

    *value=
        (uint32_t)response[7]|
        ((uint32_t)response[8]<<8)|
        ((uint32_t)response[9]<<16)|
        ((uint32_t)response[10]<<24);

    return true;
}

static bool rmd_read_angle(
    float *angle_deg){

    uint8_t command[8]={
        CMD_READ_ANGLE
    };

    uint8_t response[13];

    if(!rmd_transaction(
        command,
        response)){

        return false;
    }

    int32_t raw=
        (int32_t)(
            (uint32_t)response[7]|
            ((uint32_t)response[8]<<8)|
            ((uint32_t)response[9]<<16)|
            ((uint32_t)response[10]<<24));

    *angle_deg=
        raw*0.01f;

    return true;
}

static bool rmd_read_status1(
    int8_t *temperature,
    float *voltage,
    uint16_t *error){

    uint8_t command[8]={
        CMD_READ_STATUS1
    };

    uint8_t response[13];

    if(!rmd_transaction(
        command,
        response)){

        return false;
    }

    *temperature=
        (int8_t)response[4];

    *voltage=
        ((uint16_t)response[7]|
        ((uint16_t)response[8]<<8))*
        0.1f;

    *error=
        (uint16_t)response[9]|
        ((uint16_t)response[10]<<8);

    return true;
}

static bool rmd_set_speed(
    float dps,
    rmd_feedback_t *feedback){

    uint32_t speed=
        (uint32_t)(int32_t)
        lroundf(
            dps*100.0f);

    uint8_t command[8]={
        CMD_SPEED,
        0x00,
        0x00,
        0x00,
        (uint8_t)speed,
        (uint8_t)(speed>>8),
        (uint8_t)(speed>>16),
        (uint8_t)(speed>>24)
    };

    uint8_t response[13];

    if(!rmd_transaction(
        command,
        response)){

        return false;
    }

    feedback->temp_c=
        (int8_t)response[4];

    feedback->iq_a=
        (int16_t)(
            (uint16_t)response[5]|
            ((uint16_t)response[6]<<8))*
        0.01f;

    feedback->speed_dps=
        (int16_t)(
            (uint16_t)response[7]|
            ((uint16_t)response[8]<<8));

    return true;
}

/* ===================== REFERENCE GENERATOR ===================== */

static float ref_pos=0.0f;
static float ref_vel=0.0f;

static void ref_limit(void){

    if(ref_pos>motor_max_deg){

        ref_pos=
            motor_max_deg;

        if(ref_vel>0.0f)
            ref_vel=0.0f;
    }
    else if(ref_pos<motor_min_deg){

        ref_pos=
            motor_min_deg;

        if(ref_vel<0.0f)
            ref_vel=0.0f;
    }
}

static void ref_track(
    float target,
    float dt){

    const float wn=
        2.0f*
        PI_F*
        REF_BANDWIDTH_HZ;

    float acc=
        wn*wn*
        (target-ref_pos)-
        2.0f*
        REF_ZETA*
        wn*
        ref_vel;

    acc=
        clampf(
            acc,
            -REF_ACC_MAX_DPS2,
            REF_ACC_MAX_DPS2);

    ref_vel=
        clampf(
            ref_vel+
            acc*dt,
            -REF_VEL_MAX_DPS,
            REF_VEL_MAX_DPS);

    ref_pos+=
        ref_vel*
        dt;

    ref_limit();
}

static void ref_brake(
    float dt){

    float dv=
        REF_ACC_MAX_DPS2*
        dt;

    if(ref_vel>dv){

        ref_vel-=dv;
    }
    else if(ref_vel<-dv){

        ref_vel+=dv;
    }
    else{

        ref_vel=0.0f;
    }

    ref_pos+=
        ref_vel*
        dt;

    ref_limit();
}

/* ===================== BLUETOOTH MOTOR FEEDBACK ===================== */

static void bluetooth_send_motor_angle(
    float knee_angle){

    if(!bt_connected)
        return;

    if(!bt_tx_ready||
       bt_congested){

        bt_tx_skipped++;

        return;
    }

    char message[24];

    int length=
        snprintf(
            message,
            sizeof(message),
            "M,%.2f\n",
            knee_angle);

    if(length<=0||
       length>=(int)sizeof(message)){

        return;
    }

    bt_tx_ready=
        false;

    esp_err_t err=
        esp_spp_write(
            bt_handle,
            length,
            (uint8_t *)message);

    if(err!=ESP_OK){

        bt_tx_ready=
            true;

        bt_tx_skipped++;
    }
    else{

        bt_tx_count++;
    }
}

static void bt_feedback_task(
    void *arg){

    float knee_angle=
        0.0f;

    bool have_angle=
        false;

    TickType_t last_wake=
        xTaskGetTickCount();

    while(1){

        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(
                BT_FEEDBACK_PERIOD_MS));

        float latest;

        if(xQueueReceive(
            motor_feedback_mb,
            &latest,
            0)==pdTRUE){

            knee_angle=
                latest;

            have_angle=
                true;
        }

        if(have_angle){

            bluetooth_send_motor_angle(
                knee_angle);
        }
    }
}

/* ===================== MOTOR CONTROL ===================== */

static void motor_task(
    void *arg){

    const float dt=
        CONTROL_PERIOD_MS/
        1000.0f;

    state_t state=
        ST_INIT;

    fault_t fault=
        FLT_NONE;

    uint16_t drive_error=
        0;

    int8_t temperature=
        0;

    float voltage=
        0.0f;

    rmd_feedback_t feedback={
        0
    };

    float zero_deg=
        0.0f;

    float absolute_deg=
        0.0f;

    float measured_deg=
        0.0f;

    float measured_knee=
        0.0f;

    float leader_deg=
        0.0f;

    float command_dps=
        0.0f;

    bool have_sample=
        false;

    int64_t last_rx_us=
        0;

    int64_t tracking_error_since=
        0;

    int64_t loop_max=
        0;

    int communication_failures=
        0;

    uint32_t cycle=
        0;

    uint32_t overruns=
        0;

    vTaskDelay(
        pdMS_TO_TICKS(1000));

    /* Read drive planning values once at startup */

    for(uint8_t index=0;
        index<4;
        index++){

        uint32_t value;

        if(rmd_read_planning_value(
            index,
            &value)){

            ESP_LOGI(
                TAG,
                "Drive planning accel[%u] = %lu dps/s",
                index,
                (unsigned long)value);

            if(index>=2&&
               value<REF_ACC_MAX_DPS2){

                ESP_LOGW(
                    TAG,
                    "Drive speed accel below REF_ACC_MAX - drive will add lag");
            }
        }

        vTaskDelay(
            pdMS_TO_TICKS(10));
    }

    /* Capture full-extension software zero */

    while(!rmd_read_angle(
        &absolute_deg)){

        ESP_LOGW(
            TAG,
            "Waiting for drive...");

        vTaskDelay(
            pdMS_TO_TICKS(200));
    }

    zero_deg=
        absolute_deg;

    ESP_LOGI(
        TAG,
        "Zero captured at %.2f deg - motor must be at full extension",
        zero_deg);

    state=
        ST_IDLE;

    TickType_t last_wake=
        xTaskGetTickCount();

    while(1){

        int64_t start_us=
            esp_timer_get_time();

        /* Get newest leader sample */

        leader_sample_t sample;

        if(xQueueReceive(
            leader_mb,
            &sample,
            0)==pdTRUE){

            leader_deg=
                sample.angle_deg;

            last_rx_us=
                sample.t_us;

            have_sample=
                true;
        }

        bool leader_fresh=
            have_sample&&
            (start_us-last_rx_us)<
            (int64_t)
            LEADER_TIMEOUT_MS*
            1000;

        /* Motor-angle measurement */

        if(rmd_read_angle(
            &absolute_deg)){

            measured_deg=
                absolute_deg-
                zero_deg;

            communication_failures=
                0;

            /*
             * Convert motor frame back to knee frame.
             *
             * Motor -90 deg
             * becomes
             * Knee +90 deg.
             */

            measured_knee=
                measured_deg/
                (FOLLOWER_GAIN*
                 FOLLOWER_DIRECTION);

            measured_knee=
                clampf(
                    measured_knee,
                    KNEE_MIN_DEG,
                    KNEE_MAX_DEG);

            xQueueOverwrite(
                motor_feedback_mb,
                &measured_knee);
        }
        else{

            communication_failures++;

            if(communication_failures>=
               COMM_FAIL_FAULT_COUNT){

                set_fault(
                    &fault,
                    FLT_COMM);
            }
        }

        /* Drive status */

        if((cycle%
            STATUS_EVERY_N_CYCLES)==0){

            if(rmd_read_status1(
                &temperature,
                &voltage,
                &drive_error)){

                if(drive_error!=0){

                    set_fault(
                        &fault,
                        FLT_DRIVE_ERROR);
                }
                else if(temperature>=
                        TEMP_FAULT_C){

                    set_fault(
                        &fault,
                        FLT_OVERTEMP);
                }
            }
        }

        /* Joint limit protection */

        if(measured_deg>
               motor_max_deg+
               LIMIT_FAULT_MARGIN_DEG||
           measured_deg<
               motor_min_deg-
               LIMIT_FAULT_MARGIN_DEG){

            set_fault(
                &fault,
                FLT_LIMIT);
        }

        /* Latch fault */

        if(fault!=FLT_NONE&&
           state!=ST_FAULT){

            rmd_simple(
                CMD_STOP);

            state=
                ST_FAULT;

            command_dps=
                0.0f;
        }

        switch(state){

            case ST_IDLE:

                if(leader_fresh){

                    ref_pos=
                        measured_deg;

                    ref_vel=
                        0.0f;

                    tracking_error_since=
                        0;

                    state=
                        ST_RUN;
                }

                break;

            case ST_RUN:
            case ST_STOPPING:
            {
                if(state==ST_RUN&&
                   !leader_fresh){

                    state=
                        ST_STOPPING;
                }
                else if(state==ST_STOPPING&&
                        leader_fresh){

                    state=
                        ST_RUN;
                }

                if(state==ST_RUN){

                    float target_knee=
                        clampf(
                            leader_deg,
                            KNEE_MIN_DEG,
                            KNEE_COMMAND_MAX_DEG);

                    /*
                     * EXTENSION SOFT STOP
                     *
                     * If the leader requests less than 3 degrees
                     * while returning from a flexed position,
                     * stop at 3 degrees instead of actively
                     * driving into physical zero.
                     *
                     * If the motor starts below 3 degrees,
                     * do not automatically bend it to 3 degrees.
                     * Hold the position it already has.
                     */

                    if(target_knee<
                       SAFE_EXTENSION_DEG){

                        if(measured_knee>
                           SAFE_EXTENSION_DEG){

                            target_knee=
                                SAFE_EXTENSION_DEG;
                        }
                        else{

                            target_knee=
                                measured_knee;
                        }
                    }

                    float motor_target=
                        MAP_TO_MOTOR(
                            target_knee);

                    ref_track(
                        motor_target,
                        dt);
                }
                else{

                    ref_brake(
                        dt);
                }

                float error=
                    ref_pos-
                    measured_deg;

                command_dps=
                    ref_vel+
                    POS_KP*
                    error;

                command_dps=
                    clampf(
                        command_dps,
                        -MOTOR_MAX_SPEED_DPS,
                        MOTOR_MAX_SPEED_DPS);

                /*
                 * Convert motor velocity back to
                 * knee velocity.
                 *
                 * FOLLOWER_DIRECTION = -1.
                 *
                 * Therefore:
                 *
                 * positive knee speed = flexion
                 * negative knee speed = extension
                 */

                float knee_speed_cmd=
                    command_dps/
                    (FOLLOWER_GAIN*
                     FOLLOWER_DIRECTION);

                /*
                 * SLOW EXTENSION BELOW 10 DEGREES
                 */

                if(measured_knee<=
                       EXTENSION_SLOW_ZONE_DEG&&
                   knee_speed_cmd<0.0f){

                    knee_speed_cmd=
                        fmaxf(
                            knee_speed_cmd,
                            -EXTENSION_MAX_SPEED_DPS);
                }

                /*
                 * ABSOLUTE SOFTWARE EXTENSION STOP
                 *
                 * Once the measured knee reaches
                 * 3 degrees, no command is allowed
                 * to drive it farther toward 0.
                 */

                if(measured_knee<=
                       SAFE_EXTENSION_DEG&&
                   knee_speed_cmd<0.0f){

                    knee_speed_cmd=
                        0.0f;
                }

                /*
                 * HOLD DEADBAND
                 *
                 * Stops the controller from repeatedly
                 * pushing in and out around 3 degrees.
                 */

                if(fabsf(
                       measured_knee-
                       SAFE_EXTENSION_DEG)<=
                       EXTENSION_HOLD_DEADBAND_DEG&&
                   leader_deg<=
                       SAFE_EXTENSION_DEG){

                    knee_speed_cmd=
                        0.0f;
                }

                /*
                 * Convert safe knee velocity back
                 * into motor velocity.
                 */

                command_dps=
                    knee_speed_cmd*
                    FOLLOWER_GAIN*
                    FOLLOWER_DIRECTION;

                /*
                 * Original hard motor limits remain
                 * as the final protection.
                 */

                if(measured_deg>=
                       motor_max_deg&&
                   command_dps>0.0f){

                    command_dps=
                        0.0f;
                }

                if(measured_deg<=
                       motor_min_deg&&
                   command_dps<0.0f){

                    command_dps=
                        0.0f;
                }

                /*
                 * Tracking fault only arms after
                 * the shaped reference has slowed.
                 */

                bool tracking_fault_armed=
                    leader_fresh&&
                    fabsf(ref_vel)<=
                    TRACK_FAULT_MAX_REF_VEL_DPS;

                if(tracking_fault_armed&&
                   fabsf(error)>
                   TRACK_ERR_FAULT_DEG){

                    if(tracking_error_since==0){

                        tracking_error_since=
                            start_us;
                    }
                    else if(
                        start_us-
                        tracking_error_since>
                        (int64_t)
                        TRACK_ERR_FAULT_MS*
                        1000){

                        set_fault(
                            &fault,
                            FLT_TRACKING);
                    }
                }
                else{

                    tracking_error_since=
                        0;
                }

                if(rmd_set_speed(
                    command_dps,
                    &feedback)){

                    if(fabsf(
                        feedback.iq_a)>
                        IQ_FAULT_A){

                        set_fault(
                            &fault,
                            FLT_OVERCURRENT);
                    }
                }

                if(state==ST_STOPPING&&
                   ref_vel==0.0f){

                    rmd_simple(
                        CMD_STOP);

                    command_dps=
                        0.0f;

                    state=
                        ST_IDLE;
                }

                break;
            }

            case ST_FAULT:
            case ST_INIT:
            default:

                break;
        }

        /* Loop timing */

        int64_t loop_us=
            esp_timer_get_time()-
            start_us;

        if(loop_us>
           loop_max){

            loop_max=
                loop_us;
        }

        if(loop_us>
           (int64_t)
           CONTROL_PERIOD_MS*
           1000){

            overruns++;
        }

        /* Logger snapshot */

        taskENTER_CRITICAL(
            &snap_mux);

        snap.state=
            state;

        snap.fault=
            fault;

        snap.drive_error=
            drive_error;

        snap.leader_deg=
            leader_deg;

        snap.ref_deg=
            ref_pos;

        snap.ref_vel=
            ref_vel;

        snap.meas_deg=
            measured_deg;

        snap.cmd_dps=
            command_dps;

        snap.iq_a=
            feedback.iq_a;

        snap.temp_c=
            temperature;

        snap.voltage_v=
            voltage;

        snap.rmd_ok=
            rmd_ok;

        snap.rmd_retry=
            rmd_retry;

        snap.rmd_fail=
            rmd_fail;

        snap.overruns=
            overruns;

        snap.loop_us_max=
            loop_max;

        taskEXIT_CRITICAL(
            &snap_mux);

        cycle++;

        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(
                CONTROL_PERIOD_MS));
    }
}

/* ===================== LOGGER ===================== */

static void log_task(
    void *arg){

    bool blink=
        false;

    while(1){

        snapshot_t current;

        taskENTER_CRITICAL(
            &snap_mux);

        current=
            snap;

        taskEXIT_CRITICAL(
            &snap_mux);

        if(current.state==
           ST_FAULT){

            blink=
                !blink;

            gpio_set_level(
                BT_LED_PIN,
                blink);
        }
        else{

            gpio_set_level(
                BT_LED_PIN,
                bt_connected?
                1:
                0);
        }

        ESP_LOGI(
            TAG,
            "%s | Leader=%.1f Ref=%.1f Meas=%.1f Err=%.2f | Cmd=%.1f dps Iq=%.2f A | %d C %.1f V",
            state_names[
                current.state],
            current.leader_deg,
            current.ref_deg,
            current.meas_deg,
            current.ref_deg-
            current.meas_deg,
            current.cmd_dps,
            current.iq_a,
            current.temp_c,
            current.voltage_v);

        if(current.state==
           ST_FAULT){

            ESP_LOGE(
                TAG,
                "FAULT: %s (drive error 0x%04X) - power cycle to clear",
                fault_names[
                    current.fault],
                current.drive_error);
        }

        ESP_LOGI(
            TAG,
            "RMD ok=%lu retry=%lu fail=%lu | parse=%lu | loop max=%lld us overruns=%lu",
            (unsigned long)
            current.rmd_ok,
            (unsigned long)
            current.rmd_retry,
            (unsigned long)
            current.rmd_fail,
            (unsigned long)
            parse_errors,
            (long long)
            current.loop_us_max,
            (unsigned long)
            current.overruns);

        ESP_LOGI(
            TAG,
            "BT %s | RX=%lu TX=%lu TXbusy=%lu maxgap=%lld ms drops=%lu",
            bt_connected?
                "UP":
                "DOWN",
            (unsigned long)
            bt_rx_count,
            (unsigned long)
            bt_tx_count,
            (unsigned long)
            bt_tx_skipped,
            (long long)
            (bt_max_gap_us/1000),
            (unsigned long)
            bt_disconnects);

        bt_max_gap_us=
            0;

        vTaskDelay(
            pdMS_TO_TICKS(
                LOG_PERIOD_MS));
    }
}

/* ===================== BLUETOOTH RECEIVE PARSER ===================== */

static char parse_buffer[32];
static int parse_position=0;
static bool parse_discarding=false;

static void parser_reset(void){

    parse_position=
        0;

    parse_discarding=
        false;
}

/*
 * Leader -> follower format:
 *
 * 37.52\n
 *
 * Valid range:
 * 0..120 deg
 */

static void process_data(
    const uint8_t *data,
    int length){

    for(int i=0;
        i<length;
        i++){

        char c=
            (char)data[i];

        if(c=='\n'){

            if(!parse_discarding&&
               parse_position>0){

                parse_buffer[
                    parse_position]=
                    '\0';

                char *end;

                float angle=
                    strtof(
                        parse_buffer,
                        &end);

                while(*end==' '||
                      *end=='\t'){

                    end++;
                }

                if(end!=parse_buffer&&
                   *end=='\0'&&
                   isfinite(angle)&&
                   angle>=
                       LEADER_VALID_MIN_DEG&&
                   angle<=
                       LEADER_VALID_MAX_DEG){

                    int64_t now=
                        esp_timer_get_time();

                    if(bt_last_rx_us!=0){

                        int64_t gap=
                            now-
                            bt_last_rx_us;

                        if(gap>
                           bt_max_gap_us){

                            bt_max_gap_us=
                                gap;
                        }
                    }

                    bt_last_rx_us=
                        now;

                    bt_rx_count++;

                    leader_sample_t sample={
                        .angle_deg=angle,
                        .t_us=now
                    };

                    xQueueOverwrite(
                        leader_mb,
                        &sample);
                }
                else{

                    parse_errors++;
                }
            }

            parser_reset();
        }
        else if(c=='\r'||
                parse_discarding){

            /* Ignore */
        }
        else if(
            parse_position<
            (int)
            sizeof(parse_buffer)-1){

            parse_buffer[
                parse_position++]=
                c;
        }
        else{

            parse_discarding=
                true;

            parse_errors++;
        }
    }
}

/* ===================== BLUETOOTH RECONNECT ===================== */

static void reconnect_cb(
    void *arg){

    if(have_scn){

        ESP_LOGI(
            TAG,
            "Reconnecting to leader (cached channel %u)...",
            leader_scn);

        esp_spp_connect(
            ESP_SPP_SEC_NONE,
            ESP_SPP_ROLE_MASTER,
            leader_scn,
            leader_address);
    }
    else{

        ESP_LOGI(
            TAG,
            "Searching for leader...");

        esp_spp_start_discovery(
            leader_address);
    }
}

static void schedule_reconnect(void){

    esp_timer_stop(
        reconnect_timer);

    esp_timer_start_once(
        reconnect_timer,
        (uint64_t)
        BT_RECONNECT_DELAY_MS*
        1000);
}

static void connect_failed(void){

    if(have_scn){

        scn_fail_count++;

        if(scn_fail_count>=
           BT_SCN_FAIL_LIMIT){

            ESP_LOGW(
                TAG,
                "Cached channel failed %d times - rediscovering",
                scn_fail_count);

            have_scn=
                false;

            scn_fail_count=
                0;
        }
    }

    schedule_reconnect();
}

/* ===================== BLUETOOTH CALLBACK ===================== */

static void spp_callback(
    esp_spp_cb_event_t event,
    esp_spp_cb_param_t *param){

    switch(event){

        case ESP_SPP_INIT_EVT:

            ESP_LOGI(
                TAG,
                "Bluetooth ready - searching for leader");

            esp_spp_start_discovery(
                leader_address);

            break;

        case ESP_SPP_DISCOVERY_COMP_EVT:

            if(param->disc_comp.status==
                   ESP_SPP_SUCCESS&&
               param->disc_comp.scn_num>0){

                leader_scn=
                    param->disc_comp.scn[0];

                have_scn=
                    true;

                scn_fail_count=
                    0;

                ESP_LOGI(
                    TAG,
                    "Leader found (channel %u) - connecting",
                    leader_scn);

                esp_spp_connect(
                    ESP_SPP_SEC_NONE,
                    ESP_SPP_ROLE_MASTER,
                    leader_scn,
                    leader_address);
            }
            else{

                ESP_LOGW(
                    TAG,
                    "Leader not found - retrying");

                schedule_reconnect();
            }

            break;

        case ESP_SPP_CL_INIT_EVT:

            if(param->cl_init.status!=
               ESP_SPP_SUCCESS){

                connect_failed();
            }

            break;

        case ESP_SPP_OPEN_EVT:

            if(param->open.status==
               ESP_SPP_SUCCESS){

                bt_handle=
                    param->open.handle;

                bt_connected=
                    true;

                bt_tx_ready=
                    true;

                bt_congested=
                    false;

                bt_last_rx_us=
                    0;

                scn_fail_count=
                    0;

                parser_reset();

                ESP_LOGI(
                    TAG,
                    "CONNECTED TO KNEE_LEADER");
            }
            else{

                connect_failed();
            }

            break;

        case ESP_SPP_DATA_IND_EVT:

            process_data(
                param->data_ind.data,
                param->data_ind.len);

            break;

        case ESP_SPP_CONG_EVT:

            bt_congested=
                param->cong.cong;

            break;

        case ESP_SPP_WRITE_EVT:

            bt_congested=
                param->write.cong;

            bt_tx_ready=
                true;

            if(param->write.status!=
               ESP_SPP_SUCCESS){

                ESP_LOGW(
                    TAG,
                    "Bluetooth feedback write failed: %d",
                    (int)
                    param->write.status);
            }

            break;

        case ESP_SPP_CLOSE_EVT:

            ESP_LOGW(
                TAG,
                "Leader disconnected (status %d) - will reconnect",
                (int)
                param->close.status);

            bt_connected=
                false;

            bt_tx_ready=
                true;

            bt_congested=
                false;

            bt_disconnects++;

            parser_reset();

            schedule_reconnect();

            break;

        default:

            break;
    }
}

/* ===================== BLUETOOTH INITIALISATION ===================== */

static void bluetooth_init(void){

    esp_err_t result=
        nvs_flash_init();

    if(result==
           ESP_ERR_NVS_NO_FREE_PAGES||
       result==
           ESP_ERR_NVS_NEW_VERSION_FOUND){

        ESP_ERROR_CHECK(
            nvs_flash_erase());

        ESP_ERROR_CHECK(
            nvs_flash_init());
    }
    else{

        ESP_ERROR_CHECK(
            result);
    }

    const esp_timer_create_args_t
        reconnect_timer_args={

            .callback=
                &reconnect_cb,

            .name=
                "bt_reconnect"
        };

    ESP_ERROR_CHECK(
        esp_timer_create(
            &reconnect_timer_args,
            &reconnect_timer));

    esp_bt_controller_config_t config=
        BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(
        esp_bt_controller_init(
            &config));

    ESP_ERROR_CHECK(
        esp_bt_controller_enable(
            ESP_BT_MODE_CLASSIC_BT));

    /*
     * Maximum Classic Bluetooth
     * transmit power.
     */

    ESP_ERROR_CHECK(
        esp_bredr_tx_power_set(
            ESP_PWR_LVL_P9,
            ESP_PWR_LVL_P9));

    ESP_ERROR_CHECK(
        esp_bluedroid_init());

    ESP_ERROR_CHECK(
        esp_bluedroid_enable());

    ESP_ERROR_CHECK(
        esp_spp_register_callback(
            spp_callback));

    esp_spp_cfg_t spp_config={

        .mode=
            ESP_SPP_MODE_CB,

        .enable_l2cap_ertm=
            true,

        .tx_buffer_size=
            0
    };

    ESP_ERROR_CHECK(
        esp_spp_enhanced_init(
            &spp_config));
}

/* ===================== STATUS LED ===================== */

static void status_led_init(void){

    gpio_config_t config={

        .pin_bit_mask=
            (1ULL<<BT_LED_PIN),

        .mode=
            GPIO_MODE_OUTPUT,

        .pull_up_en=
            GPIO_PULLUP_DISABLE,

        .pull_down_en=
            GPIO_PULLDOWN_DISABLE,

        .intr_type=
            GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(
        gpio_config(
            &config));

    gpio_set_level(
        BT_LED_PIN,
        0);
}

/* ===================== RS485 INITIALISATION ===================== */

static void rs485_init(void){

    uart_config_t config={

        .baud_rate=
            RMD_BAUD,

        .data_bits=
            UART_DATA_8_BITS,

        .parity=
            UART_PARITY_DISABLE,

        .stop_bits=
            UART_STOP_BITS_1,

        .flow_ctrl=
            UART_HW_FLOWCTRL_DISABLE,

        .source_clk=
            UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(
        uart_driver_install(
            RMD_UART,
            512,
            512,
            0,
            NULL,
            0));

    ESP_ERROR_CHECK(
        uart_param_config(
            RMD_UART,
            &config));

    ESP_ERROR_CHECK(
        uart_set_pin(
            RMD_UART,
            RMD_TX_PIN,
            RMD_RX_PIN,
            RMD_DIR_PIN,
            UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(
        uart_set_mode(
            RMD_UART,
            UART_MODE_RS485_HALF_DUPLEX));

    uart_flush_input(
        RMD_UART);

    ESP_LOGI(
        TAG,
        "RS485 ready");
}

/* ===================== MAIN ===================== */

void app_main(void){

    /*
     * FOLLOWER_DIRECTION = -1:
     *
     * Leader   0 deg -> Motor    0 deg
     * Leader  90 deg -> Motor  -90 deg
     * Leader 180 deg -> Motor -180 deg
     */

    motor_min_deg=
        fminf(
            MAP_TO_MOTOR(
                KNEE_MIN_DEG),
            MAP_TO_MOTOR(
                KNEE_MAX_DEG));

    motor_max_deg=
        fmaxf(
            MAP_TO_MOTOR(
                KNEE_MIN_DEG),
            MAP_TO_MOTOR(
                KNEE_MAX_DEG));

    ESP_LOGI(
        TAG,
        "======================================");

    ESP_LOGI(
        TAG,
        "KNEE FOLLOWER");

    ESP_LOGI(
        TAG,
        "Control loop: %d Hz",
        1000/
        CONTROL_PERIOD_MS);

    ESP_LOGI(
        TAG,
        "BT feedback: %d Hz",
        1000/
        BT_FEEDBACK_PERIOD_MS);

    ESP_LOGI(
        TAG,
        "Leader range: %.0f..%.0f deg",
        KNEE_MIN_DEG,
        KNEE_MAX_DEG);

    ESP_LOGI(
        TAG,
        "Command cap: %.0f deg",
        KNEE_COMMAND_MAX_DEG);

    ESP_LOGI(
        TAG,
        "Extension soft stop: %.1f deg",
        SAFE_EXTENSION_DEG);

    ESP_LOGI(
        TAG,
        "Extension slow zone: %.1f deg",
        EXTENSION_SLOW_ZONE_DEG);

    ESP_LOGI(
        TAG,
        "Extension max speed: %.1f dps",
        EXTENSION_MAX_SPEED_DPS);

    ESP_LOGI(
        TAG,
        "Motor range: %.0f..%.0f deg",
        motor_min_deg,
        motor_max_deg);

    ESP_LOGI(
        TAG,
        "Follower WiFi: DISABLED");

    ESP_LOGI(
        TAG,
        "Feedback packet: M,<angle>");

    ESP_LOGI(
        TAG,
        "======================================");

    leader_mb=
        xQueueCreate(
            1,
            sizeof(
                leader_sample_t));

    motor_feedback_mb=
        xQueueCreate(
            1,
            sizeof(float));

    configASSERT(
        leader_mb);

    configASSERT(
        motor_feedback_mb);

    status_led_init();

    rs485_init();

    bluetooth_init();

    /*
     * Motor control remains on core 1.
     * Bluetooth feedback/logger
     * run at low priority.
     */

    xTaskCreatePinnedToCore(
        motor_task,
        "motor",
        4096,
        NULL,
        10,
        NULL,
        1);

    xTaskCreatePinnedToCore(
        bt_feedback_task,
        "bt_feedback",
        3072,
        NULL,
        2,
        NULL,
        0);

    xTaskCreatePinnedToCore(
        log_task,
        "log",
        4096,
        NULL,
        1,
        NULL,
        0);
}