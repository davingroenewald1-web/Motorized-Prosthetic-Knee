#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_spp_api.h"


static const char *TAG = "KNEE_FOLLOWER";

/* Leader Bluetooth MAC address */
static esp_bd_addr_t leader_address =
{
    0x68, 0xFE, 0x71,
    0x90, 0x74, 0xE6
};


/* ============================================================
   RECEIVE ANGLE
   ============================================================ */

static void process_data(
        const uint8_t *data,
        int length)
{
    static char buffer[32];
    static int pos = 0;

    for (int i = 0; i < length; i++)
    {
        char c = (char)data[i];

        if (c == '\n')
        {
            buffer[pos] = '\0';

            float angle =
                    strtof(buffer, NULL);

            ESP_LOGI(
                    TAG,
                    "RECEIVED ANGLE = %.2f deg",
                    angle);

            pos = 0;
        }
        else if (c != '\r')
        {
            if (pos < sizeof(buffer) - 1)
            {
                buffer[pos++] = c;
            }
            else
            {
                pos = 0;
            }
        }
    }
}


/* ============================================================
   BLUETOOTH SPP CALLBACK
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
                    "Bluetooth ready");

            ESP_LOGI(
                    TAG,
                    "Looking for leader SPP service...");

            esp_spp_start_discovery(
                    leader_address);

            break;
        }


        case ESP_SPP_DISCOVERY_COMP_EVT:
        {
            if (param->disc_comp.status ==
                    ESP_SPP_SUCCESS &&
                param->disc_comp.scn_num > 0)
            {
                ESP_LOGI(
                        TAG,
                        "Leader found - connecting");

                esp_spp_connect(
                        ESP_SPP_SEC_NONE,
                        ESP_SPP_ROLE_MASTER,
                        param->disc_comp.scn[0],
                        leader_address);
            }
            else
            {
                ESP_LOGE(
                        TAG,
                        "Leader SPP service not found");
            }

            break;
        }


        case ESP_SPP_OPEN_EVT:
        {
            if (param->open.status ==
                ESP_SPP_SUCCESS)
            {
                ESP_LOGI(
                        TAG,
                        "CONNECTED TO KNEE_LEADER");
            }

            break;
        }


        case ESP_SPP_DATA_IND_EVT:
        {
            process_data(
                    param->data_ind.data,
                    param->data_ind.len);

            break;
        }


        case ESP_SPP_CLOSE_EVT:
        {
            ESP_LOGW(
                    TAG,
                    "Leader disconnected");

            break;
        }


        default:
            break;
    }
}


/* ============================================================
   BLUETOOTH INITIALISATION
   ============================================================ */

static void bluetooth_init(void)
{
    esp_err_t ret =
            nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(
                nvs_flash_erase());

        ESP_ERROR_CHECK(
                nvs_flash_init());
    }


    esp_bt_controller_config_t config =
            BT_CONTROLLER_INIT_CONFIG_DEFAULT();


    ESP_ERROR_CHECK(
            esp_bt_controller_init(
                    &config));


    ESP_ERROR_CHECK(
            esp_bt_controller_enable(
                    ESP_BT_MODE_CLASSIC_BT));


    ESP_ERROR_CHECK(
            esp_bluedroid_init());


    ESP_ERROR_CHECK(
            esp_bluedroid_enable());


    ESP_ERROR_CHECK(
            esp_spp_register_callback(
                    spp_callback));


    esp_spp_cfg_t spp_config =
    {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0
    };


    ESP_ERROR_CHECK(
            esp_spp_enhanced_init(
                    &spp_config));
}


/* ============================================================
   MAIN
   ============================================================ */

void app_main(void)
{
    ESP_LOGI(
            TAG,
            "KNEE FOLLOWER STARTING");

    bluetooth_init();
}