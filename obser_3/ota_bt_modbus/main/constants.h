#include "esp_system.h"
#include "esp_event.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "driver/uart.h"
#include "protocol_examples_common.h"
#include "esp_wifi.h"
#include <stdio.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "driver/gpio.h"
#include "time.h"
#include "sys/time.h"

static const char *TAG = "soil_sensor";

//-----some Globals---------
esp_err_t err;
char rx_data[10];
char sensor_data[50];// = "";
char ack_data[20];
int ii;
char value;
int bt_connected;
SemaphoreHandle_t print_mux;// = NULL;
esp_spp_cb_param_t *bt_param;
void init_globals(void);

// ------Server Certificates----------
extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

#define OTA_URL_SIZE 256
void ota_wifi_init(void);
void start_ota_wifi(void);

// -----serial communication---------
static const int RX_BUF_SIZE = 1024;
#define TXD_PIN (GPIO_NUM_1) //for UART0
#define RXD_PIN (GPIO_NUM_3) // for UART0


//-----Bluetooth Related----------
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "ESP32_BT"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_DATA

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static struct timeval time_new, time_old;
static long data_num = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

void bt_init(void);
void stop_bt(void);

//------MOD-BUS Functions-------


//---------Main Functions---------
void system_init(void);
void serial_init(void);