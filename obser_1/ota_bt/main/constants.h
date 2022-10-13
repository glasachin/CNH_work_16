static const char *TAG = "soil_sensor";

// ------Server Certificates----------
extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

#define OTA_URL_SIZE 256

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

char bt_data[10] = "";
int ii;
char value;