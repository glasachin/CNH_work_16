static const char *TAG = "soil_sensor";

// ------Server Certificates----------
extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

#define OTA_URL_SIZE 256

// -----serial communication---------
static const int RX_BUF_SIZE = 1024;
#define TXD_PIN (GPIO_NUM_1) //for UART0
#define RXD_PIN (GPIO_NUM_3) // for UART0