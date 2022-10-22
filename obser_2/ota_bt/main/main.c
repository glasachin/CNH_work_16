

#include "constants.h"



// configuration for LED blinking
unsigned int state = 1; // To blink LED at PIN2


void app_main(void)
{
    system_init();
    // data is a pointer to the memory equal to buffer size
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    
    strcpy(sensor_data,"Welcome to Soil Sensor from CNH!\n");

    while(1)
    {
        state ^= 0x01;
        gpio_set_level(2, state);
        vTaskDelay(pdMS_TO_TICKS(100));
       
        //----BT sending data
        if(bt_connected == 1)
        {
            // esp_spp_write(bt_param->data_ind.handle, strlen(sensor_data), &sensor_data);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        
    }
}

void system_init()
{
    print_mux = xSemaphoreCreateMutex();
    //GPIO Init
    ESP_LOGI(TAG,"Running");
    gpio_pad_select_gpio(2);

    /* Set the GPIO as a output */
    gpio_set_direction(2, GPIO_MODE_OUTPUT);

    //Initialize Global Variables
    init_globals();

    //Serial Port Init
    serial_init();

    //ota_wifi_init
    ota_wifi_init();

    //Initialize Bluetooth
    bt_init();

}

void serial_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void init_globals(void)
{
    // Initialize Sensor data String
    strcpy(ack_data, "data received!\n");
    // receiving data string
    strcpy(rx_data,"");
    //Sensor data string
    strcpy(sensor_data,"");// = "";
}