/* 
This file is modified file by Sachin Sharma. In this basic OTA firmware is also there other wise
OTA functionality will loose after first upgrae.

Main blink code is written along with the OTA functionality. In this code upgrade is done only when
user wants to upgrade. For that Serial communication functionality is added.

It is better to use serial receive data using interrupts.

*/

#include "constants.h"





// configuration for LED blinking
unsigned int state = 1; // To blink LED at PIN2


void app_main(void)
{
    system_init();
    // data is a pointer to the memory equal to buffer size
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    
    strcpy(sensor_data,"Hi, This is Sachin Sharma!\n");

    while(1)
    {
        state ^= 0x01;
        gpio_set_level(2, state);
        vTaskDelay(pdMS_TO_TICKS(100));
        // // printf("Hello, This is Sachin\n");
        // const int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        // // printf("JCBRO: waiting is done\n");
        // if (rxBytes > 0) 
        // {          
        //    if(*data == 'U')
        //    {
        //        //upgrade the code
        //        printf("Downloading and Upgrading the code");
        //     //    simple_ota_example_task("ota_example_task");
        //    }
        // }

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