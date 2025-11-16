#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "drv_spi.h"
#include "driver/gpio.h"

static const char *TAG = "main";
#define SDA GPIO_NUM_13
#define MISO GPIO_NUM_12
#define SCK GPIO_NUM_14
#define CS GPIO_NUM_27
#define DC GPIO_NUM_26
#define RES GPIO_NUM_25
#define BUSY GPIO_NUM_35

#define EPD_ARRAY (800 * 480 / 8)

void sendCmd(uint8_t command,spi_device_handle_t handle)
{
    uint8_t buff[1] = {command};
    gpio_set_level(CS, 0);
    gpio_set_level(DC, 0);
    spi_writeBytes(handle, 1, buff);
    gpio_set_level(CS, 1);
}

void sendData(uint8_t data,spi_device_handle_t handle)
{
    uint8_t buff[1] = {data};
    gpio_set_level(CS, 0);
    gpio_set_level(DC, 1);
    spi_writeBytes(handle, 1, buff);
    gpio_set_level(CS, 1);
}

void busy()
{
    while(1) 
    { 
        if (gpio_get_level(BUSY)) break;
    }
}

void init_display(spi_device_handle_t handle)
{
    gpio_set_level(RES, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);  
    gpio_set_level(RES, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);  
    
    sendCmd(0x1, handle);
    sendData(0x07, handle);
    sendData(0x07, handle);
    sendData(0x3f, handle);
    sendData(0x3f, handle);

    sendCmd(0x06, handle); 
    sendData(0x17, handle);
    sendData(0x17, handle);
    sendData(0x28, handle);
    sendData(0x17, handle);
    sendCmd(0x04, handle); 
    busy();
 
    sendCmd(0X00, handle); 
    sendData(0x1F, handle);
    sendCmd(0x61, handle); 
    sendData(0x03, handle);
    sendData(0x20, handle);
    sendData(0x01, handle);
    sendData(0xE0, handle);
    sendCmd(0X15, handle); 
    sendData(0x00, handle);
    sendCmd(0X50, handle); 
    sendData(0x10, handle);
    sendData(0x07, handle);
    sendCmd(0X60, handle); 
    sendData(0x22, handle);

}

void refreshScreen(spi_device_handle_t handle)
{   
  //update
  sendCmd(0x12, handle);   //DISPLAY update   
  vTaskDelay(10 / portTICK_PERIOD_MS);              //!!!The delay here is necessary, 200uS at least!!!     
  busy();          //waiting for the electronic paper IC to release the idle signal
}

//Clear screen display
void clearScreen(spi_device_handle_t handle)
{
  unsigned int i;
  //Write Data
  sendCmd(0x10, handle);      
  for(i=0;i<EPD_ARRAY;i++)       
  {
    sendData(0x00, handle);  
  }
  sendCmd(0x13, handle);      
  for(i=0;i<EPD_ARRAY;i++)       
  {
    uint8_t num = 0;
    if (i&0x8) num = 1;
    sendData(num, handle);  
  }
   refreshScreen(handle);   
}


void app_main(void)
{
    // ESP_LOGI(TAG, "ESP-IDF bare project started.");
    spi_device_handle_t gdey075t7;

    spi_bus_config_t cfg = {
        .mosi_io_num = SDA,
        .miso_io_num = -1,
        .sclk_io_num = SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096
    };
    spi_bus_initialize(SPI2_HOST, &cfg, SPI_DMA_CH_AUTO);
    // spi_begin(SDA, MISO, SCK, 64);
    // spi_addDevice(2, 1000000, CS, &gdey075t70;75t7);
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
        .command_bits = 0,
        .address_bits = 0
    };
    spi_bus_add_device(SPI2_HOST, &devcfg, &gdey075t7);
    // spi_addDevice(2, 1000000, -1, &gdey075t7);

    // gpio_set_direction(SDA, GPIO_MODE_OUTPUT);
    // gpio_set_direction(SCK, GPIO_MODE_OUTPUT);
    gpio_set_direction(CS, GPIO_MODE_OUTPUT);
    gpio_set_direction(DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(RES, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUSY, GPIO_MODE_INPUT);
    
    gpio_set_level(RES, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);  
    gpio_set_level(RES, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);  
    
    init_display(gdey075t7);
    
    clearScreen(gdey075t7);

    while (1) {
        ESP_LOGI(TAG, "Heartbeat...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
