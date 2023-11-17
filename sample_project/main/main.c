#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "DHT.h"

#define DHT_GPIO_PIN 17
#define PUMP_GPIO_PIN 12

typedef struct
{
    double Temp;
    double Humi;
}dataDHT;

dataDHT dht11;


void readDataDHT11(void *pvParameter){
    while (1)
    {
        dht_read_data(DHT_GPIO_PIN, &dht11.Humi, &dht11.Temp);
        printf("Temp=%f, Humi=%f\r\n", dht11.Temp, dht11.Humi);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

int level = 0;

void controlPump(void *pvParameter){
    while (1) {
        level = 1- level;
    if(gpio_get_level(PUMP_GPIO_PIN)){
        printf("ON\n");
    }
    else{
        printf("Off\n");
    }
    gpio_set_level ( PUMP_GPIO_PIN , level);
    printf("level: %d\n",level );
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_rom_gpio_pad_select_gpio( PUMP_GPIO_PIN ) ;
    gpio_set_direction ( PUMP_GPIO_PIN , GPIO_MODE_OUTPUT ) ;
    printf("ESP32 DHT11 TEST:%s,%s!\r\n", __DATE__, __TIME__);
    xTaskCreate(&readDataDHT11, "read", 2048, NULL, 1, NULL);
    xTaskCreate(&controlPump, "Control Pump", 2048, NULL, 2, NULL);
}