#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "dht.h"

#define DHT_GPIO_PIN 11
#define PUMP_GPIO_PIN 12

typedef struct 
{
    float Temp;
    float Humi;
}data_dht11;

void readDataDht11(void *pvParameter){
    setDHTgpio(DHT_GPIO_PIN);
    while(1){
        printf("DHT Sensor Readings\n" );
		int ret = readDHT();
		
		errorHandler(ret);

		printf("Humidity %.2f %%\n", getHumidity());
		printf("Temperature %.2f degC\n\n", getTemperature());
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    vTaskDelete(NULL);
}

// void controlPump(void *pvParameter){
//     while (1)
//     {

//         vTaskDelay(pdMS_TO_TICKS(5000));
//     }
//     vTaskDelete(NULL);
// }
void app_main(void)
{
    xTaskCreate(&readDataDht11,"Read Data DHT11", 2048, NULL, 1, NULL);
}
