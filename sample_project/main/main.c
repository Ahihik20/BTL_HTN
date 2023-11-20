#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/i2c.h"
#include "i2c-lcd.h"
#include "DHT.h"

// Constants
#define I2C_MASTER_PORT I2C_NUM_0
#define SDA_PIN GPIO_NUM_21 // LCD 1602 PIN // LCD 1602 HAS TO USE 5V VCC
#define SCL_PIN GPIO_NUM_22 // LCD 1602 PIN // LCD 1602 HAS TO USE 5V VCC
#define DRY_ADC_VALUE 4095.0
#define WET_ADC_VALUE 1400.0
#define SOIL_MOISTURE_PERCENTAGE 5 // SOID MOISTURE TO TURN ON OR OFF THE PUMP
#define WET_SOIL_MOISTURE 100.0
#define LCD_UPDATE_DELAY_MS 1500
#define DHT_GPIO_PIN 13 // DHT11 DATA PIN // DHT11 SENSOR HAS TO USE 3.3V VCC 
#define PUMP_GPIO_PIN 12 // PUMP PIN
// GPIO 34 IS SOIL MOISTURE SENSOR DATA PIN
// SOIL MOISTURE SENSOR HAS TO USE 3.3V VCC

static esp_adc_cal_characteristics_t adc1_chars;
char buffer[100];
int level = 0;

int counter = 0; //set timer to tunn on PUMP
int pumpMode = 0;
// counter = 0 => pumpMode = 0 => The pump operates in automatic mode
// counter > 0 => pumpMode = 1 => The pump operates in user-controlled mode

char timerRun[] = ""; // timer to turn on pump

typedef struct {
    double Temp;
    double Humi;
} dataDHT;

dataDHT dht11; // data DHT11

/** 
 * @brief Initialize I2C master 
 */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    i2c_param_config(I2C_MASTER_PORT, &conf);
    return i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
}

/** 
 * @brief Initialize ADC 
 */
static void adc_init(void)
{
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11)); // GPIO 34
}

/** 
 * @brief Read soil moisture percentage 
 */
static float read_soil_moisture_percentage(void)
{
    uint32_t voltage = adc1_get_raw(ADC1_CHANNEL_6); // GPIO 34
    float soil_moisture_percentage = (DRY_ADC_VALUE - voltage) / (DRY_ADC_VALUE - WET_ADC_VALUE) * WET_SOIL_MOISTURE;

    if (soil_moisture_percentage > 100)
    {
        soil_moisture_percentage = 100;
    }

    //vTaskDelay(3000 / portTICK_PERIOD_MS);
    return soil_moisture_percentage;
}

/** 
 * @brief Update LCD display 
 */
static void update_lcd_display(float soil_moisture_percentage, double temperature, double humidity)
{
    sprintf(buffer, "Soil: %.2f %%", soil_moisture_percentage);
    lcd_put_cur(0, 0); // Print soil moisture on the first line
    lcd_send_string(buffer);

    sprintf(buffer, "T:%.2fC H:%.2f%% ", temperature, humidity);
    lcd_put_cur(1, 0); // Print temperature and humidity on the second line
    lcd_send_string(buffer);
}

/** 
 * @brief Read data DHT11
 */

void readDataDHT11(void)
{
        dht_read_data(DHT_GPIO_PIN, &dht11.Humi, &dht11.Temp);
        //vTaskDelay(1500 / portTICK_PERIOD_MS);
}

/** 
 * @brief Control the pumper
 */
void controlPump(float soil_moisture_percentage){
    if(counter > 0){
        pumpMode = 1;
        counter --;
    }
    else{
        pumpMode = 0;
        counter = 0;
    }

    if(pumpMode == 1) gpio_set_level (PUMP_GPIO_PIN , 1);
    else{
        if(soil_moisture_percentage > SOIL_MOISTURE_PERCENTAGE){
            gpio_set_level (PUMP_GPIO_PIN , 0);
        }
        else{
            gpio_set_level (PUMP_GPIO_PIN , 1);
        }
    }
}

/** 
 * @brief Control the system
 */
void smartFarm(void *pvParameter){
    while (1)
    {
        float soil_moisture_percentage = read_soil_moisture_percentage();
        readDataDHT11();
        char* time = __TIME__;
        printf("ESP32 DHT11 TEST:%s!\r\n", time);
        printf("Soil Moisture Percentage is: %.2f %% \n", soil_moisture_percentage);
        printf("Temp=%f, Humi=%f\r\n", dht11.Temp, dht11.Humi);
        update_lcd_display(soil_moisture_percentage, dht11.Temp, dht11.Humi);
        controlPump(soil_moisture_percentage);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}


void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    lcd_init();
    lcd_clear();
    adc_init();
    esp_rom_gpio_pad_select_gpio( PUMP_GPIO_PIN ) ;
    gpio_set_direction ( PUMP_GPIO_PIN , GPIO_MODE_OUTPUT ) ;

    xTaskCreate(&smartFarm, "Smart Farm", 2048, NULL, 1, NULL);


    // while (1)
    // {
    //     float soil_moisture_percentage = read_soil_moisture_percentage();
    //     printf("Soil Moisture Percentage is: %.2f %% \n", soil_moisture_percentage);
    //     printf("Temp=%f, Humi=%f\r\n", dht11.Temp, dht11.Humi);
    //     update_lcd_display(soil_moisture_percentage, dht11.Temp, dht11.Humi);
    // }
}