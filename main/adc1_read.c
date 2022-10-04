/* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "soc/adc_channel.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
#if CONFIG_IDF_TARGET_ESP32
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_channel_t channel = ADC_CHANNEL_6;     // GPIO7 if ADC1, GPIO17 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_0;


static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}


static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

int CH_TO_PIN(int pin){
    switch(pin){
        case ADC1_CHANNEL_0_GPIO_NUM:
            return ADC_CHANNEL_0;
        case ADC1_CHANNEL_1_GPIO_NUM:
            return ADC_CHANNEL_1;
        case ADC1_CHANNEL_2_GPIO_NUM:
            return ADC_CHANNEL_2;
        case ADC1_CHANNEL_3_GPIO_NUM:
            return ADC_CHANNEL_3;
        case ADC1_CHANNEL_4_GPIO_NUM:
            return ADC_CHANNEL_4;
        case ADC1_CHANNEL_5_GPIO_NUM:
            return ADC_CHANNEL_5;
        case ADC1_CHANNEL_6_GPIO_NUM:
            return ADC_CHANNEL_6;
        case ADC1_CHANNEL_7_GPIO_NUM:
            return ADC_CHANNEL_7;
        default:
            return -1;
    }
}

void init_adc(int pin){
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
        adc1_config_width(width);
        adc1_config_channel_atten(CH_TO_PIN(pin), atten);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

}

/**
 * 
 * GPIO Lookup Macros
 *
 * There are macros available to specify the GPIO number of a ADC channel, or vice versa. e.g.
 *
 *   ADC1_CHANNEL_0_GPIO_NUM is the GPIO number of ADC1 channel 0.
 *
 *   ADC1_GPIOn_CHANNEL is the ADC1 channel number of GPIO n.
 *
 */

uint32_t read_voltage(int pin){
        uint32_t adc_reading = 0;
        if(CH_TO_PIN(pin) == -1)
            return UINT32_MAX;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading += adc1_get_raw((adc1_channel_t)CH_TO_PIN(pin));
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        return esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
}
