/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"

const static char *TAG2 = "PLATE";

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
//ADC1 Channels
#if CONFIG_IDF_TARGET_ESP32
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_0
#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_5
#else
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_2
#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_3
#endif

#if (SOC_ADC_PERIPH_NUM >= 2) && !CONFIG_IDF_TARGET_ESP32C3
/**
 * On ESP32C3, ADC2 is no longer supported, due to its HW limitation.
 * Search for errata on espressif website for more details.
 */
#define EXAMPLE_USE_ADC2            1
#endif

#if EXAMPLE_USE_ADC2
//ADC2 Channels
#if CONFIG_IDF_TARGET_ESP32
#define EXAMPLE_ADC2_CHAN0          ADC_CHANNEL_0
#else
#define EXAMPLE_ADC2_CHAN0          ADC_CHANNEL_0
#endif
#endif  //#if EXAMPLE_USE_ADC2

#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_11

// static int adc_raw[2][10];
static int32_t adc_raw;
// static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
// static void example_adc_calibration_deinit(adc_cali_handle_t handle);

#define CONFIG_FREERTOS_HZ 100

void configure_gpio(int32_t pin)
{
    gpio_config_t gpioConfig;
    gpioConfig.pin_bit_mask = (1ULL << pin);
    gpioConfig.mode = GPIO_MODE_OUTPUT;
    gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpioConfig.intr_type = GPIO_INTR_DISABLE;

    gpio_config(&gpioConfig);
}

int32_t adc_to_mm_x(int32_t adc)
{
    // return (int32_t)(adc * 403 - 783196); // scaled up to 4096 counts per mm
    // return (int32_t)(adc * 0.105f - 191.21f);
    return (int32_t)(adc * 0.1f - 189);
    // return (int32_t)(adc * 0.1113f - 207.87f);
    // return adc;
}

int32_t adc_to_mm_y(int32_t adc)
{
    // return (int32_t)(-adc * 379 + 702259); // scaled up to 4096 counts per mm
    // return (int32_t)(-adc * 0.0925f + 171.45f);
    return (int32_t)(-adc * 0.082f + 156);
    // return adc;
}

void correct_coords(int32_t *x, int32_t *y)
{
    // ESP_LOGI(TAG2, "Raw: (%ld, %ld)", *x, *y);

    // isolate by quadrant
    // top right
    if (*x > 0 && *y > 0) {
        // *x += *y / 8; 
        *y += *x / 10;
    } else {
        // *x += (*y) / 10;
        *y += *x / 20;
    }
    // ESP_LOGI(TAG2, "Cor: (%ld, %ld)", *x, *y);
    
}


adc_oneshot_unit_handle_t configure_adc(void)
{
    //-------------ADC2 Init---------------//
    static adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    static adc_oneshot_unit_handle_t adc2_handle;
    static adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    //-------------ADC2 Config---------------//
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, EXAMPLE_ADC2_CHAN0, &config));

    return adc2_handle;
}


void read_plate(int32_t (*plate_coords)[2], adc_oneshot_unit_handle_t adc2_handle)
{
    static int32_t x;
    static int32_t y;

    //-------------Reading---------------//

    // Set pins for x reading
    gpio_set_level(15, 1);
    gpio_set_level(16, 0);

    vTaskDelay(pdMS_TO_TICKS(1));

    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, EXAMPLE_ADC2_CHAN0, &adc_raw));
        
    x = adc_to_mm_x(adc_raw);
    
    // Set pins for y reading
    gpio_set_level(15, 0);
    gpio_set_level(16, 1);

    vTaskDelay(pdMS_TO_TICKS(1));

    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, EXAMPLE_ADC2_CHAN0, &adc_raw));
    y = adc_to_mm_y(adc_raw);

    // correct_coords(&x, &y);
    (*plate_coords)[0] = x;
    (*plate_coords)[1] = y;
}


// void app_main(void)
// {

//     //-------------GPIO Init---------------//
//     configureGPIO(17);
//     configureGPIO(16);
//     configureGPIO(2);
//     configureGPIO(15);

//     gpio_set_level(17, 1); // pin 17 is always HIGH
//     gpio_set_level(2, 0); // pin 2 is always LOW

//     // Coords
//     int32_t x = 0;
//     int32_t y = 0;

//     //-------------ADC1 Init---------------//
//     adc_oneshot_unit_handle_t adc1_handle;
//     adc_oneshot_unit_init_cfg_t init_config1 = {
//         .unit_id = ADC_UNIT_1,
//     };
//     ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

//     //-------------ADC1 Config---------------//
//     adc_oneshot_chan_cfg_t config = {
//         .bitwidth = ADC_BITWIDTH_DEFAULT,
//         .atten = EXAMPLE_ADC_ATTEN,
//     };
//     ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));
//     ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN1, &config));

//     //-------------ADC1 Calibration Init---------------//
//     adc_cali_handle_t adc1_cali_handle = NULL;
//     bool do_calibration1 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC_ATTEN, &adc1_cali_handle);


// #if EXAMPLE_USE_ADC2
//     //-------------ADC2 Init---------------//
//     adc_oneshot_unit_handle_t adc2_handle;
//     adc_oneshot_unit_init_cfg_t init_config2 = {
//         .unit_id = ADC_UNIT_2,
//         .ulp_mode = ADC_ULP_MODE_DISABLE,
//     };
//     ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

//     //-------------ADC2 Calibration Init---------------//
//     adc_cali_handle_t adc2_cali_handle = NULL;
//     bool do_calibration2 = example_adc_calibration_init(ADC_UNIT_2, EXAMPLE_ADC_ATTEN, &adc2_cali_handle);

//     //-------------ADC2 Config---------------//
//     ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, EXAMPLE_ADC2_CHAN0, &config));
// #endif  //#if EXAMPLE_USE_ADC2

//     while (1) {

//         // Set pins for x reading
//         gpio_set_level(15, 1);
//         gpio_set_level(16, 0);
        
//         // Read adc
//         ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, EXAMPLE_ADC2_CHAN0, &adc_raw[1][0]));
        
//         x = adc_to_mm_x(adc_raw[1][0]);
//         // ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_2 + 1, EXAMPLE_ADC2_CHAN0, adc_raw[1][0]);
//         // ESP_LOGI(TAG, "ADC%d Channel[%d] mm: %d", ADC_UNIT_2 + 1, EXAMPLE_ADC2_CHAN0, x);
        
//         // Set pins for y reading
//         gpio_set_level(15, 0);
//         gpio_set_level(16, 1);

//         vTaskDelay(pdMS_TO_TICKS(100));

//         ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, EXAMPLE_ADC2_CHAN0, &adc_raw[1][0]));
//         y = adc_to_mm_y(adc_raw[1][0]);
//         // ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_2 + 1, EXAMPLE_ADC2_CHAN0, adc_raw[1][0]);
//         // ESP_LOGI(TAG, "ADC%d Channel[%d] mm: %d", ADC_UNIT_2 + 1, EXAMPLE_ADC2_CHAN0, y);
        

//         // ESP_LOGI(TAG, "Raw: (%d, %d)", x, y);
//         correct_coords(&x, &y);
//         // ESP_LOGI(TAG, "Coord: (%d, %d)", x, y);
//         // ESP_LOGI(TAG, "Coords: (%d, %d)", x, y);

//         // ESP_LOGI(TAG, "Cor: (%d, %d)", x, y);

//         // if (do_calibration2) {
//         //     ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_handle, adc_raw[1][0], &voltage[1][0]));
//         //     ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_2 + 1, EXAMPLE_ADC2_CHAN0, voltage[1][0]);
//         // }
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }

//     //Tear Down
//     ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
//     if (do_calibration1) {
//         example_adc_calibration_deinit(adc1_cali_handle);
//     }

// #if EXAMPLE_USE_ADC2
//     ESP_ERROR_CHECK(adc_oneshot_del_unit(adc2_handle));
//     if (do_calibration2) {
//         example_adc_calibration_deinit(adc2_cali_handle);
//     }
// #endif //#if EXAMPLE_USE_ADC2
// }