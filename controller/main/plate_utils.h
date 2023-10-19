#include <stdint.h>
#include "esp_adc/adc_oneshot.h"

/** @brief Configures GPIO for digital output
 *
 *  @param  pin GPIO pin num
 * 
 *  @return void
 */
void configure_gpio(int32_t pin);

/** @brief Configures ADC channel 2
 * 
 *  @return ADC one shot unit handle
 */
adc_oneshot_unit_handle_t configure_adc(void);

/** @brief Converts ADC value to scaled x-axis mm output
 *
 *  @param  adc ADC output
 * 
 *  @return 32 bit signed int
 */
int32_t adc_to_mm_x(int32_t adc);

/** @brief Converts ADC value to scaled y-axis mm output
 *
 *  @param  adc ADC output
 * 
 *  @return 32 bit signed int
 */
int32_t adc_to_mm_y(int32_t adc);

/** @brief Applies a transform to both axis to realign them (due to voltage imbalance)
 *
 *  @param  x x-axis coord
 *  @param  y y-axis coord
 * 
 *  @return void
 */
void correct_coords(int32_t *x, int32_t *y);

/** @brief Performs routine for reading plate and writes to array pointer
 *
 *  @param  plate_coords plate coordinates array
 * 
 *  @return void
 */
void read_plate(int32_t (*plate_coords)[2], adc_oneshot_unit_handle_t adc2_handle);