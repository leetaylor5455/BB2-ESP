
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include <stdint.h>

#define CONFIG_FREERTOS_HZ 100

// Please consult the datasheet of your servo before changing the following parameters
#define SERVO_MIN_PULSEWIDTH_US 1000  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2000  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -4551   // Minimum angle (-50deg scaled to 2^15 range)
#define SERVO_MAX_DEGREE        4551    // Maximum angle (50deg scaled to 2^15 range)

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000      // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        3333    // 3333 ticks, 3ms

/** @brief Converts angle for comparator
 *
 *  @param  angle scaled angle (0 +- 2^15)
 * 
 *  @return 32 bit usigned integer
 */
uint32_t angle_to_compare(int32_t angle);

/** @brief Calculates pulse width and generates PWM signal on given channel
 *
 *  @param  channel mcpwm comparator handle for servo
 *  @param  angle scaled angle (0 +- 2^15)
 * 
 *  @return void
 */
void move_servo(mcpwm_cmpr_handle_t channel, int32_t angle);

/** @brief Sets up MCPWM channel on given GPIO pin
 *
 *  @param  servo_pin servo GPIO pin num
 * 
 *  @return MCPWM comparator handle
 */
mcpwm_cmpr_handle_t create_servo_instance(int32_t servo_pin);