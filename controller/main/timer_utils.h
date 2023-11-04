#include "esp_timer.h"

/** @brief Timer callback
 *
 */
void timer_callback(void* arg);


/** @brief Sets up timer using esp_timer API
 *
 *  @return esp_timer handle
 */
esp_timer_handle_t setup_timer();


/** @brief Returns current millisecond count since timer start
 *
 *  @return Milliseconds elapsed
 */
uint64_t get_millis();

/** @brief Returns fpsin adjusted count
 *
 *  @return Milliseconds in range +-8192
 */
int32_t get_sin_timer();

