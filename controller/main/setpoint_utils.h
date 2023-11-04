#include <stdint.h>

/** @brief Returns sin of value at time with given frequency, magnitude, and phase
 *
 *  @param  t time
 *  @param  freq desired frequency in Hz
 *  @param  mag desired maximum magnitude in mm
 *  @param  phase desired phase offset in degrees
 * 
 *  @return position
 */
int32_t setpoint_sin(int32_t t, float freq, int32_t mag, uint32_t phase);