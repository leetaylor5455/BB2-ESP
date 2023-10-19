#include <stdint.h>


/** @brief Fixed point sin approximation
 *
 *  Implements the 5-order polynomial approximation to sin(x).
 *  The result is accurate to within +- 1 count. ie: +/-2.44e-4.
 *
 *  @param i   angle (with 2^15 units/circle)
 *  @return    32 bit fixed point Sine value (4.12) (ie: +4096 = +1 & -4096 = -1)
 */
int32_t fpsin(int32_t i);

/** @brief Fixed point cos approximation
 *
 *  Implements the 5-order polynomial approximation to cos(x).
 *  The result is accurate to within +- 1 count. ie: +/-2.44e-4.
 *
 *  @param i   angle (with 2^15 units/circle)
 *  @return    32 bit fixed point Sine value (4.12) (ie: +4096 = +1 & -4096 = -1)
 */
#define fpcos(i) fpsin((int16_t)(((uint16_t)(i)) + 8192U))

/** @brief Scales angle in degrees to value between 0 and 2^15 for fast sine compute
 *
 *  @param angle  angle in degrees
 *  @return       32 bit int between 0 and +-2^15
 */
int32_t scale_angle(float angle);

float fp_to_degrees(int32_t fp);

/** @brief Arccos approximation
 *
 *  @param x  ratio
 *  @return   angle in range 0 +- 2^15
 */
int32_t acos_precise(float x);

/** @brief Uses lookup table to find acos (no interpolation)
 *
 *  @param x  ratio represented by 0-4096 (equivalent to 0-1 in floating point)
 *  @return   angle in range 0 +- 2^15
 */
int32_t acos_lut(int32_t x);