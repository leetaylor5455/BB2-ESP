#include <stdint.h>


/** @brief Returns servo angle between 0 +- 2^15
 *
 *  @param  theta_x       desired x plate angle
 *  @param  theta_y       desired y plate angle
 *  @param  p_ball_2d        ball 2d position (scaled by factor of 4096 relative to mm)
 *  @param  servo_angles  pointer to servo angles array
 * 
 *  @return 32 bit signed integer servo angle with range 0 +- 2^15
 */
void ik(int32_t theta_x, int32_t theta_y, int32_t p_ball_2d[2], int32_t (*servo_angles)[3]);