#include <stdint.h>

/** @brief Returns servo angle between 0 +- 2^15
 *
 *  @param  anchor_point anchor point
 *  @param  shaft_point servo shaft point
 * 
 *  @return 32 bit signed integer servo angle with range 0 +- 2^15
 */
int32_t get_servo_angle(int32_t anchor_point[3], int32_t shaft_point[3], int32_t v_servo[3]);


/** @brief Applies inverse law of cosine to find crank angle (from slider crank equation)
 *
 *  @param  distance distance between anchor point and servo shaft
 * 
 *  @return 32 bit signed integer angle with range 0 +- 2^15
 */
int32_t inverse_law_of_cosine(int32_t distance);

/** @brief Saturates angle between min and max available from the servo
 *
 *  @param  angle distance between anchor point and servo shaft
 * 
 *  @return 32 bit signed integer angle with range 0 +- 2^15
 */
int32_t saturate_angle(int32_t angle);