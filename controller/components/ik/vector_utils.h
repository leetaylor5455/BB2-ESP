#include <stdint.h>

/** @brief Cross product of two vectors
 *
 *  @param vout  output vector
 *  @param v1    vector 1
 *  @param v2    vector 2
 * 
 *  @return      void
 */
void cross(float vout[3], float v1[3], float v2[3]);

/** @brief Length of a vector
 *
 *  @param vec 3d vector
 * 
 *  @return    length
 */
int32_t norm(int32_t vec[3]);

/** @brief Float sqrt using Newton Rhapson
 *
 *  @param  x
 * 
 *  @return square root of x
 */
float fast_sqrt_float(float x);

/** @brief Fast integer sqrt with bit shifting only (by Jim Ulery) 
 *
 *  @param  x
 * 
 *  @return square root of x
 */
int32_t fast_sqrt(int32_t val);

/** @brief Populates translation matrix given rotation point
 *
 *  @param  T 4x4 translation matrix pointer
 *  @param  rotation_point 3D rotation point
 *  @param  multiply for inverse, set to -1, else set to 1
 * 
 *  @return void
 */
void translation_matrix(int32_t (*T)[4][4], int32_t rotation_point[3], int32_t multiply);

/** @brief Populates x rotation matrix
 *
 *  @param  Rx 4x4 rotation matrix pointer
 *  @param  theta_x  x rotation angle in degrees
 * 
 *  @return void
 */
void x_rotation_matrix(int32_t (*Rx)[4][4], int32_t theta_x);

/** @brief Populates y rotation matrix
 *
 *  @param  Ry 4x4 rotation matrix pointer
 *  @param  theta_x  y rotation angle in degrees
 * 
 *  @return void
 */
void y_rotation_matrix(int32_t (*Ry)[4][4], int32_t theta_y);

/** @brief Performs the transformation on a given 3D point
 *
 *  @param  point point to be rotated
 *  @param  p_reference reference point
 *  @param  T forward translation matrix
 *  @param  Ti inverse translation matrix
 *  @param  Rx x rotation matrix
 *  @param  Ry y rotation matrix
 * 
 *  @return void
 */
void transform_point(int32_t (*point)[3], int32_t p_reference[3], int32_t T[4][4], int32_t Ti[4][4], int32_t Rx[4][4], int32_t Ry[4][4]);

/** @brief Prints 4x4 matrix to console
 *
 *  @param  M matrix
 *  @param  rows number of rows
 *  @param  cols number of cols
 * 
 *  @return void
 */
void print_matrix(int32_t M[4][4], int32_t rows, int32_t cols);

/** @brief Uses the dot product to determine angle between two vectors
 *
 *  @param  v_servo servo arm vector
 *  @param  v_anchor_shaft vector from anchor point to shaft
 *  
 *  @return 32 bit signed integer 0 +- 2^15
 */
int32_t angle_between(int32_t v_servo[3], int32_t v_anchor_shaft[3]);