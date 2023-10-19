/** @file trig_helpers.c
 *  @brief Trig functions for inverse kinematics. *
 *  @author Lee Taylor
 *  @author Andew Steadman
 *  @bug No known bugs.
 */

#include <stdint.h>
#include <math.h>
#include <stdio.h>

int32_t fpsin(int32_t i)
{
    /* Convert (signed) input to a value between 0 and 8192. (8192 is pi/2, which is the region of the curve fit). */
    /* ------------------------------------------------------------------- */
    i <<= 1;
    uint32_t c = i<0; //set carry for output pos/neg

    if(i == (i|0x4000)) // flip input value to corresponding value in range [0..8192)
        i = (1<<15) - i;
    i = (i & 0x7FFF) >> 1;
    /* ------------------------------------------------------------------- */

    /* The following section implements the formula:
     = y * 2^-n * ( A1 - 2^(q-p)* y * 2^-n * y * 2^-n * [B1 - 2^-r * y * 2^-n * C1 * y]) * 2^(a-q)
    Where the constants are defined as follows:
    */
    enum {A1=3370945099UL, B1=2746362156UL, C1=292421UL};
    enum {n=13, p=32, q=31, r=3, a=12};

    uint32_t y = (C1*((uint32_t)i))>>n;
    y = B1 - (((uint32_t)i*y)>>r);
    y = (uint32_t)i * (y>>n);
    y = (uint32_t)i * (y>>n);
    y = A1 - (y>>(p-q));
    y = (uint32_t)i * (y>>n);
    y = (y+(1UL<<(q-a-1)))>>(q-a); // Rounding

    return c ? -y : y;
}


//Cos(x) = sin(x + pi/2)
// int32_t fpcos(int16_t i) { return fpsin((int16_t)(((uint16_t)(i)) + 8192U)); }

// #define fpcos(i) fpsin((int16_t)(((uint16_t)(i)) + 8192U))

int32_t scale_angle(float angle)
{
    return (int32_t)(angle * 91.0222f);
}

float fp_to_degrees(int32_t fp)
{
    return (float)(fp * 90.0f / 8192.0f);
}

// Absolute error <= 6.7e-5
int32_t acos_precise(float x) 
{
    float negate = x < 0;
    x = fabs(x);
    float ret = -0.0187293;
    ret = ret * x;
    ret = ret + 0.0742610;
    ret = ret * x;
    ret = ret - 0.2121144;
    ret = ret * x;
    ret = ret + 1.5707288;
    ret = ret * sqrt(1.0 - x);
    ret = ret - 2 * negate * ret;
    // return negate * 180.0f + ret * 57.29578f;
    return (int32_t)(negate * 16384 + ret * 5215.189175f);
}

static int16_t LUT[256] = {8192, 8172, 8151, 8131, 8110, 8090, 8069, 8049, 8028, 8008, 7987, 7967, 
7946, 7926, 7906, 7885, 7865, 7844, 7824, 7803, 7783, 7762, 7742, 7721, 7700, 7680, 7659, 7639, 
7618, 7598, 7577, 7556, 7536, 7515, 7495, 7474, 7453, 7433, 7412, 7391, 7371, 7350, 7329, 7308, 
7288, 7267, 7246, 7225, 7204, 7184, 7163, 7142, 7121, 7100, 7079, 7058, 7037, 7016, 6995, 6974, 
6953, 6932, 6911, 6890, 6869, 6848, 6827, 6805, 6784, 6763, 6742, 6720, 6699, 6678, 6656, 6635, 
6614, 6592, 6571, 6549, 6528, 6506, 6485, 6463, 6441, 6420, 6398, 6376, 6354, 6333, 6311, 6289, 
6267, 6245, 6223, 6201, 6179, 6157, 6135, 6113, 6090, 6068, 6046, 6024, 6001, 5979, 5956, 5934, 
5911, 5889, 5866, 5843, 5821, 5798, 5775, 5752, 5729, 5706, 5683, 5660, 5637, 5614, 5590, 5567, 
5544, 5520, 5497, 5473, 5450, 5426, 5402, 5378, 5354, 5330, 5306, 5282, 5258, 5234, 5210, 5185, 
5161, 5136, 5112, 5087, 5062, 5038, 5013, 4988, 4963, 4938, 4912, 4887, 4862, 4836, 4810, 4785, 
4759, 4733, 4707, 4681, 4655, 4628, 4602, 4575, 4549, 4522, 4495, 4468, 4441, 4414, 4386, 4359, 
4331, 4303, 4276, 4247, 4219, 4191, 4162, 4134, 4105, 4076, 4047, 4018, 3988, 3959, 3929, 3899, 
3869, 3838, 3808, 3777, 3746, 3715, 3683, 3652, 3620, 3588, 3555, 3523, 3490, 3457, 3424, 3390, 
3356, 3322, 3287, 3252, 3217, 3182, 3146, 3110, 3073, 3036, 2999, 2961, 2922, 2884, 2845, 2805, 
2765, 2724, 2683, 2641, 2598, 2555, 2511, 2467, 2422, 2376, 2329, 2281, 2232, 2182, 2131, 2079, 
2026, 1971, 1915, 1857, 1798, 1736, 1672, 1606, 1537, 1465, 1390, 1310, 1225, 1134, 1034, 925, 
801, 654, 462, 0};

int32_t acos_lut(int32_t x)
{
    int32_t negate = x < 0;
    x = abs(x);

    // Ensure within bounds
    if (x > 4096) { x = 4096; }

    // Grab value from LUT by converting to index
    int32_t ret = LUT[(x/16)-1];
    return negate ? 16384-ret : ret;
}