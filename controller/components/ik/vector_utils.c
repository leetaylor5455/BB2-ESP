#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "trig_utils.h"

const int32_t MULTIPLIER = 4096;

void print_matrix(int32_t M[4][4], int32_t rows, int32_t cols)
{
    for (int32_t i = 0; i < rows; i++) {
        printf("\n");
        for (int j = 0; j < cols; j++) {
            printf("%ld, ", M[i][j]);
        }
    }

    printf("\n");
}

void cross(float vout[3], float v1[3], float v2[3])
{
    vout[0] = v1[1]*v2[2]-v1[2]*v2[1];
    vout[1] =-(v1[0]*v2[2]-v1[2]*v2[0]);
    vout[2] = v1[0]*v2[1]-v1[1]*v2[0];
}

float fast_sqrt_float(float x) {

    int32_t i;
    float y, r;

    y = x * 0.5f;
    i = *(int32_t*)&x;
    i = 0x5f3759df - (i >> 1);
    r = *(float*)&i;
    r = r * (1.5f - (r * r * y));
    // r = r * (1.5f - (r * r * y)); // no need for that much precision
    
    return x * r;
}

int32_t fast_sqrt(int32_t val) {
    int32_t temp, g=0, b = 0x8000, bshft = 15;
    do {
        if (val >= (temp = (((g << 1) + b)<<bshft--))) {
           g += b;
           val -= temp;
        }
    } while (b >>= 1);
    return g;
}

int32_t norm(int32_t vec[3])
{
    return fast_sqrt(vec[0]*vec[0] +  vec[1]*vec[1] + vec[2]*vec[2]);
}

void translation_matrix(int32_t (*T)[4][4], int32_t rotation_point[3], int32_t multiply)
{
    for (int i = 0; i < 3; i++) {
        (*T)[i][3] = multiply * rotation_point[i];
    }

    // print_matrix((*T), 4, 4);
    
}

void x_rotation_matrix(int32_t (*Rx)[4][4], int32_t theta_x)
{
    (*Rx)[1][1] = fpcos(theta_x);
    (*Rx)[1][2] = -fpsin(theta_x);
    (*Rx)[2][1] = fpsin(theta_x);
    (*Rx)[2][2] = fpcos(theta_x);
}

void y_rotation_matrix(int32_t (*Ry)[4][4], int32_t theta_y)
{
    (*Ry)[0][0] = fpcos(theta_y);
    (*Ry)[0][2] = fpsin(theta_y);
    (*Ry)[2][0] = -fpsin(theta_y);
    (*Ry)[2][2] = fpcos(theta_y);
}

void transform_point(int32_t (*point)[3], int32_t p_reference[3], int32_t T[4][4], int32_t Ti[4][4], int32_t Rx[4][4], int32_t Ry[4][4])
{
    int i, j, k;

    // Add a dummy dimension
    int32_t point_dummy[4] = {p_reference[0], p_reference[1], p_reference[2], 4096};

    // Intermediate result matrix for T * Rx
    int32_t TRx[4][4];

    // Multiply T and Rx matrices
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            TRx[i][j] = 0;
            for (k = 0; k < 4; k++) {
                TRx[i][j] += T[i][k] * Rx[k][j];
            }
        }
    }

    // print_matrix(TRx, 4, 4);

    // Intermediate result matrix for TRx * Ry
    int32_t TRxy[4][4];

    // Multiply trx and r_y matrices
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            TRxy[i][j] = 0;
            for (k = 0; k < 4; k++) {
                TRxy[i][j] += TRx[i][k] * Ry[k][j];
            }
        }
    }

    // Amend compounding 4096 multiplication
    for (i = 1; i < 3; i++) {
        for (j = 0; j < 3; j+=2) {
            TRxy[i][j] /= MULTIPLIER;
        }
    }

    // print_matrix(TRxy, 4, 4);
    // print_matrix(Ti, 4, 4);

    // Intermediate result matrix for TRxy * Ti
    int32_t TRxyTi[4][4];

    // TRxy[3][3] = 4096;
    // Ti[3][3] = 4096;

    // Multiply TRxy and Ti matrices
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            TRxyTi[i][j] = 0;
            for (k = 0; k < 4; k++) {
                TRxyTi[i][j] += TRxy[i][k] * Ti[k][j];
            }
        }
    }

    for (i = 0; i < 3; i++) {
        TRxyTi[i][3] /= MULTIPLIER;
    }

    // print_matrix(TRxyTi, 4, 4);

    // Multiply point by full transformation matrix
    for (i = 0; i < 3; i++) {
        (*point)[i] = 0;
        for (j = 0; j < 4; j++) {
            (*point)[i] += TRxyTi[i][j] * point_dummy[j];
        }
        (*point)[i] /= MULTIPLIER;
        
    }
}

int32_t angle_between(int32_t v_servo[3], int32_t v_anchor_shaft[3])
{
    // Dot product
    int i;
    int32_t dot = 0;
    for (i = 0; i < 3; i++) {
        dot+= v_servo[i] * v_anchor_shaft[i];
    }

    int32_t den = (norm(v_servo)*norm(v_anchor_shaft))/4096;

    return 16384 - acos_lut( dot / den );
}