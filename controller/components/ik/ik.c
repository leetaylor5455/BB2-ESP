#include <stdio.h>
#include <stdint.h>
#include "trig_utils.h"
#include "vector_utils.h"
#include "ik_utils.h"
#include "ik.h"
// #include "robot_config.h"


void ik(int32_t theta_x, int32_t theta_y, int32_t p_ball_2d[2], int32_t (*servo_angles)[3])
{

    // Ball position and angles
    // int32_t p_ball_3d[3] = {122880*2, 122880, 32768};
    // int32_t p_ball_3d[3] = {0, 0, 0};
    // int32_t theta_x = scale_angle(10);
    // int32_t theta_y = scale_angle(10);

    // Anchor points
    static int32_t p_anchor1_default[3] = {-204800, -118241, 0};
    static int32_t p_anchor2_default[3] = {204800, -118241, 0};
    static int32_t p_anchor3_default[3] = {0, 236483, 0};
    static int32_t p_anchor1[3];
    static int32_t p_anchor2[3];
    static int32_t p_anchor3[3];

    // Servo shaft points
    static int32_t p_servo_shaft1[3] = {-116119, -67041, -188416};
    static int32_t p_servo_shaft2[3] = {116119, -67041, -188416};
    static int32_t p_servo_shaft3[3] = {0, 134083, -188416};

    // Servo arm vectors
    static int32_t v_servo1[3] = {88681, 51200, 0};
    static int32_t v_servo2[3] = {-88681, 51200, 0};
    static int32_t v_servo3[3] = {0, -102400, 0};

    int32_t p_ball_3d[3] = {p_ball_2d[0]*4096, p_ball_2d[1]*4096, 24576};

    // // Servo angles
    // int32_t t_servo1;
    // int32_t t_servo2;
    // int32_t t_servo3;


    // Translation matrices
    static int32_t T[4][4] = { {1, 0, 0, 0},
                        {0, 1, 0, 0},
                        {0, 0, 1, 0},
                        {0, 0, 0, 4096} };

    static int32_t Ti[4][4] = { {1, 0, 0, 0},
                         {0, 1, 0, 0},
                         {0, 0, 1, 0},
                         {0, 0, 0, 4096} };

    // Rotation matrices
    static int32_t Rx[4][4] = { {1, 0, 0, 0},
                         {0, 0, 0, 0},
                         {0, 0, 0, 0},
                         {0, 0, 0, 1} };

    static int32_t Ry[4][4] = { {0, 0, 0, 0},
                         {0, 1, 0, 0},
                         {0, 0, 0, 0},
                         {0, 0, 0, 1} };

    // Apply population functions
    translation_matrix(&T, p_ball_3d, 1);
    translation_matrix(&Ti, p_ball_3d, -1);
    x_rotation_matrix(&Rx, theta_x); // multiply by 90 to scale up
    y_rotation_matrix(&Ry, theta_y);

    // // Apply transformation
    transform_point(&p_anchor1, p_anchor1_default, T, Ti, Rx, Ry);
    transform_point(&p_anchor2, p_anchor2_default, T, Ti, Rx, Ry);
    transform_point(&p_anchor3, p_anchor3_default, T, Ti, Rx, Ry);

    // printf("\n\np1: (%f, %f, %f)", p_anchor1[0]/4096.0f, p_anchor1[1]/4096.0f, p_anchor1[2]/4096.0f);
    // printf("\n\np2: (%f, %f, %f)", p_anchor2[0]/4096.0f, p_anchor2[1]/4096.0f, p_anchor2[2]/4096.0f);
    // printf("\n\np3: (%f, %f, %f)", p_anchor3[0]/4096.0f, p_anchor3[1]/4096.0f, p_anchor3[2]/4096.0f);

    (*servo_angles)[0] = saturate_angle(get_servo_angle(p_anchor1, p_servo_shaft1, v_servo1));
    (*servo_angles)[1] = saturate_angle(get_servo_angle(p_anchor2, p_servo_shaft2, v_servo2));
    (*servo_angles)[2] = saturate_angle(get_servo_angle(p_anchor3, p_servo_shaft3, v_servo3));

    // printf("\n\nservo 1 angle: %f", fp_to_degrees((*servo_angles)[0]));
    // printf("\n\nservo 2 angle: %f", fp_to_degrees((*servo_angles)[1]));
    // printf("\n\nservo 3 angle: %f\n\n", fp_to_degrees((*servo_angles)[2]));    

}