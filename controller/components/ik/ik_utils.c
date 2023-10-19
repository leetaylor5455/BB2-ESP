#include <stdint.h>
#include <stdio.h>
#include "robot_config.h"
#include "trig_utils.h"
#include "vector_utils.h"

int32_t inverse_law_of_cosine(int32_t distance)
{
    // Decompose the acos argument so we can use as many constants as possible
    float dTerm = 2048.0f / distance;
    int32_t arg = (int32_t)(dTerm * COSINE_LINKAGE_TERM) + distance/50;

    // printf("\ninverse law of cosine: %f", fp_to_degrees(acos_lut(arg)));
    return acos_lut(arg);
}

int32_t get_servo_angle(int32_t anchor_point[3], int32_t shaft_point[3], int32_t v_servo[3])
{
    // Get vector from anchor to shaft
    int32_t v_anchor_shaft[3];
    int32_t v_servo_reduced[3];

    for (int i = 0; i < 3; i++) {
        v_anchor_shaft[i] = anchor_point[i] - shaft_point[i];
        v_servo_reduced[i] = v_servo[i];

        // Prevent overflow by dividing by 32
        v_anchor_shaft[i] >>= 5;
        v_servo_reduced[i] >>= 5;
    }

    int32_t distance = norm(v_anchor_shaft) * 32;

    // printf("\ndistance: %d", distance);

    return -(angle_between(v_servo_reduced, v_anchor_shaft) - inverse_law_of_cosine(distance));
}

int32_t saturate_angle(int32_t angle)
{
    if (angle < -4551) {
        return -4551;
    } else if (angle > 4551) {
        return 4551;
    } else {
        return angle;
    }
}

