#include "setpoint_utils.h"
#include "trig_utils.h"
#include <math.h>
#include <stdio.h>

static int32_t cos_pos = 0;
static int32_t cycles_since_switch = 0;

int32_t setpoint_sin(int32_t t, float freq, int32_t mag, uint32_t phase)  {
    // Time is in milliseconds, so should t/1000 is time in seconds

    // For fpsin, time is 8192 = pi/2, so 1 second is equal to 5215
    // > hence, t in should be scaled by 5.215
    // For fpsin, magnitude is +- 4096, so dividing factor is 4096/mag
    // return mag*(int32_t)sin(freq*t/1000 + phase);

    cycles_since_switch++;

    // Every half cycle, tell cos to switch y half planes
    if (!phase && abs(fpsin(t)) >= 4090 && cycles_since_switch > 10) {
        cos_pos = 1-cos_pos;
        // fprintf(stdout, "cycle");
        cycles_since_switch = 0;
    } 

    if (phase) { // Atm assume that phase is always 90 degrees for circular path
        // With cos_pos being only 0 or 1, we can take 1-2*cos_pos to alternate 
        // between -1 and 1
        return (1-2*cos_pos)*fpsin(t + phase) / (4096/mag);
    } else {
        return fpsin(t) / (4096/mag);
    }
}