#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "ik.h"
#include "servo_utils.h"
#include "plate_utils.h"
#include "trig_utils.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "trig_utils.h"
#include "timer_utils.h"
#include "setpoint_utils.h"
#include "esp_timer.h"
// #include "driver/mcpwm_prelude.h"

#define CONFIG_FREERTOS_HZ 100

const static int32_t MAX_ANGLE = 910; // 10 degrees (deg * 91 for scaling)
const static float lowpass_factor = 0.0f;

const static char *TAG = "CTRLR";

int32_t saturate(int32_t theta) {
    if (theta > MAX_ANGLE) {
        return MAX_ANGLE;
    } 
    
    if (theta < -MAX_ANGLE) {
        return -MAX_ANGLE;
    }

    return theta;
}

int32_t sign(int32_t x)
{
    return (x > 0) - (x < 0);
}

float lowpass(float raw, float prev)
{
    return lowpass_factor*prev + (1-lowpass_factor)*raw;
}

void app_main(void)
{
    // Timing
    static int32_t ms_per_cycle = 10;  
    int32_t cycles_per_v_read = 3; // Number of cycles per velocity reading
    int32_t v_read_cycle = 0;      // Current v read cycle

    // Plate angles
    int32_t theta_x = 0;
    int32_t theta_y = 0;
    // Servo angles
    int32_t servo_angles[3] = {0, 0, 0};
    int32_t prev_servo_angles[3] = {0, 0, 0};
    int32_t smoothed_servo_angles[3] = {0, 0, 0};
    int32_t i;
    float smoothing_ratio = 0.94f;

    // Plate coords
    static int32_t raw_plate_signal[2] = {0, 0};
    static int32_t output_plate_signal[2] = {0, 0};
    static int32_t filter_cutoff_cycles = 30; // Cutoff for when ball is not on plate
    static int32_t max_d = 4; // Maximum difference in position between samples
    static int32_t origin[2] = {0, 0};

    // Control states
    float setpoint[2] = {0.0f, 0.0f};
    float coords[2] = {0.0f, 0.0f};
    float prev_coords[2] = {0.0f, 0.0f};
    float offs[2] = {0.0f, 0.0f};
    float prev_offs[2] = {0.0f, 0.0f};
    float vels[2] = {0.0f, 0.0f};
    int32_t sums[2] = {0, 0};

    // Filtering vars
    int32_t cycles_since_loss = 0;
    int32_t cycles_since_spike[2] = {1, 1};
    // int32_t cycles_since_spike_y = 1;
    int32_t ball_on = 1;

    // Control gains (separate for eventual tuning - sim says dynamics are different due to geometry)
    static float kpx = 0.5f;
    static float kix = 0.01f;
    static float kdx = 220.0f;

    static float kpy = 0.5f;
    static float kiy = 0.01f;
    static float kdy = 220.0f;
    
    //-------------Servo Init---------------//
    mcpwm_cmpr_handle_t servo1 = create_servo_instance(32);
    mcpwm_cmpr_handle_t servo2 = create_servo_instance(25);
    mcpwm_cmpr_handle_t servo3 = create_servo_instance(27);

    //-------------ADC Init---------------//
    adc_oneshot_unit_handle_t adc_handle = configure_adc();

    //-------------GPIO Init---------------//
    configure_gpio(17);
    configure_gpio(16);
    configure_gpio(2);
    configure_gpio(15);

    gpio_set_level(17, 1); // pin 17 is always HIGH
    gpio_set_level(2, 0); // pin 2 is always LOW

    // Start the clock

    esp_timer_handle_t timer = setup_timer();
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 50)); // 50 us period
    // uint16_t t = 0;
    
    // For fixed point sine function to work, it needs to take in a value between +-8192
    // -> sine timer oscillates between these values
    int32_t sin_t = 0;


    while (1) {

        // Get time
        // t = get_millis();
        sin_t = get_sin_timer();

        // Set dynamic setpoint (circle)
        for (i = 0; i < 2; i++) {
            setpoint[i] = (float)setpoint_sin(sin_t, 1, 10, i*8192U);
        }

        // Use fprintf for easy copy into spreadsheet visualisation
        // fprintf(stdout, "%.2f, %.2f\n", setpoint[0], setpoint[1]);

        // ESP_LOGI(TAG, "st: (%ld)", sin_t);
        
        // Get the ball position
        read_plate(&raw_plate_signal, adc_handle);

        // ESP_LOGI(TAG, "Raw: (%ld, %ld)", raw_plate_signal[0], raw_plate_signal[1]);

        //-------------Plate Coord Filtering---------------//

        // When nothing is pressed on the plate, x coord goes very low
        if (raw_plate_signal[0] < -120 || raw_plate_signal[1] > 100) {
            cycles_since_loss++;
            // In case it's lost momentarily, check that it's off for multiple cycles
            if (cycles_since_loss > filter_cutoff_cycles) {
                for (i = 0; i < 2; i++) {
                    coords[i] = 0;
                    sums[i] = 0;
                }
                cycles_since_loss = 0;
                ball_on = 0;
            }

        } else { // Signal is stable
            if (ball_on) {
                // Limit dx
                // If raw signal is more than expected different than the previous (filtered) coord, it is a spike
                // But as time goes on, we need to allow for a larger margin (+max difference per cycle)
                for (i = 0; i < 2; i++) {
                    if (abs(raw_plate_signal[i] - prev_coords[i]) > max_d*cycles_since_spike[i]) {
                        cycles_since_spike[i]++;
                        coords[i] = prev_coords[i];
                    } else {
                        coords[i] = 0.5f*((float)(raw_plate_signal[i]) + prev_coords[i]);
                        // coords[i] = lowpass((float)(raw_plate_signal[i]), prev_coords[i]);
                        prev_coords[i] = coords[i];
                        cycles_since_spike[i] = 1;
                    }
                }
            } else {
                // Ball has gone from not on the plate, to on the plate, so switch instantly
                ball_on = 1;
                for (i = 0; i < 2; i++) {
                    coords[i] = raw_plate_signal[i];
                    prev_coords[i] = coords[i];
                }
            }
        }        
        

        // ESP_LOGI(TAG, "Out: (%ld, %ld)", output_plate_signal[0], output_plate_signal[1]);

        if (ball_on) {
            // Offsets
            for (i = 0; i < 2; i++) {
                offs[i] = coords[i] - setpoint[i];
                sums[i] += offs[i];
                // Anti windup type condition
                // > if the ball is travelling in the opposite direction to the sum
                // > set to zero
                // if (sign(sums[i]) != sign((int32_t)(offs[i]))) {
                //     sums[i] = 0;
                // }
            }

            // Read velocity only after certain no. of cycles (too short intervals otherwise)
            if (v_read_cycle >= cycles_per_v_read) {
                // Set states
                for (i = 0; i < 2; i++) {
                    vels[i] = (offs[i] - prev_offs[i]) / cycles_per_v_read; // Normalise for per-cycle
                    prev_offs[i] = offs[i];
                }
                v_read_cycle = 0;
            } else {
                v_read_cycle++;
            }
        }
               
        

        // ESP_LOGI(TAG, "sx: %ld", sums[0]);
        // ESP_LOGI(TAG, "sy: %ld", sums[1]);
        // ESP_LOGI(TAG, "sign: %ld", sign(20));
        // ESP_LOGI(TAG, "ox: %ld", ox);
        // ESP_LOGI(TAG, "prev_ox: %ld", prev_ox);

        // sx += active_plate_signal[0];
        // sy += active_plate_signal[1];

        // P
        // theta_y = -saturate((int32_t)(kpx*ox));
        // theta_x = saturate((int32_t)(kpy*oy));

        // PD
        // theta_y = -saturate((int32_t)(kpx*ox + kdx*vx));
        // theta_x = saturate((int32_t)(kpy*oy + kdy*vy));

        // ESP_LOGI(TAG, "theta_y: %ld", theta_y);

        // PID
        theta_y = -saturate((int32_t)(kpx*offs[0] + kdx*vels[0] + kix*sums[0]));
        theta_x = saturate((int32_t)(kpy*offs[1] + kdy*vels[1] + kiy*sums[1]));
               

        // ESP_LOGI(TAG, "theta_x: %ld", theta_x);
        // ESP_LOGI(TAG, "theta_y: %ld", theta_y);
        // ESP_LOGI(TAG, "theta_y: %f", fp_to_degrees(theta_y));

        // ik(theta_x, theta_y-182, origin, &servo_angles);
        for (i = 0; i < 2; i++) {
            output_plate_signal[i] = (int32_t)(coords[i]);
        }

        // Constants added to thetas are for rough leveling of the plate
        ik(theta_x+100, theta_y-230, output_plate_signal, &servo_angles);

        // Smooth servo angles
        // Also not optimal (current overload at large accelerations - best fixed in hardware)
        for (i = 0; i < 3; i++) {
            smoothed_servo_angles[i] = ((float)(servo_angles[i]) * (1.0f - smoothing_ratio)) + ((float)(prev_servo_angles[i]) * smoothing_ratio);
            prev_servo_angles[i] = smoothed_servo_angles[i];
        }
        
        // Move the servos to the positions calculated by IK
        move_servo(servo1, smoothed_servo_angles[0]);
        move_servo(servo2, smoothed_servo_angles[1]);
        move_servo(servo3, smoothed_servo_angles[2]);

        // fprintf for easy serial copy into spreadsheet
        // fprintf(stdout, "%ld, %ld\n", coords[0], theta_y-182);
        // fprintf(stdout, "%.2f, %.2f\n", coords[0], coords[1]);
        // fprintf(stdout, "%ld, %ld\n", raw_plate_signal[0], raw_plate_signal[1]);

        // Delay by a bit (sets watchdog timer off - could be current overload?)
        // Not optimal, needs fixing so that cycles can be faster
        vTaskDelay(pdMS_TO_TICKS(ms_per_cycle));
    }
    

}
