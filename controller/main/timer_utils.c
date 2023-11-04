#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "esp_timer.h"
#include "timer_utils.h"

// Global variable to store the timer value
static uint64_t milliseconds = 0;
const static int32_t PI_BY_TWO = 8192;
// const static int32_t PI = 8192*2;
static int32_t sin_timer = -PI_BY_TWO;

static int32_t addition_factor = 1;

// Callback function for the timer
void timer_callback(void* arg) {

    if (milliseconds >= PI_BY_TWO*2) {
        addition_factor = 1 - addition_factor; // Toggle between 1 and 0
        milliseconds = 0;
    } else {
        milliseconds++;
        if (addition_factor > 0) {
            sin_timer++;
        } else {
            sin_timer--;
        }
    }
}

esp_timer_handle_t setup_timer() {
    // Configure and start the timer args
    const esp_timer_create_args_t timer_args = {
            .callback = &timer_callback,
            .name = "millisecond_timer"
    };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    return timer;
}

uint64_t get_millis() {
    return sin_timer;
}

int32_t get_sin_timer() {
    return sin_timer;
}