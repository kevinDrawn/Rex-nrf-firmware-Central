/*
 * filter.c
 *
 *  Created on: Jan 13, 2025
 *      Author: psh
 */
#include "filter_central.h"

float M_PI = 3.141592;

// Initialize Filter
void Filter_Init(Filter *filter, float cutoff_freq, uint16_t window_size) {

    // Low-pass filter coefficient calculate
    float RC = 1.0f / (2.0f * M_PI * cutoff_freq);
    filter->alpha = dt / (RC + dt);

    // Moving Average Filter Initialization
    filter->buffer_size = window_size;
    filter->buffer = (float *)malloc(window_size * sizeof(float));

    for (uint16_t i = 0; i < window_size; i++) {
        filter->buffer[i] = 0.0f;
    }

    filter->sum = 0.0f;
    filter->buffer_index = 0;
    filter->prev_input = 0.0f;
    filter->prev_output = 0.0f;
}

// Cancel Memory
void Filter_Destroy(Filter *filter) {
    if (filter->buffer != NULL) {
        free(filter->buffer);
        filter->buffer = NULL;
    }
}

// Low-pass filter
float Filter_Low_pass(Filter *filter, float input) {
    float output = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_output;
    filter->prev_output = output;
    filter->prev_input = input;
    return output;
}

// Moving Average Filter
float Filter_Moving_Average(Filter *filter, float input) {
    filter->sum -= filter->buffer[filter->buffer_index];
    filter->buffer[filter->buffer_index] = input;
    filter->sum += input;
    filter->buffer_index = (filter->buffer_index + 1) % filter->buffer_size;
    return filter->sum / filter->buffer_size;
}


