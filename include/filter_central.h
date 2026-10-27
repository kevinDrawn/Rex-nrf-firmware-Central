/*
 * filter.h
 *
 *  Created on: Jan 13, 2025
 *      Author: qoxmf
 */
#pragma once

#ifndef INC_FILTER_H_
#define INC_FILTER_H_

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

// Low Pass Filter
#define SAMPLING_RATE 1000.0f // Sampling Frequency (1 kHz)
#define CUTOFF_FREQ 10.0f     // Default Cut-off Frequency
#define dt 1.0f/SAMPLING_RATE

// Filter Cut-off Frequency
#define cutoff_freqs_torque_axis 7.0
#define cutoff_freqs_motor_command 60
#define cutoff_freqs_hall 10.0
#define cutoff_freqs_hall_dot 10.0
#define cutoff_freqs_torque_can 10.0

// 필터 구조체 정의
typedef struct {
    float alpha;                // Filter Coefficient
    float prev_input;           // Previous input value
    float prev_output;          // Previous output value

    float *buffer;              // 데이터를 저장하는 버퍼 (가변 크기 지원)
    uint16_t buffer_size;       // 이동 평균 필터의 윈도우 크기
    uint16_t buffer_index;      // 현재 버퍼 인덱스
    float sum;                  // 합계 변수
} Filter;

extern float M_PI;

// 필터 초기화 및 사용 함수 선언
void Filter_Init(Filter *filter, float cutoff_freq, uint16_t window_size);
void Filter_Destroy(Filter *filter);
float Filter_Low_pass(Filter *filter, float input);
float Filter_Moving_Average(Filter *filter, float input);

#endif /* INC_FILTER_H_ */
