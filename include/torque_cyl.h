#pragma once

/*
 * axis.c
 *
 *  Created on: Feb 21, 2025
 *      Author: qoxmf
 */

/*
 * axis.h
 *
 *  Created on: Feb 21, 2025
 *      Author: qoxmf
 */


#include "filter_central.h"
#include "stdio.h"

typedef enum {
    CHAR_A = 0,
    CHAR_B,
    CHAR_C,
    CHAR_D,
    CHAR_L,
    CHAR_J,
    CHAR_M,
    CHAR_I,
    CHAR_N,
    CHAR_O,
    NUM_CHARACTERISTICS
} CharacteristicType;

typedef struct {
    int   adc_zero;   // ADC 오프셋
    float c_slope;    // 변환 기울기
} CharacteristicData;

extern CharacteristicData char_table[NUM_CHARACTERISTICS];

float Measured_Torque_axis(float adc);
float Measured_Torque_Character_cantilever(int16_t adc,
                                           CharacteristicType type);


