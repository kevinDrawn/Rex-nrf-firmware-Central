#include "torque_cyl.h"

// characteristic별 데이터 저장 - CHAR_A Right Shoe, CHAR_B Left Shoe
CharacteristicData char_cantilever_table[NUM_CHARACTERISTICS] = {
    [CHAR_A] = { .adc_zero = 1430, .c_slope = 0.06f*0.36f},
    [CHAR_B] = { .adc_zero = 2636, .c_slope = 0.06f},
    [CHAR_C] = { .adc_zero = 1970, .c_slope = 0.0231},
    [CHAR_D] = { .adc_zero = 1350, .c_slope = 0.06f*0.36f}, 
    [CHAR_L] = { .adc_zero = 1955, .c_slope = 0.03}, 
    [CHAR_J] = { .adc_zero = 1765, .c_slope = 0.027}, 
    [CHAR_M] = { .adc_zero = 2600, .c_slope = 0.025}, 
    [CHAR_I] = { .adc_zero = 2190, .c_slope = 0.027}, 
    [CHAR_N] = { .adc_zero = 2190, .c_slope = 0.027}, 
    [CHAR_O] = { .adc_zero = 2650, .c_slope = 0.027}, 
};

float axis_slope = 0.0223; // 바꿀 때 python 로깅도 함께 수정

// Axis Torque Sensing
float Measured_Torque_axis(float adc){
	// float adcval_bias = adc - 1700; // 바꿀 때 python 로깅도 함께 수정
    float adcval_bias = adc - 1230; // 바꿀 때 python 로깅도 함께 수정
    float torque = axis_slope * (adcval_bias); // Torque in N.mm

	float torque_axis = -0.001f * torque; //N.m

	return torque_axis;
}

float Measured_Torque_Character_cantilever(int16_t adc,
                                           CharacteristicType type)
{
    // 유효 범위 검사
    if (type >= NUM_CHARACTERISTICS) {
        return 0.0f;   // 안전 장치
    }

    // 선택된 characteristic 데이터 참조
    CharacteristicData *ch = &char_cantilever_table[type];

    // ADC 보정 및 토크 계산
    float adcval_bias = (float)adc - (float)ch->adc_zero;
    float torque_cantilever = ch->c_slope * adcval_bias;

    return torque_cantilever/1000;
}