#ifndef PID_H
#define PID_H

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "math.h"

typedef struct {
    float KP;
    float KI;
    float KD;
    float fdb;
    float ref;
    float cur_error;
    float integral;
    float error[2];
    float output;
    float outputMax;
    float outputMin;
} PID_t;

void PID_PositionParam_init(PID_t *pid);

void PID_SpeedParam_init(PID_t *pid);

void PID_Calc(__IO PID_t *pid);

void PID_Calc_encoder_p(__IO PID_t *pid);

#endif