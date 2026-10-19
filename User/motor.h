#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "math.h"
#include "pid.h"

typedef struct //重写
{
    uint8_t pid_flag; //PID控制标志，0表示不进行PID计算，1表示进行PID计算
    long motor_dutyfactor; //电机占空比
    long motor_position_ref; //电机目标位置
    long motor_speed_ref; //电机目标速度
    long motor_position_fdb; //电机反馈位置
    PID_t* pid_position_control; //位置环PID结构体指针
    PID_t* pid_speed_control; //速度环PID结构体指针
} MOTOR_t;

extern MOTOR_t* motor_ptr;
extern uint32_t encoder_count_last;
extern uint32_t encoder_count;
extern long deffer;

//电机初始化
void MOTOR_Init(MOTOR_t *motor);
//定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
//电机控制函数
void MOTOR_Control(MOTOR_t *motor);
//电机使能函数
void MOTOR_Enable(MOTOR_t *motor);
//电机失能函数
void MOTOR_Disable(MOTOR_t *motor);

#endif