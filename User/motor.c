#include "motor.h"

static MOTOR_t motor_instance;
static PID_t pid_position;
static PID_t pid_speed;

MOTOR_t *motor_ptr = &motor_instance;
uint32_t encoder_count_last;
uint32_t encoder_count;
long deffer;

void MOTOR_Enable(MOTOR_t *motor)
{
    //启动定时器
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor->motor_dutyfactor); //初始化占空比为
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 1);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, 0);
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void MOTOR_Init(MOTOR_t *motor)
{
    motor->pid_position_control = &pid_position;
    motor->pid_speed_control = &pid_speed;
    motor->pid_flag=0; //PID控制标志，0表示不进行PID计算，1表示进行PID计算
    motor->motor_dutyfactor=0; //电机占空比
    motor->motor_position_ref=3120; //电机目标位置
    motor->motor_speed_ref=0; //电机目标角速度
    motor->motor_position_fdb=0; //电机反馈位置
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1)
    {
        motor_ptr->pid_flag=1; //PID控制标志位置1，表示可以进行PID计算
        encoder_count=__HAL_TIM_GET_COUNTER(&htim2);
        deffer=encoder_count - encoder_count_last;
        if(deffer>32768) //处理编码器反转时的跳变
        {
            deffer-=65536;
        }
        else if(deffer<-32768)
        {
            deffer+=65536;
        }
        motor_ptr->motor_position_fdb+=deffer;
        encoder_count_last=encoder_count;
        motor_ptr->pid_speed_control->fdb= deffer; // 10ms采样周期
    }
}

void MOTOR_Control(MOTOR_t *motor)
{
    if(motor->pid_flag==1) //如果PID控制标志位置1，进行PID计算
    {
        motor->pid_flag=0; //清除PID控制标志
        //位置环PID计算
        motor->pid_position_control->ref=motor->motor_position_ref;
        motor->pid_position_control->fdb=motor->motor_position_fdb;
        PID_Calc_encoder_p(motor->pid_position_control);
        //将位置环输出作为速度环目标值
        motor->pid_speed_control->ref=motor->pid_position_control->output;
        //速度环PID计算
        PID_Calc(motor->pid_speed_control);
        //将速度环输出作为电机占空比
        float temp;
        if (motor->pid_speed_control->output <0)
        {
            temp = -motor->pid_speed_control->output;
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 0);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, 1);
        }
        else
        {
            temp = motor->pid_speed_control->output;
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 1);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, 0);
        }
        motor->motor_dutyfactor = temp;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor->motor_dutyfactor);
    }
}