#include "pid.h"

void PID_SpeedParam_init(PID_t *pid)
{
    pid->KP = 1.0f;
    pid->KI = 0;
    pid->KD = 0;
    pid->fdb = 0;
    pid->ref = 0;
    pid->cur_error = 0;
    pid->error[0] = 0;
    pid->error[1] = 0;
    pid->output = 0;
    pid->outputMax = 999;
    pid->outputMin = 0;
}

void PID_PositionParam_init(PID_t *pid)
{
    pid->KP = 0.01f;
    pid->KI = 0;
    pid->KD = 0;
    pid->fdb = 0;
    pid->ref = 0;
    pid->cur_error = 0;
    pid->integral = 0;
    pid->error[0] = 0;
    pid->error[1] = 0;
    pid->output = 0;
    pid->outputMax = 104;
    pid->outputMin = 0;
}

void PID_Calc(__IO PID_t *pid)
{
    pid->cur_error = pid->ref - pid->fdb;
    pid->output += pid->KP * (pid->cur_error - pid->error[1])
                 + pid->KI * pid->cur_error 
                 + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
    pid->error[0] = pid->error[1];
    pid->error[1] = pid->cur_error;

    if (pid->output > pid->outputMax) pid->output = pid->outputMax;
    if (pid->output < -pid->outputMax) pid->output = -pid->outputMax;
}

void PID_Calc_encoder_p(__IO PID_t *pid){
    pid->cur_error = pid->ref - pid->fdb;
    pid->integral += pid->cur_error;
    pid->output = pid->KP * pid->cur_error + pid->KI * pid->integral + pid->KD * (pid->cur_error - pid->error[1]);
    pid->error[0] = pid->error[1];
    pid->error[1] = pid->ref - pid->fdb;
    /*设定输出上限*/
    if(pid->output > pid->outputMax) pid->output = pid->outputMax;
    if(pid->output < -pid->outputMax) pid->output = -pid->outputMax;

    if(fabs(pid->output)<pid->outputMin)
        pid->output=0;
}