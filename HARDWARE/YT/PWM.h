#ifndef _PWM_H
#define _PWM_H
#include "sys.h"

// PID控制器结构体
typedef struct 
{
    double kp;
    double ki;
    double kd;
    double error_sum;
    double last_error;
} PIDController;

extern TIM_HandleTypeDef TIM3_Handler;      //定时器句柄 

void TIM3_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM_SetTIM3Compare1(u32 compare);
void TIM_SetTIM3Compare2(u32 compare);


void PWM_init(void);
void MOVEX(uint16_t Compare);
void MOVEY(uint16_t Compare);
void MOVE_Init(void);
void PID_Init(PIDController *pid, double kp, double ki, double kd);
double PID_Update(PIDController *pid, double setpoint, double measured_value);

#endif 


