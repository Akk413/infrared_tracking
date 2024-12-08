#include "PWM.h"
TIM_HandleTypeDef     TIM3_Handler;          //定时器句柄 
TIM_OC_InitTypeDef     TIM3_CH1Handler;        //定时器3通道1句柄
TIM_OC_InitTypeDef     TIM3_CH2Handler;        //定时器3通道2句柄


void TIM3_Init(u16 arr,u16 psc)
{  
    TIM3_Handler.Instance=TIM3;                          //通用定时器3
    TIM3_Handler.Init.Prescaler=psc;                     //分频系数
    TIM3_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM3_Handler.Init.Period=arr;                        //自动装载值
    TIM3_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
    HAL_TIM_Base_Init(&TIM3_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM3_Handler); //使能定时器3和定时器3更新中断：TIM_IT_UPDATE   
}

//TIM3 PWM部分初始化 
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
    TIM3_Handler.Instance=TIM3;             //定时器3
    TIM3_Handler.Init.Prescaler=psc;       //定时器分频
    TIM3_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//向上计数模式
    TIM3_Handler.Init.Period=arr;          //自动重装载值
    TIM3_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM3_Handler);       //初始化PWM
    
    TIM3_CH1Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM3_CH1Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM3_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //输出比较极性为高 
    HAL_TIM_PWM_ConfigChannel(&TIM3_Handler,&TIM3_CH1Handler,TIM_CHANNEL_1);//配置TIM3通道2
    
    HAL_TIM_PWM_Start(&TIM3_Handler,TIM_CHANNEL_1);//开启PWM通道2
    
    TIM3_CH2Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM3_CH2Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM3_CH2Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //输出比较极性为高 
    HAL_TIM_PWM_ConfigChannel(&TIM3_Handler,&TIM3_CH2Handler,TIM_CHANNEL_2);//配置TIM3通道2

    HAL_TIM_PWM_Start(&TIM3_Handler,TIM_CHANNEL_2);//开启PWM通道2
    
}

//定时器底册驱动，开启时钟，设置中断优先级
//此函数会被HAL_TIM_Base_Init()函数调用
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();            //使能TIM3时钟
        HAL_NVIC_SetPriority(TIM3_IRQn,1,3);    //设置中断优先级，抢占优先级1，子优先级3
        HAL_NVIC_EnableIRQ(TIM3_IRQn);          //开启ITM3中断   
    }
}

//定时器底层驱动，时钟使能，引脚配置
//此函数会被HAL_TIM_PWM_Init()调用
//htim:定时器句柄
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    if(htim->Instance==TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();            //使能定时器3
//        __HAL_AFIO_REMAP_TIM3_PARTIAL();        //TIM3通道引脚部分重映射使能
        __HAL_RCC_GPIOA_CLK_ENABLE();            //开启GPIOA时钟
        
        GPIO_Initure.Pin=GPIO_PIN_6 | GPIO_PIN_7; //PB6|7
        GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用推挽输出
        GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
        GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
        HAL_GPIO_Init(GPIOA,&GPIO_Initure);     
    }
}

//设置TIM通道1|2的占空比
//compare:比较值
void TIM_SetTIM3Compare1(u32 compare)
{
    TIM3->CCR1=compare; 
}
void TIM_SetTIM3Compare2(u32 compare)
{
    TIM3->CCR2=compare; 
}

//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM3_Handler);
}

//回调函数，定时器中断服务函数调用
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim==(&TIM3_Handler))
    {
    }
}


void MOVEX(uint16_t Compare)
{
    TIM_SetTIM3Compare1(Compare);
}

void MOVEY(uint16_t Compare)
{
    TIM_SetTIM3Compare2(Compare);
}

void MOVE_Init(void)
{
    MOVEX(950);//0°/500 - 60°/950 - 120°/1400 
    MOVEY(1340);//0°/900 - 60°/1340 - 120°/1780
}

// 假设的PWM和舵机控制函数
void SetServoPWM(uint8_t channel, uint16_t pulse_width);
uint16_t GetServoPosition(uint8_t channel);


// PID控制器初始化函数
void PID_Init(PIDController *pid, double kp, double ki, double kd) 
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error_sum = 0;
    pid->last_error = 0;
}


// PID更新函数
double PID_Update(PIDController *pid, double setpoint, double measured_value) 
{
    double error = setpoint - measured_value;
    pid->error_sum += error;
    double derivative = error - pid->last_error;
    pid->last_error = error;
    return pid->kp * error + pid->ki * pid->error_sum + pid->kd * derivative;
}

