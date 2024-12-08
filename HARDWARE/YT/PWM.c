#include "PWM.h"
TIM_HandleTypeDef     TIM3_Handler;          //��ʱ����� 
TIM_OC_InitTypeDef     TIM3_CH1Handler;        //��ʱ��3ͨ��1���
TIM_OC_InitTypeDef     TIM3_CH2Handler;        //��ʱ��3ͨ��2���


void TIM3_Init(u16 arr,u16 psc)
{  
    TIM3_Handler.Instance=TIM3;                          //ͨ�ö�ʱ��3
    TIM3_Handler.Init.Prescaler=psc;                     //��Ƶϵ��
    TIM3_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //���ϼ�����
    TIM3_Handler.Init.Period=arr;                        //�Զ�װ��ֵ
    TIM3_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//ʱ�ӷ�Ƶ����
    HAL_TIM_Base_Init(&TIM3_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM3_Handler); //ʹ�ܶ�ʱ��3�Ͷ�ʱ��3�����жϣ�TIM_IT_UPDATE   
}

//TIM3 PWM���ֳ�ʼ�� 
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
    TIM3_Handler.Instance=TIM3;             //��ʱ��3
    TIM3_Handler.Init.Prescaler=psc;       //��ʱ����Ƶ
    TIM3_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//���ϼ���ģʽ
    TIM3_Handler.Init.Period=arr;          //�Զ���װ��ֵ
    TIM3_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM3_Handler);       //��ʼ��PWM
    
    TIM3_CH1Handler.OCMode=TIM_OCMODE_PWM1; //ģʽѡ��PWM1
    TIM3_CH1Handler.Pulse=arr/2;            //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM3_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //����Ƚϼ���Ϊ�� 
    HAL_TIM_PWM_ConfigChannel(&TIM3_Handler,&TIM3_CH1Handler,TIM_CHANNEL_1);//����TIM3ͨ��2
    
    HAL_TIM_PWM_Start(&TIM3_Handler,TIM_CHANNEL_1);//����PWMͨ��2
    
    TIM3_CH2Handler.OCMode=TIM_OCMODE_PWM1; //ģʽѡ��PWM1
    TIM3_CH2Handler.Pulse=arr/2;            //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM3_CH2Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //����Ƚϼ���Ϊ�� 
    HAL_TIM_PWM_ConfigChannel(&TIM3_Handler,&TIM3_CH2Handler,TIM_CHANNEL_2);//����TIM3ͨ��2

    HAL_TIM_PWM_Start(&TIM3_Handler,TIM_CHANNEL_2);//����PWMͨ��2
    
}

//��ʱ���ײ�����������ʱ�ӣ������ж����ȼ�
//�˺����ᱻHAL_TIM_Base_Init()��������
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();            //ʹ��TIM3ʱ��
        HAL_NVIC_SetPriority(TIM3_IRQn,1,3);    //�����ж����ȼ�����ռ���ȼ�1�������ȼ�3
        HAL_NVIC_EnableIRQ(TIM3_IRQn);          //����ITM3�ж�   
    }
}

//��ʱ���ײ�������ʱ��ʹ�ܣ���������
//�˺����ᱻHAL_TIM_PWM_Init()����
//htim:��ʱ�����
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    if(htim->Instance==TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();            //ʹ�ܶ�ʱ��3
//        __HAL_AFIO_REMAP_TIM3_PARTIAL();        //TIM3ͨ�����Ų�����ӳ��ʹ��
        __HAL_RCC_GPIOA_CLK_ENABLE();            //����GPIOAʱ��
        
        GPIO_Initure.Pin=GPIO_PIN_6 | GPIO_PIN_7; //PB6|7
        GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //�����������
        GPIO_Initure.Pull=GPIO_PULLUP;          //����
        GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
        HAL_GPIO_Init(GPIOA,&GPIO_Initure);     
    }
}

//����TIMͨ��1|2��ռ�ձ�
//compare:�Ƚ�ֵ
void TIM_SetTIM3Compare1(u32 compare)
{
    TIM3->CCR1=compare; 
}
void TIM_SetTIM3Compare2(u32 compare)
{
    TIM3->CCR2=compare; 
}

//��ʱ��3�жϷ�����
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM3_Handler);
}

//�ص���������ʱ���жϷ���������
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
    MOVEX(950);//0��/500 - 60��/950 - 120��/1400 
    MOVEY(1340);//0��/900 - 60��/1340 - 120��/1780
}

// �����PWM�Ͷ�����ƺ���
void SetServoPWM(uint8_t channel, uint16_t pulse_width);
uint16_t GetServoPosition(uint8_t channel);


// PID��������ʼ������
void PID_Init(PIDController *pid, double kp, double ki, double kd) 
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error_sum = 0;
    pid->last_error = 0;
}


// PID���º���
double PID_Update(PIDController *pid, double setpoint, double measured_value) 
{
    double error = setpoint - measured_value;
    pid->error_sum += error;
    double derivative = error - pid->last_error;
    pid->last_error = error;
    return pid->kp * error + pid->ki * pid->error_sum + pid->kd * derivative;
}

