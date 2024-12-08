#include "myiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////     
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F103������
//IIC��������       
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2017/5/30
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved                                      
//////////////////////////////////////////////////////////////////////////////////     

/**
 * @brief   IIC��ʱ����
 * @note    ���ڿ���IICͨ������
 * @param   ��
 * @retval  ��
 */
volatile uint16_t i2c_delay_cycles =1;
static void iic_delay(void)
{
    uint16_t i;
    for(i = 0; i < i2c_delay_cycles; i++)
    {
        delay_us(1);// æ�ȴ�ѭ���������������ʱ
    }
}


//IIC��ʼ��
void iic_init(void)
{
    GPIO_InitTypeDef gpio_init_struct = {0};
    
    __HAL_RCC_GPIOB_CLK_ENABLE();   //ʹ��GPIOBʱ��
    
    //PH4,5��ʼ������
    gpio_init_struct.Pin=GPIO_PIN_10|GPIO_PIN_11;
    gpio_init_struct.Mode=GPIO_MODE_OUTPUT_OD;  //�������
    gpio_init_struct.Pull=GPIO_NOPULL;          //����
    gpio_init_struct.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOB,&gpio_init_struct);
    
    iic_stop();
}

//����IIC��ʼ�ź�
void iic_start(void)
{
    IIC_SDA(1);
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(0);
    iic_delay();
    IIC_SCL(0);
    iic_delay();
}      
//����IICֹͣ�ź�
void iic_stop(void)
{
    IIC_SDA(0);
    iic_delay();
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(1);
    iic_delay();
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 iic_wait_ack(void)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;
    
    IIC_SDA(1);
    iic_delay();
    IIC_SCL(1);
    iic_delay();
    
    while (IIC_SDA_READ != 0)
    {
        waittime++;
        if (waittime > 250)
        {
            iic_stop();
            rack = 1;
            break;
        }
    }
    
    IIC_SCL(0);
    iic_delay();
    
    return rack;
} 



//����ACKӦ��
void iic_ack(void)
{
    IIC_SDA(0);
    iic_delay();
    IIC_SCL(1);
    iic_delay();
    IIC_SCL(0);
    iic_delay();
    IIC_SDA(1);
    iic_delay();
}


//������ACKӦ��
void iic_nack(void)
{
    IIC_SDA(1);
    iic_delay();
    IIC_SCL(1);
    iic_delay();
    IIC_SCL(0);
    iic_delay();
}


//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��              
void iic_send_byte(u8 data)
{                        
    uint8_t t;
    
    for (t=0; t<8; t++)
    {
        IIC_SDA((data & 0x80) >> 7);
        iic_delay();
        IIC_SCL(1);
        iic_delay();
        IIC_SCL(0);
        data <<= 1;
    }
    
    IIC_SDA(1);
}


void SDA_IN(void)
{

    GPIO_InitTypeDef gpio_init_struct = {0};
    
    /* ʹ��GPIO�˿�ʱ�� */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* ����SDA���� */
    gpio_init_struct.Pin = GPIO_PIN_11;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);
    
}

void SDA_OUT(void)
{

    GPIO_InitTypeDef gpio_init_struct = {0};
    
    /* ʹ��GPIO�˿�ʱ�� */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* ����SDA���� */
    gpio_init_struct.Pin = GPIO_PIN_11;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);
    
}


uint8_t iic_read_byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    
    SDA_IN(); // ����SDAΪ����ģʽ
    for (i = 0; i < 8; i++)
    {
        IIC_SCL(0); 
        delay_us(2);
        IIC_SCL(1);
        receive <<= 1;
        if (IIC_SDA_READ) // ���SDA���ϵ�����
            receive++;
        delay_us(1);
    }
    IIC_SCL(0);
    SDA_OUT(); // ����SDAΪ���ģʽ
    if (!ack)
        iic_nack(); // ����NACK�ź�
    else
        iic_ack();  // ����ACK�ź�
    return receive;
}
