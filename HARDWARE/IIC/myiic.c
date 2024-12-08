#include "myiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////     
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F103开发板
//IIC驱动代码       
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/5/30
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved                                      
//////////////////////////////////////////////////////////////////////////////////     

/**
 * @brief   IIC延时函数
 * @note    用于控制IIC通信速率
 * @param   无
 * @retval  无
 */
volatile uint16_t i2c_delay_cycles =1;
static void iic_delay(void)
{
    uint16_t i;
    for(i = 0; i < i2c_delay_cycles; i++)
    {
        delay_us(1);// 忙等待循环以生成所需的延时
    }
}


//IIC初始化
void iic_init(void)
{
    GPIO_InitTypeDef gpio_init_struct = {0};
    
    __HAL_RCC_GPIOB_CLK_ENABLE();   //使能GPIOB时钟
    
    //PH4,5初始化设置
    gpio_init_struct.Pin=GPIO_PIN_10|GPIO_PIN_11;
    gpio_init_struct.Mode=GPIO_MODE_OUTPUT_OD;  //推挽输出
    gpio_init_struct.Pull=GPIO_NOPULL;          //上拉
    gpio_init_struct.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOB,&gpio_init_struct);
    
    iic_stop();
}

//产生IIC起始信号
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
//产生IIC停止信号
void iic_stop(void)
{
    IIC_SDA(0);
    iic_delay();
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(1);
    iic_delay();
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
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



//产生ACK应答
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


//不产生ACK应答
void iic_nack(void)
{
    IIC_SDA(1);
    iic_delay();
    IIC_SCL(1);
    iic_delay();
    IIC_SCL(0);
    iic_delay();
}


//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答              
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
    
    /* 使能GPIO端口时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* 配置SDA引脚 */
    gpio_init_struct.Pin = GPIO_PIN_11;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);
    
}

void SDA_OUT(void)
{

    GPIO_InitTypeDef gpio_init_struct = {0};
    
    /* 使能GPIO端口时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* 配置SDA引脚 */
    gpio_init_struct.Pin = GPIO_PIN_11;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);
    
}


uint8_t iic_read_byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    
    SDA_IN(); // 设置SDA为输入模式
    for (i = 0; i < 8; i++)
    {
        IIC_SCL(0); 
        delay_us(2);
        IIC_SCL(1);
        receive <<= 1;
        if (IIC_SDA_READ) // 检查SDA线上的数据
            receive++;
        delay_us(1);
    }
    IIC_SCL(0);
    SDA_OUT(); // 设置SDA为输出模式
    if (!ack)
        iic_nack(); // 发送NACK信号
    else
        iic_ack();  // 发送ACK信号
    return receive;
}
