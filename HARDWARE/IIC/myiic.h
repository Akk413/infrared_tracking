#ifndef _MYIIC_H
#define _MYIIC_H
#include "sys.h"
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

//IO操作
/* IO操作 */
#define IIC_SCL(x)                  do { (x) ?                                                                  \
                                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET):   \
                                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); \
                                    } while (0)
#define IIC_SDA(x)                  do { (x) ?                                                                  \
                                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET):   \
                                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); \
                                    } while (0)
#define IIC_SDA_READ                ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET) ? 0 : 1)


//IIC所有操作函数
void iic_delay(void);
void iic_init(void);                //初始化IIC的IO口				 
void iic_start(void);				//发送IIC开始信号
void iic_stop(void);	  			//发送IIC停止信号
void iic_send_byte(u8 data);			//IIC发送一个字节
u8 iic_read_byte(unsigned char ack);//IIC读取一个字节
u8 iic_wait_ack(void); 				//IIC等待ACK信号
void iic_ack(void);					//IIC发送ACK信号
void iic_nack(void);				//IIC不发送ACK信号
void SDA_IN(void);
void SDA_OUT(void);

extern volatile uint16_t i2c_delay_cycles;

#endif

