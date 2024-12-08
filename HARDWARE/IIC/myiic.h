#ifndef _MYIIC_H
#define _MYIIC_H
#include "sys.h"
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

//IO����
/* IO���� */
#define IIC_SCL(x)                  do { (x) ?                                                                  \
                                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET):   \
                                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); \
                                    } while (0)
#define IIC_SDA(x)                  do { (x) ?                                                                  \
                                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET):   \
                                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); \
                                    } while (0)
#define IIC_SDA_READ                ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET) ? 0 : 1)


//IIC���в�������
void iic_delay(void);
void iic_init(void);                //��ʼ��IIC��IO��				 
void iic_start(void);				//����IIC��ʼ�ź�
void iic_stop(void);	  			//����IICֹͣ�ź�
void iic_send_byte(u8 data);			//IIC����һ���ֽ�
u8 iic_read_byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 iic_wait_ack(void); 				//IIC�ȴ�ACK�ź�
void iic_ack(void);					//IIC����ACK�ź�
void iic_nack(void);				//IIC������ACK�ź�
void SDA_IN(void);
void SDA_OUT(void);

extern volatile uint16_t i2c_delay_cycles;

#endif

