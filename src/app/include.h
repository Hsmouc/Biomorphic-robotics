#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"

/*
 * Include �û��Զ����ͷ�ļ�
 */

#include  "gpio.h"       //IO�ڲ���
#include  "default.h"
#include  "LED.h"        //��
#include  "exti.h"       //EXTI�ⲿGPIO�ж�
#include  "uart.h"       //����
#include  "lptmr.h"      //�͹��Ķ�ʱ��(��ʱ)
#include  "isr.h"
#include  "PIT.h"
#include  "D_SCCB.h"
#include  "isr.h"
#include  "dma.h"
#include  "ov7725.h"
#include  "pca9685.h"
#include  "iic.h"
#include  "servo.h"
#include  "AT24C1024.h"



//#undef  VECTOR_085
//#define VECTOR_085    PIT1_IRQHandler     //���¶���85���ж�ΪPIT1_IRQHandler�ж�
//extern void PIT1_IRQHandler();                 //PIT1 ��ʱ�жϷ�����

#endif  //__INCLUDE_H__
