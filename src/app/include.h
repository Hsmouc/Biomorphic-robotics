#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"

/*
 * Include 用户自定义的头文件
 */

#include  "gpio.h"       //IO口操作
#include  "default.h"
#include  "LED.h"        //灯
#include  "exti.h"       //EXTI外部GPIO中断
#include  "uart.h"       //串口
#include  "lptmr.h"      //低功耗定时器(延时)
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
//#define VECTOR_085    PIT1_IRQHandler     //重新定义85号中断为PIT1_IRQHandler中断
//extern void PIT1_IRQHandler();                 //PIT1 定时中断服务函数

#endif  //__INCLUDE_H__
