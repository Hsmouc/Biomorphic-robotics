#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"

/*
 * Include 用户自定义的头文件
 */

#include  "gpio.h"       //IO口操作
#include  "LED.h"        //灯
#include  "iic.h"
#include  "pca9685.h"
#include  "servo.h"
#include  "exti.h"       //EXTI外部GPIO中断
#include  "uart.h"       //串口
#include  "lptmr.h"      //低功耗定时器(延时)
#include  "spi.h"
#include  "NRF24L01.h"
#include  "PIT.h"
#include  "isr.h"
#include "AT24C1024.h"
#include  "iic2.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "getattitude.h"
#include  "adc.h"
#include "D_SCCB.h"
#include "ov7725.h"
#include "dma.h"




#endif  //__INCLUDE_H__
