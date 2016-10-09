#ifndef __SERVO_H
#define __SERVO_H

#include "include.h"


extern void Servo_Init(void);
extern void Servo_Run(u8 num,u32 pos);
extern u32 SERVO_Angle(u32 angle_temp,u32 angle_set,u32 T);
extern void SERVO_Control_unit(u8 servo_num,u32 angle_set,u32 T);
extern void SERVO_Control_all(void);
extern void SERVO_control(u8* servo_num,u32* position_now,u32* time_now);
extern void SERVO_run_all();

//extern void key_value();

#undef  VECTOR_084
#define VECTOR_084    PIT0_IRQHandler     //重新定义84号中断为PIT0_IRQHandler中断
extern void PIT0_IRQHandler();            //PIT0 定时中断服务函数


#endif
