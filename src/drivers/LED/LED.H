#ifndef __LED_H__
#define __LED_H__




typedef enum LEDn
{
  LED0,              //对应的引脚号
  LED1,
  LED2,
  LED3,
  LED4,
  LED5,
  LED6,
  LED7,
  LED8,
  LED9
    
}LEDn;



//LED亮灭状态宏定义
typedef enum LED_status
{
  LED_OFF=0,         //灯亮(对应低电平)
  LED_ON=1           //灯暗(对应高电平)
}LED_status;



extern void    LED_init(void);                                                 // LED初始化    函数
extern void    LED(LEDn ledn,LED_status status);
extern void    LED_turn(LEDn ledn);
extern void    water_lights(void);                                               // 流水灯

//定义延时,方便流水灯的延时
#include "delay.h"
#define   LED_DELAY           delay()
#define   LED_DELAY_MS(x)     delayms(x)

#endif