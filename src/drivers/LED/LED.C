/******************** 嵌入式开发工作室 ********************

 * 描述         ：LED、触摸开关、红外避障函数定义
 *

**********************************************************************************/	

#include "common.h"
#include "gpio.h"
#include  "LED.h"
#include  "gpio_cfg.h"


#define KEY()	    PTC18_IN

#define HW1()	    PTE4_IN
#define HW2()	    PTE5_IN
#define HW3()	    PTE6_IN

u8 mark=1;
/*************************************************************************
*
*  函数名称：LED_init
*  功能说明：初始化LED端口，设置IO口为输出方向
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-2   已测试
*  备    注：
*************************************************************************/
void LED_init(void)
{
    //LED
    gpio_init  (PORTC, 1, GPO, HIGH);
    gpio_init  (PORTC, 2, GPO, HIGH);          
    gpio_init  (PORTC, 3, GPO, HIGH);
    
    
    gpio_init  (PORTA, 4, GPO, LOW);
    gpio_init  (PORTA, 5, GPO, LOW);          
    gpio_init  (PORTC, 5, GPO, LOW);
    gpio_init  (PORTC, 6, GPO, LOW);
    gpio_init  (PORTA, 19, GPO, LOW); 
    gpio_init  (PORTE, 26, GPO, LOW); 
    
    //触摸开关
    gpio_init  (PORTC, 18, GPI, LOW);
    
    //红外
    gpio_init  (PORTE, 4, GPI, LOW);
    gpio_init  (PORTE, 5, GPI, LOW);
    gpio_init  (PORTE, 6, GPI, LOW);
    
}

/*************************************************************************
*
*  函数名称：LED_ST
*  功能说明：设置LED灯亮灭
*  参数说明：LEDn        LED端口（LED0、LED1、LED2、LED3）
*            status      LED状态（LED_ON、LED_OFF）
*  函数返回：无
*  修改时间：2012-2-2   已测试
*  备    注：
*************************************************************************/
void LED(LEDn ledn,LED_status status)
{
    switch(ledn)
    {
        case 1:
        {
            if(status)
                PTC1_OUT=0;
            else
                PTC1_OUT=1;
        }break;
        case 2:
        {
            if(status)
                PTC2_OUT=0;
            else
                PTC2_OUT=1;
        }break;
        case 3:
        {
            if(status)
                PTC3_OUT=0;
            else
                PTC3_OUT=1;
        }break;        
        case 4:
        {
            if(status)
                PTA4_OUT=1;
            else
                PTA4_OUT=0;
        }break;
        case 5:
        {
            if(status)
                PTA5_OUT=1;
            else
                PTA5_OUT=0;

        }break;
        case 6:
        {
            if(status)
                PTC5_OUT=1;
            else
                PTC5_OUT=0;            

        }break;
        case 7:
        {
                    if(status)
                PTC6_OUT=1;
            else
                PTC6_OUT=0;  

        }break;
        case 8:
        {
            if(status)
                PTA19_OUT=1;
            else
                PTA19_OUT=0;

        }break;
        case 9:
        {
            if(status)
                PTE26_OUT=1;
            else
                PTE26_OUT=0;
        }break;
        default: break;
    }
}

void  LED_turn(LEDn ledn)
{
    if(mark)
    {
        LED(ledn,LED_ON);
        mark=0;
    }
    else    
    {
        LED(ledn,LED_OFF);
        mark=1;
    }
}

/*************************************************************************
*
*  函数名称：water_lights
*  功能说明：四个LED轮流闪烁，流水灯
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-10   已测试
*  备    注：
*************************************************************************/
void water_lights(void)
{

    LEDn    n;
    LED_init();
    for(n=LED0;n<=LED4;n++)
    {
        LED(n,LED_ON);
        LED_DELAY_MS(100);
        LED(n,LED_OFF);
    }
}











