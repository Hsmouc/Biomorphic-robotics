
/******************** ********************
 * 文件名       ：isr.c
 * 描述         ：中断处理例程
 *

**********************************************************************************/

#include "common.h"
#include "include.h"


volatile u16 Vnum=0;

/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：PORTA_IRQHandler
*  功能说明：PORTA端口中断服务函数
*  参数说明：无
*  函数返回：无
*  修改时间：2012-1-25    已测试
*  备    注：引脚号需要自己初始化来清除
*************************************************************************/
void PORTC_IRQHandler()
{
    uint8  n = 0;    //引脚号
    uint32 flag = PORTC_ISFR;
    PORTC_ISFR  = ~0;                                   //清中断标志位

    n = 7;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
          //场中断需要判断是场结束还是场开始
          if(img_flag == IMG_START)                   //需要开始采集图像
          {
              img_flag = IMG_GATHER;                  //标记图像采集中
              disable_irq(89);
      
              PORTC_ISFR = 1 << 7;            //清空PCLK标志位
              DMA_DADDR(CAMERA_DMA_CH)=  (u32)IMG_BUFF;//马上恢复目的地址
              DMA_EN(CAMERA_DMA_CH);                  //使能通道CHn 硬件请求
              
          }
          else                                        //图像采集错误
          {
              disable_irq(89);                        //关闭PTC的中断
              img_flag = IMG_FAIL;                    //标记图像采集失败
          }
    }
  
}


void DMA0_IRQHandler()
{
    img_flag = IMG_FINISH ;
//    DMA_IRQ_DIS(CAMERA_DMA_CH);
    DMA_IRQ_CLEAN(CAMERA_DMA_CH);           //清除通道传输中断标志位
}


