/******************** ********************
 * 文件名       ：main.c
 
 *

**********************************************************************************/

#include "common.h"
#include "include.h"


u8 uart_Rx_dat[10000];
u32 receive_add;    //串口接收数据长度  
u8 receive_true_mark=0;
u8 uart_receive_mark=0;



void main(void)
{
    uart_init(UART4, 19200);                    //初始化串口4，波特率为19200 ,波特率太大，容易不稳定
    DisableInterrupts;          //关总中断
    uart_irq_EN(UART4);
    EnableInterrupts;           //开总中断
  
  
    volatile u32 i;
    u8 status;	//用于判断接收/发送状态
    u8 txbuf[32]={0x1f,2,3,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,0};	 //发送缓冲
    u8 rxbuf[32];
    u8 Tx_Rx_mark=0;
    gpio_init  (PORTA, 5, GPO, LOW);
    gpio_init  (PORTC, 5, GPO, LOW);
    gpio_init  (PORTC, 6, GPO, LOW);
    gpio_init  (PORTA, 19, GPO, LOW);
    NRF_Init();
    DelayUs0(50); 
    while(1)
    {
          if(Tx_Rx_mark==0)
          {
		/*发送模式*/
                NRF_TX_Mode(); 
		/*开始发送数据*/
		status=NRF_Tx_Dat(txbuf); 
                uart_putchar(UART4,status);
		/*判断发送状态*/  
		if(status& TX_DS)
		{
                        PTA5_OUT=0;
                        PTC5_OUT=0;
                        PTC6_OUT=1;
                        Tx_Rx_mark=1;
		}
		else if(status & MAX_RT)
		{
			PTA5_OUT=0;
                        PTC5_OUT=1;
                        PTC6_OUT=0;
		}
		else
		{
			PTA5_OUT=1;
                        PTC5_OUT=0;
                        PTC6_OUT=0;
		}
          }
          else if(Tx_Rx_mark==1)
          {
                /*接收模式*/
                NRF_RX_Mode();
                /*等待接收数据*/
		status = NRF_Rx_Dat(rxbuf);
                
		/*判断接收状态*/
		switch(status)
		{
                    case RX_DR:
                    {					
                          PTA5_OUT=0;
                          PTC5_OUT=1;
                          PTC6_OUT=0;
                          Tx_Rx_mark=0;
                    }
                    break;
      
                    default:
                    {
                          PTA5_OUT=0;
                          PTC5_OUT=0;
                          PTC6_OUT=1;
                    }
                    break;  	
		}
                
                if(rxbuf[1]==0x01)
                {
                    PTA19_OUT=1;
                    uart_putchar(UART4,status);
                }
                else
                {
                    PTA19_OUT=0;               
                }
          
          }
    } 
}
