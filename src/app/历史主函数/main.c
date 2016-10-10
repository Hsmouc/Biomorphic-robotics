/******************** ********************
 * 文件名       ：main.c
 * 描述         ：UART串口查询接收实验
 *
**********************************************************************************/
#include "common.h"
#include "include.h"


u8 interrupt_mark1=0;     //全局变量

u8 str[20];
u8 str0[8]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
u32 i = 0;
u32 j = 0;

u8 status;	//用于判断接收/发送状态
u8 txbuf[32]={0x1f,2,3,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31};	 //发送缓冲
u8 rxbuf[32];
u8 Tx_Rx_mark=0;  

u32 servo_position_set[32];     //设置位置
u32 servo_time_set[32];    //动作运行时间 动作设置时间
u8  servo_num_set[32];        //舵机编号


void NRF()
{
    if(Tx_Rx_mark==0)
    {
          /*发送模式*/
          NRF_TX_Mode();
          
          /*开始发送数据*/
          status = NRF_Tx_Dat(txbuf,MAX_ONCE_TX_NUM);
          
          /*判断发送状态*/  
          if(status& TX_DS)
          {
                  LED(LED2,LED_ON);
                  LED(LED3,LED_OFF);
                  LED(LED4,LED_OFF);
                  Tx_Rx_mark=1;
          }
          else if(status & MAX_RT)
          {
                  LED(LED2,LED_OFF);
                  LED(LED3,LED_OFF);
                  LED(LED4,LED_ON);
          }
          else
          {
                  LED(LED2,LED_OFF);
                  LED(LED3,LED_OFF);
                  LED(LED4,LED_ON);
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
                  LED(LED2,LED_OFF);
                  LED(LED3,LED_ON);
                  LED(LED4,LED_OFF);
                  Tx_Rx_mark=0;
              }
              break;

              default:
              {
                  LED(LED2,LED_OFF);
                  LED(LED3,LED_OFF);
                  LED(LED4,LED_OFF);
              }
              break;  	
          }
          
          if(rxbuf[1]==2)
          {
              PTA19_OUT=1;
          }
          else
          {
              PTA19_OUT=0;               
          }
    
    }
}




void  main(void)
{
    
    uart_init(UART4, 19200);                    //初始化串口4，波特率为19200 ,波特率太大，容易不稳定
    LED_init();
    Servo_Init();
    NRF_Init();
    AT24C02_Init();
 /*   
    DisableInterrupts;                                //禁止总中断
    pit_init_ms(PIT0, 1000);                          //初始化PIT0，定时时间为： 1000ms
    EnableInterrupts;			              //开总中断   
 */   
    
//    uart_sendStr(UART4 , "\n\n\n接受数据，并进行发送:");

                
    ReadAT24C02_flash(str,0,10);
    while(1)
    {

            
        if(uart_pendstr(UART4, str)!=0)
        {
            uart_sendN(UART4, str,10);             
            WriteAT24C02_flash(str,0,10);         
        }
        
        if(interrupt_mark1)       
        {
            interrupt_mark1=0;
        }
      
        if(str[1]!=0x00)
        {

            LED(LED0,LED_ON);
            LED(LED1,LED_OFF);
            servo_num_set[0]=1;
            servo_position_set[0]=str[3]*256+str[4];
            servo_time_set[0]=str[5];
            
            servo_num_set[1]=2;
            servo_position_set[1]=str[7]*256+str[8];
            servo_time_set[1]=str[9]; 
            
            

 //           Servo_Run(str[2],str[3]*256+str[4]);   //0----500
            SERVO_control(servo_num_set,servo_position_set,servo_time_set);
 //           SERVO_Control_unit(str[2],str[3]*256+str[4],str[5]);
            
        }
        else
        {

            LED(LED0,LED_OFF);
            LED(LED1,LED_ON);

        }


 //       NRF();
//        SERVO_control(servo_num_set,servo_position_set,servo_time_set);
     
    }
}
