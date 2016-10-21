/******************** ********************
 * 文件名       ：isr.c
 * 描述         ：中断处理例程
 *

**********************************************************************************/



#include "common.h"
#include "include.h"
#include "math.h"




u8 interrupt_mark1=0;     //全局变量



u32 i = 0;
u32 j = 0;

u8 system_run_mark;  //舵机开始标志
u8 system_mark;   //系统开始标志
///////////////////////////////////////////////////

u8 Motor_dat[30];      // 电机信息
u8 Offline_RUN;
u8 ONline_RUN;
u8 servo_SQ;
/////////////////////////////下位机反馈数据///////////////////////////////
u8 Send_Voltage[9]       = {0x40,0x5b,0x53,0x01,0x00,0x00,0x06,0x5d,0x25};    // 发送电压
u8 Send_Ampere[9]        = {0x40,0x5b,0x53,0x02,0x00,0x00,0x06,0x5d,0x25};     // 发送电流
u8 Servo_dat_ACK[8]      = {0x40,0x5b,0x53,0x03,0x00,0x05,0x5d,0x25};       // 舵机信息反馈
u8 Motor_dat_ACK[8]      = {0x40,0x5b,0x53,0x05,0x00,0x05,0x5d,0x25};      // 电机信息反馈
u8 RUN_Online_Ack[8]     = {0x40,0x5b,0x53,0x06,0x00,0x05,0x5d,0x25};         // 在线运行反馈
u8 RUN_Online_loop_Ack[8]= {0x40,0x5b,0x53,0x07,0x00,0x05,0x5d,0x25};    // 在线循环运行反馈
u8 EEPROM_Download_Ack[8]= {0x40,0x5b,0x53,0x0a,0x00,0x05,0x5d,0x25};     // 下载动作组反馈
u8 EEPROM_Erase_Ack[8]   = {0x40,0x5b,0x53,0x0b,0x00,0x05,0x5d,0x25};       // 擦除动作组反馈
u8 RUN_Offline_Ack[8]    = {0x40,0x5b,0x53,0x0c,0x00,0x05,0x5d,0x25};        // 脱机运行反馈
u8 STOP_Offline_Ack[8]   = {0x40,0x5b,0x53,0x0d,0x00,0x05,0x5d,0x25};        // 脱机运行停止
u8 Send_Image[9]         = {0x40,0x5b,0x53,0x0e,0x00,0x00,0x06,0x5d,0x25};      // 发送图像
u8 Send_Attitude[16]     = {0x40,0x5b,0x53,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0d,0x5d,0x25};   // 发送姿态
u8 Send_ID[9]            = {0x40,0x5b,0x53,0x11,0x00,0x00,0x06,0x5d,0x25};         // 发送ID
//u8 MPU6050_Dat[9]        = {0x40,0x5b,0x53,0x11,0x00,0x00,0x06,0x5d,0x25}




///////////////////////////舵机动作////////////////////////
//每个动作组是由一个或多个动作组成，每个动作是由一个或多个舵机组成
// 存放顺序 舵机号 1个字节 舵机位置 2个字节 运行时间 2个字节
// 每个舵机信息5个字节，一个动作32个舵机信息
u8 servo_num;                    //每个动作中舵机个数
u8 servo_group_num[100];        //动作组中动作个数
u32 servo_position_set[32];     //舵机设置位置
u32 servo_time_set[32];    //舵机运行时间 动作设置时间
u8  servo_num_set[32];        //舵机编号
u8 servo_mark=0;              //舵机在线离线运行标志
u8  servo_run_over[32];        //舵机达到设置位置标志
u8  servo_run_over_mark=1;     //舵机全部达到指定位置标志
u8 servo_control_mark=1; 

void motionGet(u8 flag,u8 *servo_dat){ 
  system_run_mark=1;
  servo_control_mark=1;
  //read_mark=1;
    LED(LED1,LED_ON);
    LED(LED2,LED_OFF);    
    servo_SQ=flag;
    servo_group_num[servo_SQ]=ReadAT24C1024_byte(servo_SQ*5000,0x00);
    if(servo_group_num[servo_SQ]!=0){
      ReadAT24C1024_flash(servo_dat,servo_SQ*5000+1,0x00,servo_group_num[servo_SQ]*160); 
    }
  servo_run_over_mark=1;
}

void motionCtr(u8 *servo_dat){
  j=0;
  while(j<servo_group_num[servo_SQ]){
    if(servo_run_over_mark)  //所有舵机到达设置位置后读取下一个动作值
    {
      for(i=0;i<32;i++){
        servo_run_over[i]=0;    
      }
      for(i=0;i<32;i++){
        if((servo_dat[ 160*j + 5*i ]-1)==i){
          servo_num_set[i]=servo_dat[ 160*j + 5*i ];
          servo_position_set[i]=(servo_dat[ 160*j + 5*i + 1]*256 + servo_dat[ 160*j + 5*i + 2 ])/5;
          servo_time_set[i]=(servo_dat[ 160*j + 5*i + 3 ]*256 + servo_dat[ 160*j + 5*i + 4])/100;
        }
      } 
      SERVO_control(servo_num_set,servo_position_set,servo_time_set);
      j++;
     }
    servo_run_over_mark=1;
    SERVO_run_all();
    //检测舵机是否全部到达指定位置
    for(i=0;i<32;i++){
      servo_run_over_mark=servo_run_over_mark&&servo_run_over[i];    
    }
  } 
}





/////////////////////////串口接收数据/////////////////////////////////////
u8 uart_Rx_dat[10000];
u32 receive_add;    //串口接收数据长度  
u8 receive_true_mark=0;
u8 uart_receive_mark=0;
void uart_Rx()
{

      if(uart_receive_mark==1)
      {
          uart_receive_mark=0;       
          if(receive_add>=8)
          {
              LED(LED9,LED_ON);
              if(uart_Rx_dat[1]==0x5b && uart_Rx_dat[2]==0x52&& uart_Rx_dat[receive_add-2]==0x5d&& uart_Rx_dat[receive_add-1]==0x25)
              {
                  LED(LED4,LED_ON);
                  switch(uart_Rx_dat[3])
                  {
                      case 0x01:        //获取电压
                      {
                          uart_sendN(UART4, Send_Voltage,9);
    
                      }break;
                      case 0x02:       //获取电流   
                      {
                          uart_sendN(UART4, Send_Ampere,9);
       
                      }break;
                      case 0x03:      //舵机控制
                      {
                          
                          servo_num = ( uart_Rx_dat[receive_add-4]*256 + uart_Rx_dat[receive_add-3]-4 )/11;
                          for(i=0;i<(receive_add-8)/11;i++)
                          {
                              servo_dat[5*i]= uart_Rx_dat[11*i+5];
                              servo_dat[5*i+1]= uart_Rx_dat[11*i+7];
                              servo_dat[5*i+2]= uart_Rx_dat[11*i+8];
                              servo_dat[5*i+3]= uart_Rx_dat[11*i+13];
                              servo_dat[5*i+4]= uart_Rx_dat[11*i+14];
                          }
                          ONline_RUN=1;
                          Offline_RUN=0;
 //                         uart_sendN(UART4, Servo_dat_ACK,8);
                          uart_sendN(UART4, servo_dat,servo_num*5);           //反馈处理后信息，调试用

                      }break;
                      case 0x04:    //舵机控制
                      {
                          
                      }break;
                      case 0x05:    //电机控制
                      {
                          uart_sendN(UART4, Motor_dat_ACK,8);
                      }break;
                      case 0x06:    //在线运行动作组
                      {
                        
                          servo_num = ( uart_Rx_dat[receive_add-4]*256 + uart_Rx_dat[receive_add-3]-4 )/11;
                          for(i=0;i<(receive_add-8)/11;i++)
                          {
                              servo_dat[5*i]= uart_Rx_dat[11*i+5];
                              servo_dat[5*i+1]= uart_Rx_dat[11*i+7];
                              servo_dat[5*i+2]= uart_Rx_dat[11*i+8];
                              servo_dat[5*i+3]= uart_Rx_dat[11*i+13];
                              servo_dat[5*i+4]= uart_Rx_dat[11*i+14];
                          }
                          ONline_RUN=1;
                          Offline_RUN=0;
                          uart_sendN(UART4, RUN_Online_Ack,8);
                      }break;
                      case 0x07:   //在线循环运行动作组
                      {
                          servo_num = ( uart_Rx_dat[receive_add-4]*256 + uart_Rx_dat[receive_add-3]-4 )/11;
                          for(i=0;i<(receive_add-8)/11;i++)
                          {
                              servo_dat[5*i]= uart_Rx_dat[11*i+5];
                              servo_dat[5*i+1]= uart_Rx_dat[11*i+7];
                              servo_dat[5*i+2]= uart_Rx_dat[11*i+8];
                              servo_dat[5*i+3]= uart_Rx_dat[11*i+13];
                              servo_dat[5*i+4]= uart_Rx_dat[11*i+14];
                          }
                          ONline_RUN=1;
                          Offline_RUN=0;
                          uart_sendN(UART4, RUN_Online_loop_Ack,8);
                      }break;
                      case 0x08:  //在线停止运行动作组
                      {
                          uart_sendN(UART4, Motor_dat_ACK,8);
                      }break;
                      case 0x09:  //EEPROM初始化
                      {

                      }break;
                      case 0x0a:  //下载动作组
                      {
                          ONline_RUN=0;                          
                          Offline_RUN=0;
                          servo_SQ = uart_Rx_dat[6];
                          servo_group_num[servo_SQ]=(receive_add-11)/(11*32);
                          for(i=0;i<(receive_add-11)/11;i++)
                          {
                              servo_dat[5*i]= uart_Rx_dat[11*i+8];
                              servo_dat[5*i+1]= uart_Rx_dat[11*i+10];
                              servo_dat[5*i+2]= uart_Rx_dat[11*i+11];
                              servo_dat[5*i+3]= uart_Rx_dat[11*i+16];
                              servo_dat[5*i+4]= uart_Rx_dat[11*i+17];
                          }
                          if(servo_SQ<13)
                          {
                              WriteAT24C1024_byte(servo_SQ*5000,servo_group_num[servo_SQ],0x00);
                              DelayMs1(10);
                              WriteAT24C1024_flash(servo_dat,servo_SQ*5000+1,0x00,servo_group_num[servo_SQ]*160);
                          }
                          else
                          {
 
                              WriteAT24C1024_byte((servo_SQ-13)*5000,servo_group_num[servo_SQ],0x02);
                              DelayMs1(10);
                              WriteAT24C1024_flash(servo_dat,(servo_SQ-13)*5000+1,0x02,servo_group_num[servo_SQ]*160);                          
                          }
                      
//                          uart_sendN(UART4, EEPROM_Download_Ack,8);
                          uart_sendN(UART4, servo_dat,servo_group_num[servo_SQ]*160);   //反馈处理后信息，调试用
                      }break;  
                      case 0x0b:  //擦除动作组
                      {

                          uart_sendN(UART4, EEPROM_Erase_Ack,8);
                      }break;
                      case 0x0c:  //脱机运行动作组
                      {
                          ONline_RUN=0;                          
                          Offline_RUN=1;
                          servo_SQ = uart_Rx_dat[6];
                          if(servo_SQ<13)
                          {
                              servo_group_num[servo_SQ]=ReadAT24C1024_byte(servo_SQ*5000,0x00);
                              ReadAT24C1024_flash(servo_dat,servo_SQ*5000+1,0x00,servo_group_num[servo_SQ]*160);
                          }
                          else
                          {
                              servo_group_num[servo_SQ]=ReadAT24C1024_byte((servo_SQ-13)*5000,0x02);
                              ReadAT24C1024_flash(servo_dat,(servo_SQ-13)*5000+1,0x02,servo_group_num[servo_SQ]*160);                          
                          }
 //                         uart_putchar(UART4,servo_group_num[servo_SQ]);
                          uart_sendN(UART4, servo_dat,servo_group_num[servo_SQ]*160);     //反馈处理后信息，调试用
 //                         uart_sendN(UART4, RUN_Offline_Ack,8);
                      }break;
                      case 0x0d:  //脱机停止动作组
                      {
                          ONline_RUN=1;                          
                          Offline_RUN=0;
                          servo_init_mark=1;     //舵机上电标志清零
                          uart_sendN(UART4, STOP_Offline_Ack,8);
                      }break;
                      case 0x0e:   //获取图像信息
                      {
                          uart_sendN(UART4, Send_Image,9);
                      }break; 
                      case 0x0f:   //获取姿态信息
                      {

                          uart_sendN(UART4, Send_Attitude,16);
                      }break;
                      case 0x10:   //获取压力信息
                      {

                      }break;
                      case 0x11:   //获取ID信息
                      {
                          uart_sendN(UART4, Send_ID,9);
                      }break;
                      default: break; 
                  }
                  
              }
          }
      }
      else
      {
            LED(LED9,LED_OFF);
            LED(LED4,LED_OFF);

      }
      
}



///////////////////////////无线通信////////////////////////
u8 status;	//用于判断接收/发送状态
u8 txbuf[32]={0x1f,2,3,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31};	 //发送缓冲
u8 rxbuf[32];
u8 Tx_Rx_mark; 
u8 NRF_Rx_dat[10000];
u8 NRF_receive_mark=0;
u32 rx_i,NRF_receive_num;
u8 rx_last_num;

void NRF()
{
    if(Tx_Rx_mark==0)
    {
          /*发送模式*/
          NRF_TX_Mode();
          
          /*开始发送数据*/
//          status=NRF_Tx_Dat(txbuf); 
          NRF_Tx_Dat(txbuf);
//          uart_putchar(UART4,status);
          
          /*判断发送状态*/  
          if(status& TX_DS)
          {
                  LED(LED5,LED_ON);
                  LED(LED8,LED_OFF);
//                  Tx_Rx_mark=1;
          }
          else if(status & MAX_RT)
          {
                  LED(LED5,LED_OFF);
                  LED(LED8,LED_ON);
          }
          else
          {
                  LED(LED5,LED_OFF);
                  LED(LED8,LED_ON);
          }
    }
    else if(Tx_Rx_mark==1)
    {
          /*接收模式*/
          NRF_RX_Mode();
          /*等待接收数据*/
//          status = NRF_Rx_Dat(rxbuf);
          /*判断接收状态*/
          switch(status)
          {
              case RX_DR:
              {					
                  LED(LED6,LED_ON);
                  LED(LED7,LED_OFF);
                  if(rxbuf[1]==0x40&&rxbuf[2]==0x5B&&rxbuf[3]==0x52)
                  {
                      rx_i=0;
                  }
                  
                  for(i=2;i<32;i++)
                  {
                      if(rxbuf[i-1]==0x5D&&rxbuf[i]==0x25)
                      {
                          NRF_receive_mark=1;
                          Tx_Rx_mark=0;          //正确接收后切换到发送模式
                          rx_last_num=i;
                          
                          break;
                      }
                  }
                  
                  if(NRF_receive_mark==0)
                  {
                      for(i=0;i<31;i++)
                      {
                          NRF_Rx_dat[rx_i*31+i]=rxbuf[i+1];
                      }
                  }
                  else
                  {
                      for(i=0;i<rx_last_num;i++)
                      {
                          NRF_Rx_dat[rx_i*31+i]=rxbuf[i+1];
                      }
                      NRF_receive_num=rx_i*31+rx_last_num;
                  }
                  
                  if(NRF_receive_mark)
                  {
                      switch(NRF_Rx_dat[3])
                      {
                          case 0x01:        //获取电压
                          {
                              txbuf[0]=9;
                              for(i=0;i<9;i++)
                              {
                                  txbuf[i+1]=Send_Voltage[i];
                              }
                              uart_sendN(UART4, Send_Voltage,9);
        
                          }break;
                          case 0x02:       //获取电流   
                          {
                              txbuf[0]=9;
                              for(i=0;i<9;i++)
                              {
                                  txbuf[i+1]=Send_Ampere[i];
                              }
                              uart_sendN(UART4, Send_Ampere,9);
           
                          }break;
          
                          case 0x0c:  //脱机运行动作组
                          {
                              ONline_RUN=0;                          
                              Offline_RUN=1;
                              servo_SQ = NRF_Rx_dat[6];
                              if(servo_SQ<13)
                              {
                                  servo_group_num[servo_SQ]=ReadAT24C1024_byte(servo_SQ*5000,0x00);
                                  ReadAT24C1024_flash(servo_dat,servo_SQ*5000+1,0x00,servo_group_num[servo_SQ]*160);
                              }
                              else
                              {
                                  servo_group_num[servo_SQ]=ReadAT24C1024_byte((servo_SQ-13)*5000,0x02);
                                  ReadAT24C1024_flash(servo_dat,(servo_SQ-13)*5000+1,0x02,servo_group_num[servo_SQ]*160);                          
                              }
                              RUN_Offline_Ack[4]= servo_SQ;
                              txbuf[0]=8;
                              for(i=0;i<8;i++)
                              {
                                  txbuf[i+1]=RUN_Offline_Ack[i];
                              }
                          }break;
                          case 0x0d:  //脱机停止动作组
                          {
                              ONline_RUN=1;                          
                              Offline_RUN=0;
                              txbuf[0]=8;
                              for(i=0;i<8;i++)
                              {
                                  txbuf[i+1]=STOP_Offline_Ack[i];
                              }
                              uart_sendN(UART4, STOP_Offline_Ack,8);
                          }break;
                          case 0x0e:   //获取图像信息
                          {
                              txbuf[0]=9;
                              for(i=0;i<9;i++)
                              {
                                  txbuf[i+1]=Send_Image[i];
                              }
                              uart_sendN(UART4, Send_Image,9);
                          }break; 
                          case 0x0f:   //获取姿态信息
                          {
                              
                              txbuf[0]=16;
                              for(i=0;i<16;i++)
                              {
                                  txbuf[i+1]=Send_Attitude[i];
                              }
                              uart_sendN(UART4, Send_Attitude,16);
                          }break;
                          case 0x10:   //获取压力信息
                          {
      
                          }break;
                          case 0x11:   //获取ID信息
                          {
                              txbuf[0]=9;
                              for(i=0;i<9;i++)
                              {
                                  txbuf[i+1]=Send_ID[i];
                              }
                              uart_sendN(UART4, Send_ID,9);
                          }break;

                          case 0x12:   //获取姿态信息
                          {

                              txbuf[0]=16;
                              for(i=0;i<16;i++)
                              {
                                  txbuf[i+1]=Send_Attitude[i];
                              }
                              uart_sendN(UART4, Send_Attitude,16);
                          }break;

                          default: break; 
                      }
                  
                  }

                  rx_i++;
                  
              }
              break;

              default:
              {
                  LED(LED6,LED_OFF);
                  LED(LED7,LED_ON);
              }
              break;  	
          }
          

    
    }
}



//////////////////////////////////////////////////////////


void system_init()
{
    uart_init(UART4, 115200);                    //初始化串口4，波特率为19200 ,波特率太大，容易不稳定
    DisableInterrupts;          //关总中断
    uart_irq_EN(UART4);
    EnableInterrupts;           //开总中断 
    
    LED_init();
    Servo_Init();
    NRF_Init();
    AT24C1024_Init();
    //MPU6050_Init();
    
    
    pit_init_ms(PIT1, 1);                          //初始化PIT1    
    
    //Tx_Rx_mark=1;
    //Offline_RUN=1; 
    //ONline_RUN=0;
    //system_run_mark=1;
    
    exti_init(PORTC, 18, rising_down);      //PORTC18 端口外部中断初始化 ，上升沿触发中断，内部下拉
    exti_init(PORTC, 8, falling_down);       //PORTC8 端口外部中断初始化 ，低电平触发中断，内部下拉
    exti_init(PORTE, 4, falling_down);      //PORTE4 端口外部中断初始化 ，低电平触发中断，内部下拉
    //exti_init(PORTE, 5, falling_down);      //PORTE5 端口外部中断初始化 ，低电平触发中断，内部下拉//
    exti_init(PORTE, 6, falling_down);      //PORTE6 端口外部中断初始化 ，低电平触发中断，内部下拉
        
}











/*************************************************************************
*
*  函数名称：PIT1_IRQHandler
*  功能说明：PIT1 定时中断服务函数

*************************************************************************/
void PIT1_IRQHandler(void)
{
   

    
    PIT_Flag_Clear(PIT1);       //清中断标志位
}



/*************************************************************************
*
*  函数名称：PORTn_IRQHandler
*  功能说明：PORTn端口中断服务函数
*  参数说明：无
*  函数返回：无
*  备    注：引脚号需要自己初始化来清除
*************************************************************************/
u8 read_mark=0;   //读取动作组标志
u8 read_num=0;
u8 hw_read_mark=0;
u8 hw_read=0;  

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

void PORTE_IRQHandler()
{

    if(PORTE_ISFR & (1 << 4) || PORTE_ISFR & (1 << 6))           //PTE4 5 6触发中断
    {
        read_mark = 1;
        PORTE_ISFR  |= (1 << 4);        //写1清中断标志位      //写1清中断标志位
        PORTE_ISFR  |= (1 << 6);        //写1清中断标志位
        /*  以下为用户任务  */
        //LED(LED3,LED_ON);
        //system_run_mark=0;
        //servo_control_mark=0;
        hw_read_mark=1;
        /*  以上为用户任务  */
       
    }

}
