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

volatile u16 Vnum=0;


///////////////////////////舵机动作////////////////////////
//每个动作组是由一个或多个动作组成，每个动作是由一个或多个舵机组成
u8 servo_SQ;  //动作组编号
// 存放顺序 舵机号 1个字节 舵机位置 2个字节 运行时间 2个字节
// 每个舵机信息5个字节，一个动作32个舵机信息
u8 servo_dat[5000]; 
u8 servo_num;                    //每个动作中舵机个数
u8 servo_group_num[100];        //动作组中动作个数
u32 servo_position_set[32];     //舵机设置位置
u32 servo_time_set[32];    //舵机运行时间 动作设置时间
u8  servo_num_set[32];        //舵机编号
u8 servo_mark=0;              //舵机在线离线运行标志
u8  servo_run_over[32];        //舵机达到设置位置标志
u8  servo_run_over_mark=1;     //舵机全部达到指定位置标志
u8 servo_control_mark=1; 

void servo()
{     
    //读取动作
    if(servo_mark==1)           //在线
    {
       
      
        //首先读取舵机当前值      
        for(i=0;i<32;i++)
        {
            servo_position_set[ i ] = servo_position_now[ i ];
        }
        //单个舵机调试时
         if(servo_num==1)        
         {
              for(i=0;i<32;i++)
              {            
                  if(i==(servo_dat[ 0 ]-1))
                  {
                      servo_num_set[ i ] = servo_dat[ 0 ];
                      servo_position_set[ i ] = (servo_dat[ 1 ]*256 + servo_dat[ 2 ])/5;
                      servo_time_set[ i ] = (servo_dat[ 3 ]*256 + servo_dat[ 4 ])/100;
                  }
              }  
         }
         else     //多个舵机
         {
              j=0;
              for(i=0;i<32;i++)
              {            
                  if(j<servo_num)
                  {
                      if(i==(servo_dat[ 5*j ]-1))
                      {
                          servo_num_set[ i ] = servo_dat[ 5*j ];
                          servo_position_set[ i ] = (servo_dat[ 5*j + 1]*256 + servo_dat[ 5*j + 2 ])/5;
                          servo_time_set[ i] = (servo_dat[ 5*j + 3 ]*256 + servo_dat[ 5*j + 4])/100;
                          j++;
                      }
                  }
              }
         }
        
        
        
        SERVO_control(servo_num_set,servo_position_set,servo_time_set);  
        SERVO_run_all();
    }
    else if(servo_mark==2)     //离线
    {

        j=0;
        while(j<servo_group_num[servo_SQ])
        {


            if(servo_run_over_mark)  //所有舵机到达设置位置后读取下一个动作值
            {
                                 
                for(i=0;i<32;i++)
                {
                    servo_run_over[i]=0;    
                }
                for(i=0;i<32;i++)
                {                       

                    if((servo_dat[ 160*j + 5*i ]-1)==i)
                    {
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
            for(i=0;i<32;i++)
            {
                servo_run_over_mark=servo_run_over_mark&&servo_run_over[i];    
            }
        }
      
 
    }
}


///////////////////////////姿态读取////////////////////////
int temp_Pitch;
int temp_Roll;
int temp_Yaw;


u16 Attitude[6];


void MPU6050()
{
      zitaishuju();
      temp_Pitch=Pitch;
      temp_Roll=Roll;
      temp_Yaw=Yaw;
      if(temp_Pitch<0)
      {
          Pitch=fabs(Pitch);
          Send_Attitude[4]=45;
      }
      else
      {
          Pitch=Pitch;
          Send_Attitude[4]=43;
      }
      
      if(Roll<0)
      {
          Roll=fabs(Roll);
          Send_Attitude[7]=45;
      }
      else
      {
          Roll=Roll;
          Send_Attitude[7]=43;
      }
      
      if(Yaw<0)
      {
          Yaw=fabs(Yaw);
          Send_Attitude[10]=45;
      }
      else
      {
          Yaw=Yaw;
          Send_Attitude[10]=43;
      }
      //提取整数部分和小数部分
      Attitude[0]=Pitch;
      Attitude[1]=(Pitch-Attitude[0])*100;
      Attitude[2]=Roll;
      Attitude[3]=(Roll-Attitude[2])*100;
      Attitude[4]=Yaw;
      Attitude[5]=(Yaw-Attitude[4])*100;
      
      
      Send_Attitude[5]=Attitude[0];
      Send_Attitude[6]=Attitude[1];

      Send_Attitude[8]=Attitude[2];
      Send_Attitude[9]=Attitude[3];

      Send_Attitude[11]=Attitude[4];
      Send_Attitude[12]=Attitude[5];      
    
    
}

///////////////////////////电压检测////////////////////////
u16 vol;
u8 vol_H,vol_L;

void Voltage()
{
      Read_ADC(ADC1, &vol);             //查询方式读取AD转换结果
      vol=(13200*vol)/((1<<16)-1);      //v=（ad转换后的数值/65535（十六位位精度））*参考电压
      
      vol_H = vol/1000;                 //整数部分
      vol_L = vol%1000/100;             //小数部分
      
      Send_Voltage[4] = vol_H;
      Send_Voltage[5] = vol_L;
/*      
      if(vol<4000)
      {
          DelayUs(500);
          PTC14_OUT=0;
          DelayUs(500);
          PTC14_OUT=1;
      }
*/      
}


////////////////////////超声波测距////////////////////////////////
u32 time_val;
u8 succeed_flag;
u8 fail_flag;
u16 wait_time;
u32 distance;
void Ultrasonic()
{
      if(fail_flag==0)
      {
          PTC1_OUT=1;
          DelayUs(30);
          PTC1_OUT=0;  
          wait_time=0;
          while(PTC2_IN==0&&wait_time<500)wait_time++;          
          if(PTC2_IN)
          {
              pit_time_start(PIT0);                 //开始计时
              fail_flag=1; 
          }
      }
      if(succeed_flag==1)
      {
          succeed_flag=0;
          fail_flag=0;
          distance=time_val*0.172;     //单位mm
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
           /*               
                          case 0x03:      //舵机控制
                          {
                              
                              servo_num = ( NRF_Rx_dat[NRF_receive_num-4]*256 + NRF_Rx_dat[NRF_receive_num-3]-4 )/11;
                              for(i=0;i<(NRF_receive_num-8)/11;i++)
                              {
                                  servo_dat[5*i]= NRF_Rx_dat[11*i+5];
                                  servo_dat[5*i+1]= NRF_Rx_dat[11*i+7];
                                  servo_dat[5*i+2]= NRF_Rx_dat[11*i+8];
                                  servo_dat[5*i+3]= NRF_Rx_dat[11*i+13];
                                  servo_dat[5*i+4]= NRF_Rx_dat[11*i+14];
                              }
                              ONline_RUN=1;
                              Offline_RUN=0;
                              txbuf[0]=8;
                              for(i=0;i<8;i++)
                              {
                                  txbuf[i+1]=Servo_dat_ACK[i];
                              }
      //                         uart_sendN(UART4, Servo_dat_ACK,8);
      //                        uart_putchar(UART4,NRF_Rx_dat[NRF_receive_num-3]);
                              uart_sendN(UART4, servo_dat,servo_num*5);           //反馈处理后信息，调试用
      
                          }break;
                          case 0x04:    //舵机控制
                          {
                              
                          }break;
                          case 0x05:    //电机控制
                          {
                              txbuf[0]=8;
                              for(i=0;i<8;i++)
                              {
                                  txbuf[i+1]=Motor_dat_ACK[i];
                              }
                              uart_sendN(UART4, Motor_dat_ACK,8);
                          }break;
                          case 0x06:    //在线运行动作组
                          {
                              servo_num = ( NRF_Rx_dat[NRF_receive_num-4]*256 + NRF_Rx_dat[NRF_receive_num-3]-4 )/11;
                              for(i=0;i<(NRF_receive_num-8)/11;i++)
                              {
                                  servo_dat[5*i]= NRF_Rx_dat[11*i+5];
                                  servo_dat[5*i+1]= NRF_Rx_dat[11*i+7];
                                  servo_dat[5*i+2]= NRF_Rx_dat[11*i+8];
                                  servo_dat[5*i+3]= NRF_Rx_dat[11*i+13];
                                  servo_dat[5*i+4]= NRF_Rx_dat[11*i+14];
                              }
                              ONline_RUN=1;
                              Offline_RUN=0;
                              txbuf[0]=8;
                              for(i=0;i<8;i++)
                              {
                                  txbuf[i+1]=RUN_Online_Ack[i];
                              }
                              uart_sendN(UART4, RUN_Online_Ack,8);
                          }break;
                          case 0x07:   //在线循环运行动作组
                          {
                              servo_num = ( NRF_Rx_dat[NRF_receive_num-4]*256 + NRF_Rx_dat[NRF_receive_num-3]-4 )/11;
                              for(i=0;i<(NRF_receive_num-8)/11;i++)
                              {
                                  servo_dat[5*i]= NRF_Rx_dat[11*i+5];
                                  servo_dat[5*i+1]= NRF_Rx_dat[11*i+7];
                                  servo_dat[5*i+2]= NRF_Rx_dat[11*i+8];
                                  servo_dat[5*i+3]= NRF_Rx_dat[11*i+13];
                                  servo_dat[5*i+4]= NRF_Rx_dat[11*i+14];
                              }
                              ONline_RUN=1;
                              Offline_RUN=0;
                              txbuf[0]=8;
                              for(i=0;i<8;i++)
                              {
                                  txbuf[i+1]=RUN_Online_loop_Ack[i];
                              }
                              uart_sendN(UART4, RUN_Online_loop_Ack,8);
                          }break;
                          case 0x08:  //在线停止运行动作组
                          {
                              
                          }break;
                          case 0x09:  //EEPROM初始化
                          {
      
                          }break;
                          case 0x0a:  //下载动作组
                          {
                              ONline_RUN=0;                          
                              Offline_RUN=0;
                              servo_SQ = NRF_Rx_dat[6];
                              servo_group_num[servo_SQ]=(NRF_receive_num-11)/(11*32);
                              for(i=0;i<(NRF_receive_num-11)/11;i++)
                              {
                                  servo_dat[5*i]= NRF_Rx_dat[11*i+8];
                                  servo_dat[5*i+1]= NRF_Rx_dat[11*i+10];
                                  servo_dat[5*i+2]= NRF_Rx_dat[11*i+11];
                                  servo_dat[5*i+3]= NRF_Rx_dat[11*i+16];
                                  servo_dat[5*i+4]= NRF_Rx_dat[11*i+17];
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
                              txbuf[0]=8;
                              for(i=0;i<8;i++)
                              {
                                  txbuf[i+1]=EEPROM_Download_Ack[i];
                              }
      //                          uart_sendN(UART4, EEPROM_Download_Ack,8);
                              uart_sendN(UART4, servo_dat,servo_group_num[servo_SQ]*160);   //反馈处理后信息，调试用
                          }break;  
                          case 0x0b:  //擦除动作组
                          {
                              txbuf[0]=8;
                              for(i=0;i<8;i++)
                              {
                                  txbuf[i+1]=EEPROM_Erase_Ack[i];
                              }
                              uart_sendN(UART4, EEPROM_Erase_Ack,8);
                          }break;
              */
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
 //                             uart_putchar(UART4,servo_group_num[servo_SQ]);
//                              uart_sendN(UART4, servo_dat,servo_group_num[servo_SQ]*160);     //反馈处理后信息，调试用
                              RUN_Offline_Ack[4]= servo_SQ;
                              txbuf[0]=8;
                              for(i=0;i<8;i++)
                              {
                                  txbuf[i+1]=RUN_Offline_Ack[i];
                              }
      //                         uart_sendN(UART4, RUN_Offline_Ack,8);
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

///////////////////////////wifi通信////////////////////////
u8 wifi_receive;
u8 wifi_read_num=1;

void wifi()
{
    if( uart_query(UART1)!=0 )
    {
          wifi_receive=uart_getchar(UART1);
//          uart_putchar(UART1,wifi_receive);
    }  
    switch(wifi_receive)
    {
    
        case 0x00:
        {

            servo_SQ=0;
            servo_group_num[servo_SQ]=ReadAT24C1024_byte(servo_SQ*5000,0x00);
            if(servo_group_num[servo_SQ]!=0)
            {            
                ReadAT24C1024_flash(servo_dat,servo_SQ*5000+1,0x00,servo_group_num[servo_SQ]*160); 
            }

            wifi_read_num=1;
        }break;
        case 0x01:
        {
            if(wifi_read_num==1)
            {
                servo_SQ=1;
                servo_group_num[servo_SQ]=ReadAT24C1024_byte(servo_SQ*5000,0x00);
                if(servo_group_num[servo_SQ]!=0)
                {            
                    ReadAT24C1024_flash(servo_dat,servo_SQ*5000+1,0x00,servo_group_num[servo_SQ]*160); 
                }
                wifi_read_num=2;
            } 
        }break;
        case 0x02:
        {
            if(wifi_read_num==1)
            {
                servo_SQ=2;
                servo_group_num[servo_SQ]=ReadAT24C1024_byte(servo_SQ*5000,0x00);
                if(servo_group_num[servo_SQ]!=0)
                {            
                    ReadAT24C1024_flash(servo_dat,servo_SQ*5000+1,0x00,servo_group_num[servo_SQ]*160); 
                }
                wifi_read_num=2;
            }
        }break;
        case 0x03:
        {
            if(wifi_read_num==1)
            {
                servo_SQ=3;
                servo_group_num[servo_SQ]=ReadAT24C1024_byte(servo_SQ*5000,0x00);
                if(servo_group_num[servo_SQ]!=0)
                {            
                    ReadAT24C1024_flash(servo_dat,servo_SQ*5000+1,0x00,servo_group_num[servo_SQ]*160); 
                }
                wifi_read_num=2;
            }
        }break;        
        case 0x04:
        {
            if(wifi_read_num==1)
            {
                servo_SQ=4;
                servo_group_num[servo_SQ]=ReadAT24C1024_byte(servo_SQ*5000,0x00);
                if(servo_group_num[servo_SQ]!=0)
                {            
                    ReadAT24C1024_flash(servo_dat,servo_SQ*5000+1,0x00,servo_group_num[servo_SQ]*160); 
                }
                wifi_read_num=2;
            } 
        }break;          
        
        
        default: break;
    }

    
}



//////////////////////////////////////////////////////////


void system_init()
{
    uart_init(UART1, 9600);     //初始化串口4，波特率为19200 
    uart_init(UART4, 19200);    //初始化串口4，波特率为19200 ,波特率太大，容易不稳定
    DisableInterrupts;          //关总中断
    uart_irq_EN(UART4);         //串口通信中断使能
    EnableInterrupts;           //开总中断 
    
    LED_init();                 //led初始化
    Servo_Init();               //舵机初始化
    NRF_Init();                 //无线通信初始化
    AT24C1024_Init();           //存储初始化
    MPU6050_Init();             //姿态传感器初始化
    DMP_init();                 
    ADC_Init(ADC1, 3, 20);     //初始化AD1位单通道20输入，转换结果16bit, 查询模式  电压检测
    
    pit_init_ms(PIT1, 100);      //初始化PIT1    
    
    Tx_Rx_mark=1;
    Offline_RUN=1; 
    ONline_RUN=0;
    system_run_mark=1;
    system_mark=0;             //系统开始标志
//    servo_group_num[0]=ReadAT24C1024_byte(0,0x00);
//    ReadAT24C1024_flash(servo_dat,1,0x00,servo_group_num[0]*160);
//    servo_init_mark=1;   //舵机上电标志  
    
    exti_init(PORTC, 18, rising_down);      //PORTC18 端口外部中断初始化 ，上升沿触发中断，内部下拉
    exti_init(PORTC, 8, falling_down);      //PORTC8 端口外部中断初始化 ，低电平触发中断，内部下拉
    exti_init(PORTE, 3, falling_down);      //PORTE4 端口外部中断初始化 ，低电平触发中断，内部下拉
    exti_init(PORTE, 4, falling_down);      //PORTE4 端口外部中断初始化 ，低电平触发中断，内部下拉
    exti_init(PORTE, 5, falling_down);      //PORTE5 端口外部中断初始化 ，低电平触发中断，内部下拉//
    exti_init(PORTE, 6, falling_down);      //PORTE6 端口外部中断初始化 ，低电平触发中断，内部下拉
        
    exti_init(PORTC, 2, falling_down);      //PORTC2 端口外部中断初始化 ，低电平触发中断，内部下拉
}











/*************************************************************************
*
*  函数名称：PIT1_IRQHandler
*  功能说明：PIT1 定时中断服务函数

*************************************************************************/
void PIT1_IRQHandler(void)
{
   
    Voltage();
    

    
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

    //红外中断
     if(PORTE_ISFR & (1 << 4))           //PTE 4 6触发中断
    {

        PORTE_ISFR  |= (1 << 4);        //写1清中断标志位

        //  以下为用户任务  
        system_run_mark=0;
//        servo_control_mark=0;
        hw_read_mark=1;
        //  以上为用户任务  
    }
  
    if(PORTE_ISFR & (1 << 6))           //PTE 4 6触发中断
    {
        PORTE_ISFR  |= (1 << 6);        //写1清中断标志位
        // 以下为用户任务  
        system_run_mark=0;
//        servo_control_mark=0;
        hw_read_mark=1;
        //  以上为用户任务  
    }
 
    
    
}