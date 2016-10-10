/******************** ********************
 * 文件名       ：main.c
 * 描述         ：
 *
**********************************************************************************/
#include "common.h"
#include "include.h"
#include "math.h"

u8 interrupt_mark1=0;     //全局变量
u8 servo_init_mark;

u32 i = 0;
u32 j = 0;



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


///////////////////////////舵机动作////////////////////////
u8 servo_SQ;  //动作组编号
// 存放顺序 舵机号 1个字节 舵机位置 2个字节 运行时间 2个字节
// 每个舵机信息5个字节，一个动作组32个舵机信息
u8 servo_dat[5000]; 
//u8 servo_dat1[5000]; 
u8 servo_num;                    //动作组中舵机个数
u8 servo_group_num[100];        //动作组中动作个数
u32 servo_position_set[32];     //设置位置
u32 servo_time_set[32];    //动作运行时间 动作设置时间
u8  servo_num_set[32];        //舵机编号

void servo(u8 servo_mark)
{     
    //读取动作
    if(servo_mark==1)           //在线
    {

        for(i=0;i<servo_num;i++)
        {
            servo_num_set[i]=servo_dat[ 5*i ];
            servo_position_set[i]=(servo_dat[ 5*i + 1]*256 + servo_dat[ 5*i + 2 ])/5;
            servo_time_set[i]=(servo_dat[ 5*i + 3 ]*256 + servo_dat[ 5*i + 4])/100;
        } 
        
        for(i=servo_num;i<32;i++)        //清楚未选舵机数据防止错误
        {
            servo_num_set[i]=0;
            servo_position_set[i]=0;
            servo_time_set[i]=0;
        }
            
        SERVO_control(servo_num_set,servo_position_set,servo_time_set);    
    }
    else if(servo_mark==2)     //离线
    {
        for(j=0;j<servo_group_num[servo_SQ];j++)
        {
            for(i=0;i<32;i++)
            {
                servo_num_set[i]=servo_dat[ 160*j + 5*i ];
                servo_position_set[i]=(servo_dat[ 160*j + 5*i + 1]*256 + servo_dat[ 160*j + 5*i + 2 ])/5;
                servo_time_set[i]=(servo_dat[ 160*j + 5*i + 3 ]*256 + servo_dat[ 160*j + 5*i + 4])/100;
            }    
            SERVO_control(servo_num_set,servo_position_set,servo_time_set);
            DelayMs(500);
        }
    }
}


///////////////////////////姿态读取////////////////////////
u16  Pitch;  //俯仰
u16  Roll ;  //翻滚
u16  Yaw  ;  //偏航

//====定义了陀螺仪的偏差===========
#define Gx_offset -3.06
#define Gy_offset 1.01
#define Gz_offset -0.88

u16 ax,ay,az,gx,gy,gz;
float aax,aay,aaz,ggx,ggy,ggz;//存储量化后的数据
float Ax,Ay,Az;//单位 g(9.8m/s^2)
float Gx,Gy,Gz;//单位 °/s
float Angel_accX,Angel_accY,Angel_accZ;//存储加速度计算出的角度
long LastTime,NowTime,TimeSpan;//用来对角速度积分的
void MPU6050()
{
    gx = mpu6050_getdata('G','X');
    gy = mpu6050_getdata('G','Y');
    gz = mpu6050_getdata('G','Z');
    ax = mpu6050_getdata('A','X');
    ay = mpu6050_getdata('A','Y');
    az = mpu6050_getdata('A','Z');
    //对加速度进行量化，得出单位为g的加速度值
    Ax=ax/16384.00;
    Ay=ay/16384.00;
    Az=az/16384.00;
    //用加速度计算三个轴和水平面坐标系之间的夹角
    Angel_accX=atan(Ax/sqrt(Az*Az+Ay*Ay))*180/3.14;
    Angel_accY=atan(Ay/sqrt(Ax*Ax+Az*Az))*180/3.14;
    Angel_accZ=atan(Az/sqrt(Ax*Ax+Ay*Ay))*180/3.14;
    
    
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
              LED(LED0,LED_ON);
              if(uart_Rx_dat[1]==0x5b && uart_Rx_dat[2]==0x52&& uart_Rx_dat[receive_add-2]==0x5d&& uart_Rx_dat[receive_add-1]==0x25)
              {
                  LED(LED1,LED_ON);
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
                          uart_putchar(UART4,servo_group_num[servo_SQ]);
                          uart_sendN(UART4, servo_dat,servo_group_num[servo_SQ]*160);     //反馈处理后信息，调试用
 //                         uart_sendN(UART4, RUN_Offline_Ack,8);
                      }break;
                      case 0x0d:  //脱机停止动作组
                      {
                          Offline_RUN=0;
                          uart_sendN(UART4, STOP_Offline_Ack,8);
                      }break;
                      case 0x0e:   //获取图像信息
                      {
                          uart_sendN(UART4, Send_Image,9);
                      }break; 
                      case 0x0f:   //获取姿态信息
                      {
                          MPU6050();
                          Send_Attitude[4]=1;
                          Send_Attitude[5]=Angel_accX;
                          Send_Attitude[6]=(Angel_accX-Send_Attitude[5])*100;
                          Send_Attitude[7]=2;
                          Send_Attitude[8]=Angel_accY;
                          Send_Attitude[9]=(Angel_accY-Send_Attitude[8])*100;
                          Send_Attitude[10]=1;
                          Send_Attitude[11]=Angel_accZ;
                          Send_Attitude[12]=(Angel_accZ-Send_Attitude[11])*100;

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
            LED(LED0,LED_OFF);
            LED(LED1,LED_OFF);

      }
      
}



///////////////////////////无线通信////////////////////////
u8 status;	//用于判断接收/发送状态
u8 txbuf[32]={0x1f,2,3,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31};	 //发送缓冲
u8 rxbuf[32];
u8 Tx_Rx_mark;  
void NRF()
{
    if(Tx_Rx_mark==0)
    {
          /*发送模式*/
          NRF_TX_Mode();
          
          /*开始发送数据*/
          status=NRF_Tx_Dat(txbuf); 
//          uart_putchar(UART4,status);
          
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
//              uart_sendN(UART4, rxbuf,32);
//                uart_putchar(UART4,status);
          }
          else
          {
             
          }
    
    }
}




void  main(void)
{
  
    uart_init(UART4, 19200);                    //初始化串口4，波特率为19200 ,波特率太大，容易不稳定
    DisableInterrupts;          //关总中断
    uart_irq_EN(UART4);
    EnableInterrupts;           //开总中断 
    
    LED_init();
    Servo_Init();
    NRF_Init();
    AT24C1024_Init();
    MPU6050_Init();
    Tx_Rx_mark=0;
    Offline_RUN=0; 
    ONline_RUN=0;
    servo_group_num[0]=ReadAT24C1024_byte(0,0x00);
    ReadAT24C1024_flash(servo_dat,1,0x00,servo_group_num[0]*160);
    servo_init_mark=1;   //舵机上电标志   

    while(1)
    {
        NRF();
        uart_Rx();
        if(ONline_RUN==1)
        {
            servo(1);
        }
        if(Offline_RUN == 1)
        {
            
            servo(2); 

        }
        

        
    }
}
