#include "include.h"
#include "servo.h"
#include "pca9685.h"

u32 servo_position_now[32];    //当前位置
u32 temp_position[32];
u8 servo_time_mark=0;   //全局变量
u8 time_mark[32];
u32 servo_run_time;
u32 servo_time[32];
u32 temp_time[32];
u8  temp_servo_num[32];
u32 angle_now;
u8 servo_init_mark;
u8 servo_run_over[32];
u8 servo_control_mark=1;

u8 k;

void Servo_Init(void)
{
    DisableInterrupts;          //关总中断
//    gpio_init(PORTB,18, GPO,0);  
//    gpio_init(PORTB,19, GPO,0);
//   gpio_init(PORTB,20, GPO,0);
//    gpio_init(PORTB,21, GPO,0);
//    gpio_init(PORTB,22, GPO,0);
//    gpio_init(PORTB,23, GPO,0);
    IIC_Init();
 
    pit_init_ms(PIT0, 1);                          //初始化PIT0
    
    EnableInterrupts;           //开总中断 
    
    PCA9685PWM_convert( );
    WritePCA9685REG_1(LEDDRV1,PCA9685_MODE1,0x20);    
    WritePCA9685REG_1(LEDDRV1,PCA9685_MODE1,0x10);
    WritePCA9685REG_1(LEDDRV1,PCA9685_PRE_SCALE,0x79);   
    WritePCA9685REG_1(LEDDRV1,PCA9685_MODE1,0x20);
    
    
    WritePCA9685REG_1(LEDDRV2,PCA9685_MODE1,0x20);   
    WritePCA9685REG_1(LEDDRV2,PCA9685_MODE1,0x10);
    WritePCA9685REG_1(LEDDRV2,PCA9685_PRE_SCALE,0x79);   
    WritePCA9685REG_1(LEDDRV2,PCA9685_MODE1,0x20);
    
    angle_now=0;
    for(u8 i=0;i<32;i++)
    {
        servo_position_now[i]=0;
    }

    servo_init_mark=1;
}

void Servo_Run(u8 num,u32 pos)
{
    u8 OFF_L=0;
    u8 OFF_H=0;
    
    OFF_L=(pos+102)%256;
    OFF_H=(pos+102)/256;
 
    
    switch(num)
    {
        case 0:
        {
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED0_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED0_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED0_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED0_OFF_H,OFF_H);       
        }break;
        case 1:
        {
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED1_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED1_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED1_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED1_OFF_H,OFF_H);       
        }break;    
        case 2:
        {
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED2_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED2_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED2_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED2_OFF_H,OFF_H);       
        }break;
        case 3:
        {
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED3_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED3_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED3_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED3_OFF_H,OFF_H);       
        }break;
        case 4:
        {
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED4_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED4_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED4_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED4_OFF_H,OFF_H);       
        }break;
        case 5:
        {
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED5_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED5_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED5_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED5_OFF_H,OFF_H);       
        }break;
        case 6:
        {
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED6_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED6_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED6_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED6_OFF_H,OFF_H);      
        }break;
        case 7:
        {
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED7_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED7_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED7_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED7_OFF_H,OFF_H);       
        }break;
        case 8:
        {
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED8_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED8_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED8_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED8_OFF_H,OFF_H);       
        }break;
        case 9:
        {
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED9_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED9_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED9_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED9_OFF_H,OFF_H);       
        }break;
        case 10:
        {
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED10_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED10_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED10_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED10_OFF_H,OFF_H);       
        }break;
        case 11:
        {
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED11_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED11_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED11_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED11_OFF_H,OFF_H);       
        }break;    
        case 12:
        {
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED12_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED12_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED12_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED12_OFF_H,OFF_H);       
        }break;
        case 13:
        {
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED13_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED13_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED13_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED13_OFF_H,OFF_H);       
        }break;
        case 14:
        {
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED14_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED14_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED14_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED14_OFF_H,OFF_H);       
        }break;
        case 15:
        {
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED15_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED15_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED15_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED15_OFF_H,OFF_H);       
        }break;
        case 16:
        {
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED0_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED0_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED0_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED0_OFF_H,OFF_H);       
        }break;
        case 17:
        {
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED1_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED1_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED1_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED1_OFF_H,OFF_H);       
        }break;    
        case 18:
        {
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED2_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED2_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED2_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED2_OFF_H,OFF_H);       
        }break;
        case 19:
        {
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED3_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED3_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED3_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED3_OFF_H,OFF_H);       
        }break;
        case 20:
        {
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED4_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED4_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED4_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED4_OFF_H,OFF_H);       
        }break;
        case 21:
        {
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED5_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED5_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED5_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED5_OFF_H,OFF_H);       
        }break;
        case 22:
        {
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED6_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED6_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED6_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED6_OFF_H,OFF_H);       
        }break;
        case 23:
        {
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED7_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED7_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED7_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED7_OFF_H,OFF_H);       
        }break;
        case 24:
        {
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED8_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED8_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED8_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED8_OFF_H,OFF_H);       
        }break;
        case 25:
        {
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED9_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED9_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED9_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED9_OFF_H,OFF_H);       
        }break;
        case 26:
        {
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED10_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED10_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED10_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED10_OFF_H,OFF_H);       
        }break;
        case 27:
        {
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED11_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED11_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED11_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED11_OFF_H,OFF_H);       
        }break;    
        case 28:
        {
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED12_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED12_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED12_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED12_OFF_H,OFF_H);       
        }break;
        case 29:
        {
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED13_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED13_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED13_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED13_OFF_H,OFF_H);       
        }break;
        case 30:
        {
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED14_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED14_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED14_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED14_OFF_H,OFF_H);       
        }break;
        case 31:
        {
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED15_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED15_ON_H,0x00);
             
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED15_OFF_L,OFF_L);
             WritePCA9685REG_1(LEDDRV2,PCA9685_LED15_OFF_H,OFF_H);       
        }break;
        default: break; 
    }

}

//单舵机实时位置
u32 SERVO_Angle(u32 angle_temp,u32 angle_set,u32 T)
{
    if(servo_time_mark)       
    {
        servo_time_mark=0;
        servo_run_time++;
    }  
    if(T!=0)
    { 
        if(servo_run_time>=T)
        {
            servo_run_time=0;
            if(angle_temp<angle_set)
            {
                angle_temp=angle_temp+1;

            }
            else if(angle_temp>angle_set)
            {
                angle_temp=angle_temp-1;
          
            }
            else
            {
                angle_temp=angle_set;
                
            } 

        }
    }
    else
    {
           angle_temp=angle_set; 
    }
    
    
    return angle_temp;
}

//单个舵机控制 输入舵机编号  设置位置  执行时间
void SERVO_Control_unit(u8 servo_num,u32 angle_set,u32 T)
{
    angle_now = SERVO_Angle(angle_now,angle_set,T);
    Servo_Run(servo_num,angle_now);
}
//所有舵机实时角度
void SERVO_Control_all(void)
{

    for(k=0;k<32;k++)
    {
          if(temp_time[k]!=0)    //如果舵机运行时间不为零
          {
              servo_time[k]=servo_time[k]+1;    //运行时间累加
              if(servo_time[k]>=temp_time[k])   //累加到舵机运行时间，舵机位置运行一步
              {
                  servo_time[k]=0;
                  if(servo_position_now[k]<temp_position[k])
                  {
                      servo_position_now[k]=servo_position_now[k]+1;
      
                  }
                  else if(servo_position_now[k]>temp_position[k])
                  {
                      servo_position_now[k]=servo_position_now[k]-1;                 
                  }
                  else
                  {
                      servo_position_now[k]=temp_position[k];
                      servo_run_over[k]=1;
                  } 
              }
          }
          else               //如果舵机运行时间为零  舵机位置直接等于目标位置
          {
                 servo_position_now[k]=temp_position[k];
                 servo_time[k]=0;
                 servo_run_over[k]=1;
          }         
    }
 
}

//输入舵机编号 位置  执行时间
void SERVO_control(u8* servo_num,u32* position_now,u32* time_now)
{

    u8 kk;
 
    for(kk=0;kk<32;kk++)
    {
        temp_position[kk] = position_now[kk];
        temp_servo_num[kk]=servo_num[kk]-1;
        if(servo_init_mark)     //没有此操作，上电时舵机都会回一次原点    
        {
            temp_time[kk] = 0;
            servo_position_now[kk]=300;
        }
        else
        {
            temp_time[kk] = time_now[kk];
        }
    }
    servo_init_mark=0;     //舵机上电标志清零
}
//将舵机转动与舵机值设置分开
void SERVO_run_all()
{
      u8 kk;
      for(kk=0;kk<32;kk++)
      {      
          Servo_Run(temp_servo_num[kk],servo_position_now[kk]);  
      }
}



void PIT0_IRQHandler(void)
{
   
    if(servo_control_mark)
        SERVO_Control_all();


    
    PIT_Flag_Clear(PIT0);       //清中断标志位
}




