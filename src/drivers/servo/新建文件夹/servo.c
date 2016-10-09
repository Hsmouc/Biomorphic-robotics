#include "include.h"
#include "servo.h"
#include "pca9685.h"

u32 servo_position_now[32];    //当前位置
u8 servo_time_mark=0;   //全局变量
u32 servo_run_time;
u32 angle_now;

void Servo_Init(void)
{
    DisableInterrupts; 
    gpio_init(PORTB,18, GPO,0);
    gpio_init(PORTB,19, GPO,0);
    gpio_init(PORTB,20, GPO,0);
    gpio_init(PORTB,21, GPO,0);
    gpio_init(PORTB,22, GPO,0);
    gpio_init(PORTB,23, GPO,0);
    IIC_Init();
 
    pit_init_ms(PIT0, 10);                          //初始化PIT0，定时时间为： 1ms
    
    EnableInterrupts;           //开总中断 
    
    PCA9685PWM_convert( );
    WritePCA9685REG_1(LEDDRV1,PCA9685_MODE1,0x20);
    
    WritePCA9685REG_1(LEDDRV1,PCA9685_MODE1,0x10);
    WritePCA9685REG_1(LEDDRV1,PCA9685_PRE_SCALE,0x79);
    
    WritePCA9685REG_1(LEDDRV1,PCA9685_MODE1,0x20);
    
    angle_now=0;
    for(u8 i=0;i<32;i++)
    {
        servo_position_now[i]=0;
    }

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
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED6_ON_L,0x00);
             WritePCA9685REG_1(LEDDRV1,PCA9685_LED6_ON_H,0x00);
             
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
        default: break; 
    }

}

//舵机实时位置
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
//            LED_turn(LED4);
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

//输入舵机编号 位置  执行时间
void SERVO_control(u8* servo_num,u32* position_now,u32* time_now)
{
    if(servo_num[0]==1)
    {       
            servo_position_now[0] = SERVO_Angle(servo_position_now[0],position_now[0],time_now[0]);
            Servo_Run(0,servo_position_now[0]);
    }
    if(servo_num[1]==2)
    {
            servo_position_now[1] = SERVO_Angle(servo_position_now[1],position_now[1],time_now[1]);
            Servo_Run(1,servo_position_now[1]);
    }
    if(servo_num[2]==3)
    {
            servo_position_now[2] = SERVO_Angle(servo_position_now[2],position_now[2],time_now[2]);
            Servo_Run(2,servo_position_now[2]);
    }
    if(servo_num[3]==4)
    {
            servo_position_now[3] = SERVO_Angle(servo_position_now[3],position_now[3],time_now[3]);
            Servo_Run(3,servo_position_now[3]);
    }
    if(servo_num[4]==5)
    {
            servo_position_now[4] = SERVO_Angle(servo_position_now[4],position_now[4],time_now[4]);
            Servo_Run(4,servo_position_now[4]);
    }
    if(servo_num[5]==6)
    {
            servo_position_now[5] = SERVO_Angle(servo_position_now[5],position_now[5],time_now[5]);
            Servo_Run(5,servo_position_now[5]);
    }
    if(servo_num[6]==7)
    {
            servo_position_now[6] = SERVO_Angle(servo_position_now[6],position_now[6],time_now[6]);
            Servo_Run(6,servo_position_now[6]);
    }
    if(servo_num[7]==8)
    {
            servo_position_now[7] = SERVO_Angle(servo_position_now[7],position_now[7],time_now[7]);
            Servo_Run(7,servo_position_now[7]);
    }
    if(servo_num[8]==9)
    {
            servo_position_now[8] = SERVO_Angle(servo_position_now[8],position_now[8],time_now[8]);
            Servo_Run(8,servo_position_now[8]);
    }
    if(servo_num[9]==10)
    {
            servo_position_now[9] = SERVO_Angle(servo_position_now[9],position_now[9],time_now[9]);
            Servo_Run(9,servo_position_now[9]);
    }
    if(servo_num[10]==11)
    {
            servo_position_now[10] = SERVO_Angle(servo_position_now[10],position_now[10],time_now[10]);
            Servo_Run(10,servo_position_now[10]);
    }
    if(servo_num[11]==12)
    {
            servo_position_now[11] = SERVO_Angle(servo_position_now[11],position_now[11],time_now[11]);
            Servo_Run(11,servo_position_now[11]);
    }
    if(servo_num[12]==13)
    {
            servo_position_now[12] = SERVO_Angle(servo_position_now[12],position_now[12],time_now[12]);
            Servo_Run(12,servo_position_now[12]);
    }
    if(servo_num[13]==14)
    {
            servo_position_now[13] = SERVO_Angle(servo_position_now[13],position_now[13],time_now[13]);
            Servo_Run(13,servo_position_now[13]);
    }
    if(servo_num[14]==15)
    {
            servo_position_now[14] = SERVO_Angle(servo_position_now[14],position_now[14],time_now[14]);
            Servo_Run(14,servo_position_now[14]);
    }
    if(servo_num[15]==16)
    {
            servo_position_now[15] = SERVO_Angle(servo_position_now[15],position_now[15],time_now[15]);
            Servo_Run(15,servo_position_now[15]);
    }

}




