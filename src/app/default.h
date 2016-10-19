//各种常量采用宏定义，学过C或C++不理解宏的请面壁！
#ifndef _DEFAULT
#define _DEFAULT

#include "common.h"
#include <stdio.h>
#include <math.h>

//#define EnableInterrupts asm("CPSIE i");
//#define DisableInterrupts asm("CPSID i");

#define _ADD_SPEED 10
#define _OPEN_TIME 60

#define VS_NUMBER 88  //场中断IRQ号
#define HS_NUMBER 72 //行中断IRQ号
#define KNOB_NUMBER 90 //旋转编码器接外部中断IRQ号
#define _SECTOR_NUMBER 200


//图像采集
#define _MAXROW      48    //采集的行数
#define _MAXCOL      128   //数组中每行的元素个数

#define _CENTER      64    //图像的中心位置
#define _DELAY       290  //等待消隐区延时，需要调试

//舵机
#define _MID      2950     //舵机中心位置，需要调试
#define _MAXLEFT  3350     //舵机左极限位置
#define _MAXRIGHT 2550     //舵机右极限位置
#define steering  FTM1_C1V //舵机转角脉冲占空比控制寄存器
#define _MAXTURN  50       //70 一次控制最大的舵机转角

//电机
#define _MAXMOTOR 4000     //电机转速脉冲最大占空比
#define _BANGBANG 300      //BANGBANG控制参数
#define _FORWARD 0
#define _BACK 1

//提取中心
#define _MAX_BEGIN_MISS 30    //根据上一场第一行中心提取本场第一行中心时，最多容忍的提取失败行数
//#define _MAX_BEGIN_ERROR 10  //提取出的本场中心与上一场第一行中心的最大容忍偏差，待调试
#define _MAX_CENTER_ERROR 10 //8 相邻两行提取出的赛道中心的最大容忍偏差，待调试
#define _MAX_MISS 2         //连续提取失败行数的最大容忍值，待调试
#define _MIN_WIDTH 10      //13 可容忍的最窄赛道宽度
#define _MIN_ROW 5           //一场图像至少提取多少行才不认为是盲区，待调试
#define _MAXBLIND 3          //连续3场盲区后将pre_center置为64

#define _START_WIDTH 30
#define _START_TIMES 3

//调试阶段各状态
#define _STAT_SHOW      0
#define _STAT_SHOW_EDGE 1
#define _STAT_STEER     2
#define _STAT_PID       3
#define _STAT_SET_SPEED 4
#define _STAT_GO        5
///////////////////////////////////////////////////////
#define SDA_OUT() GPIOC_PDDR|=2<<13
#define SDA_IN() GPIOC_PDDR&=~(2<<13)
#define SCL_OUT() GPIOC_PDDR|=2<<12

#define IIC_SDA(val) set_PORT(2,13,val)
#define IIC_SCL(val) set_PORT(2,12,val)
#define READ_SDA() read_PORT(2,13)
///////////////////////////////////////////////////////
#define SDA_OUT0() GPIOA_PDDR|=1<<17
#define SDA_IN0() GPIOA_PDDR&=~(1<<17)
#define SCL_OUT0() GPIOA_PDDR|=1<<14

#define IIC_SDA0(val) set_PORT(0,17,val)
#define IIC_SCL0(val) set_PORT(0,14,val)
#define READ_SDA0() read_PORT(0,17)
/////////////////////////////////////////////////////////
#define SDA_OUT1() GPIOB_PDDR|=1<<1
#define SDA_IN1() GPIOB_PDDR&=~(1<<1)
#define SCL_OUT1() GPIOB_PDDR|=1<<0


#define IIC_SDA1(val) set_PORT(1,1,val)
#define IIC_SCL1(val) set_PORT(1,0,val)
#define READ_SDA1() read_PORT(1,1)
/////////////////////////////////////////////////////////
#define SDA_OUT2() GPIOB_PDDR|=1<<10
#define SDA_IN2() GPIOB_PDDR&=~(1<<10)
#define SCL_OUT2() GPIOB_PDDR|=1<<9

#define IIC_SDA2(val) set_PORT(1,10,val)
#define IIC_SCL2(val) set_PORT(1,9,val)
#define READ_SDA2() read_PORT(1,10)
/////////////////////////////////////////////////////////

#define IIC_DelayUs(x)  DelayUs(x)
#define IIC_DelayMs(x)  DelayMs(x)

#define SLAVE_WRITE_ADDRESS 0x18
#define SLAVE_READ_ADDRESS 0x19

#define HXL 0x03
#define HXH 0x04
#define HYL 0x05
#define HYH 0x06
#define HZL 0x07
#define HZH 0x08

extern unsigned long int T;   //全局时间变量，每1/120秒该变量加1
extern unsigned long int T_start; //小车启动时的时间
extern unsigned short int T_stop;
extern unsigned long int T_end;
extern unsigned char status;  //启动前小车的调试状态，0为屏幕显示摄像头采集内容
                           //1为调整舵机中值，2为调整PID参数，3为设定运行速度，4启动
extern unsigned char status_knob;   //旋钮状态，0为未旋转，1为旋钮1正转，2为旋钮1反转，3为旋钮2正转，4为旋钮2反转
                                 //正反转是相对的
extern unsigned char status_button;   //旋钮按下状态变量
extern char mystr[20];               //屏幕显示字符缓存数组
extern char temp_status;              //调试阶段某个过程中的状态标记变量                                 
extern unsigned long int last_press_time;  //最后一次按键的系统时间，用于软件滤波
extern unsigned long int last_turn_time;   //最后一次旋转旋钮的系统时间，用于软件滤波
extern short int value;                 //屏幕显示数值的缓存变量
extern short int OutData[4];

//全局变量定义
//电机变量
extern unsigned char flag_pid;
extern unsigned short int speed;
extern short int maxspeed;      //最高限速（脉冲沿个数）
extern short int minspeed;      //最低限速（脉冲沿个数）
extern short int target;        //目标速度
extern char Kp_speed;          //PID车速控制的比例系数
extern char Ki_speed;           //PID车速控制的积分系数
extern char Kd_speed;           //PID车速控制的微分系数

//视频采集变量
extern unsigned char image[_MAXROW][_MAXCOL];    //图像数组，每个元素表示1个点，1表示白，0表示黑
extern unsigned char data_temp[_MAXROW/8][_MAXCOL]; //用于屏幕显示的数组
extern unsigned char row_globe;              //全局行标记变量，用于在中断中采集图像
extern unsigned char col_globe;              //全局列标记变量，用于在中断中采集图像
extern char over;                            //行采集完成标志数组，1表示本行采集完毕，0表示未采集完成
extern unsigned char skip_counter;           //用于跳过消隐区行的计数器
extern short int row;                              //行标记
extern short int col;                              //列标记
extern u32 i;                       //for循环计数变量

//中线提取变量
extern short int left[_MAXROW];            //各行赛道左边界位置
extern short int right[_MAXROW];           //各行赛道右边界位置
extern short int center[_MAXROW];          //各行赛道中心位置
extern short int pre_center;
extern short int d_center[_MAXROW-1];
extern short int dd_center;
extern unsigned char begin;          //每场第一个成功提取出中心的行号
extern unsigned char end;            //每场最后一个成功提取出中心的行号
extern unsigned char pre_end;
extern unsigned char counter_blind;  //记录连续盲区的计数器
extern short int curve;                    //一场图像中第一行中心与最后一行中心的偏差

extern unsigned char flag_start;
extern unsigned char counter_start;

//弯道计算变量
extern unsigned char Kp_angle;
extern unsigned char Kd_angle;
extern unsigned char Kp_curve;

extern unsigned char pre_Kp_angle;  
extern unsigned char pre_Kd_angle;
extern unsigned char pre_Kp_curve;

extern short int front;

extern short int result_target;
extern unsigned char Kp_target;
extern unsigned char Kp_dd_center_target;

void send_client(void);
void init_clock(void);
void init_GPIO(void);
void init_PDB(void);
void init_FTM(void);
void init_PIT(void);
void init_LPTMR(void);
void init_LCD(void);
void init_I2C(void);
void init_MPU9150(void);
void init_UART(void);

void set_PORT(unsigned char port, unsigned char num, unsigned char value);
unsigned char read_PORT(unsigned char port, unsigned char num);

void DelayUs(unsigned int us);

void waitms(unsigned long time);
void motor(short int pow);
unsigned char check_button(void);
void show_image(void);
void debug_mode(void);

unsigned char get_center(void);

void cal_angle(void);

void blind(void);

void cal_speed(void);
void flash_mem_save();
void flash_mem_read();

#endif