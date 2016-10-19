//���ֳ������ú궨�壬ѧ��C��C++�����������ڣ�
#ifndef _DEFAULT
#define _DEFAULT

#include "common.h"
#include <stdio.h>
#include <math.h>

//#define EnableInterrupts asm("CPSIE i");
//#define DisableInterrupts asm("CPSID i");

#define _ADD_SPEED 10
#define _OPEN_TIME 60

#define VS_NUMBER 88  //���ж�IRQ��
#define HS_NUMBER 72 //���ж�IRQ��
#define KNOB_NUMBER 90 //��ת���������ⲿ�ж�IRQ��
#define _SECTOR_NUMBER 200


//ͼ��ɼ�
#define _MAXROW      48    //�ɼ�������
#define _MAXCOL      128   //������ÿ�е�Ԫ�ظ���

#define _CENTER      64    //ͼ�������λ��
#define _DELAY       290  //�ȴ���������ʱ����Ҫ����

//���
#define _MID      2950     //�������λ�ã���Ҫ����
#define _MAXLEFT  3350     //�������λ��
#define _MAXRIGHT 2550     //����Ҽ���λ��
#define steering  FTM1_C1V //���ת������ռ�ձȿ��ƼĴ���
#define _MAXTURN  50       //70 һ�ο������Ķ��ת��

//���
#define _MAXMOTOR 4000     //���ת���������ռ�ձ�
#define _BANGBANG 300      //BANGBANG���Ʋ���
#define _FORWARD 0
#define _BACK 1

//��ȡ����
#define _MAX_BEGIN_MISS 30    //������һ����һ��������ȡ������һ������ʱ��������̵���ȡʧ������
//#define _MAX_BEGIN_ERROR 10  //��ȡ���ı�����������һ����һ�����ĵ��������ƫ�������
#define _MAX_CENTER_ERROR 10 //8 ����������ȡ�����������ĵ��������ƫ�������
#define _MAX_MISS 2         //������ȡʧ���������������ֵ��������
#define _MIN_WIDTH 10      //13 �����̵���խ�������
#define _MIN_ROW 5           //һ��ͼ��������ȡ�����вŲ���Ϊ��ä����������
#define _MAXBLIND 3          //����3��ä����pre_center��Ϊ64

#define _START_WIDTH 30
#define _START_TIMES 3

//���Խ׶θ�״̬
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

extern unsigned long int T;   //ȫ��ʱ�������ÿ1/120��ñ�����1
extern unsigned long int T_start; //С������ʱ��ʱ��
extern unsigned short int T_stop;
extern unsigned long int T_end;
extern unsigned char status;  //����ǰС���ĵ���״̬��0Ϊ��Ļ��ʾ����ͷ�ɼ�����
                           //1Ϊ���������ֵ��2Ϊ����PID������3Ϊ�趨�����ٶȣ�4����
extern unsigned char status_knob;   //��ť״̬��0Ϊδ��ת��1Ϊ��ť1��ת��2Ϊ��ť1��ת��3Ϊ��ť2��ת��4Ϊ��ť2��ת
                                 //����ת����Ե�
extern unsigned char status_button;   //��ť����״̬����
extern char mystr[20];               //��Ļ��ʾ�ַ���������
extern char temp_status;              //���Խ׶�ĳ�������е�״̬��Ǳ���                                 
extern unsigned long int last_press_time;  //���һ�ΰ�����ϵͳʱ�䣬��������˲�
extern unsigned long int last_turn_time;   //���һ����ת��ť��ϵͳʱ�䣬��������˲�
extern short int value;                 //��Ļ��ʾ��ֵ�Ļ������
extern short int OutData[4];

//ȫ�ֱ�������
//�������
extern unsigned char flag_pid;
extern unsigned short int speed;
extern short int maxspeed;      //������٣������ظ�����
extern short int minspeed;      //������٣������ظ�����
extern short int target;        //Ŀ���ٶ�
extern char Kp_speed;          //PID���ٿ��Ƶı���ϵ��
extern char Ki_speed;           //PID���ٿ��ƵĻ���ϵ��
extern char Kd_speed;           //PID���ٿ��Ƶ�΢��ϵ��

//��Ƶ�ɼ�����
extern unsigned char image[_MAXROW][_MAXCOL];    //ͼ�����飬ÿ��Ԫ�ر�ʾ1���㣬1��ʾ�ף�0��ʾ��
extern unsigned char data_temp[_MAXROW/8][_MAXCOL]; //������Ļ��ʾ������
extern unsigned char row_globe;              //ȫ���б�Ǳ������������ж��вɼ�ͼ��
extern unsigned char col_globe;              //ȫ���б�Ǳ������������ж��вɼ�ͼ��
extern char over;                            //�вɼ���ɱ�־���飬1��ʾ���вɼ���ϣ�0��ʾδ�ɼ����
extern unsigned char skip_counter;           //���������������еļ�����
extern short int row;                              //�б��
extern short int col;                              //�б��
extern u32 i;                       //forѭ����������

//������ȡ����
extern short int left[_MAXROW];            //����������߽�λ��
extern short int right[_MAXROW];           //���������ұ߽�λ��
extern short int center[_MAXROW];          //������������λ��
extern short int pre_center;
extern short int d_center[_MAXROW-1];
extern short int dd_center;
extern unsigned char begin;          //ÿ����һ���ɹ���ȡ�����ĵ��к�
extern unsigned char end;            //ÿ�����һ���ɹ���ȡ�����ĵ��к�
extern unsigned char pre_end;
extern unsigned char counter_blind;  //��¼����ä���ļ�����
extern short int curve;                    //һ��ͼ���е�һ�����������һ�����ĵ�ƫ��

extern unsigned char flag_start;
extern unsigned char counter_start;

//����������
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