#include "common.h"
#include "include.h"
#include "gpio.h"
#include "exti.h"
#include "dma.h"
#include "D_SCCB.h"
#include "OV7725.h"
#include "OV7725_REG.h"

#define OV7725_Delay_ms(time)  DELAY_MS(time)

#define OV7725_ID           0x21

                                  //����洢����ͼ�������

u8 volatile *	IMG_BUFF;
volatile u8	    img_flag = IMG_FINISH;		//ͼ��״̬

static u8 Ov7725_reg_Init(void);





void Ov7725_exti_Init()
{
    //IMG_BUFF = imgbuff;
	//DMAͨ��0��ʼ����PTC0�����ش���DMA���䣬Դ��ַΪPTD_BYTE0_IN��Ŀ�ĵ�ַΪ��IMG_BUFF ��ÿ�δ���1Byte������320�κ�ֹͣ���䣬����Ŀ�ĵ�ַ����	
	DMA_PORTx2BUFF_Init(CAMERA_DMA_CH, (void *)&PTD_BYTE0_IN, (void *)IMG_BUFF, PTC0, DMA_BYTE1, CAMERA_DMA_NUM , DMA_rising_down_keepon);
	
//	DMA_DIS(DMA_CH0);		//�ȹر� DMA
//	DMA_IRQ_DIS(DMA_CH0);	//����Ҫʹ��DMA�ж�,�����ж�������DMA����

        DMA_DIS(DMA_CH0);
        disable_irq(89);                        //�ر�PTA���ж�
        DMA_IRQ_CLEAN(DMA_CH0);                   //���ͨ�������жϱ�־λ
        DMA_IRQ_EN(DMA_CH0);

	exti_init(PORTC,7,rising_down);	        //���жϣ��ڲ������������ش����ж�

        
//        disable_irq(89);                       //�ر�PTC���ж�
					
}

void ov7727_get_img()
{
    img_flag = IMG_START;					//��ʼ�ɼ�ͼ��
    PORTC_ISFR=~0;							//д1���жϱ�־λ(����ģ���Ȼ�ص���һ���жϾ����ϴ����ж�)
    enable_irq(89); 						//����PTC���ж�
    while(img_flag != IMG_FINISH)           //�ȴ�ͼ��ɼ����
    {
        if(img_flag == IMG_FAIL)            //����ͼ��ɼ����������¿�ʼ�ɼ�
        {
            img_flag = IMG_START;			//��ʼ�ɼ�ͼ��
            PORTC_ISFR=~0;					//д1���жϱ�־λ(����ģ���Ȼ�ص���һ���жϾ����ϴ����ж�)
            enable_irq(89); 				//����PTC���ж�
        }
    }

}


/*OV7725��ʼ�����ñ�*/
Register_Info ov7727_reg[] =
{

    //�Ĵ������Ĵ���ֵ��
    {COM4         ,0xC1},
    {CLKRC        ,0x00}, 
    {COM3         ,0xD0}, 
    {COM5         ,0x00},
    {COM7         ,0x40},  
    {HSTART       ,0x3F},
    {HSIZE        ,0x50}, 
    {VSTRT        ,0x03},
    {VSIZE        ,0x78},
    {HREF         ,0x00}, 
    {SCAL0        ,0x0A},
    {AWB_Ctrl0    ,0xE0},
	{DSPAuto      ,0xff},
    {DSP_Ctrl2    ,0x0C}, 
    {DSP_Ctrl3    ,0x00}, 
    {DSP_Ctrl4    ,0x00},
    {HOutSize     ,0x14},
    {VOutSize     ,0x1E},
    {EXHCH        ,0x00},
	{GAM1         ,0x0c},
	{GAM2         ,0x16},
	{GAM3         ,0x2a},
	{GAM4         ,0x4e},
	{GAM5         ,0x61},
	{GAM6         ,0x6f},
	{GAM7         ,0x7b},
	{GAM8         ,0x86},
	{GAM9         ,0x8e},
	{GAM10        ,0x97},
	{GAM11        ,0xa4},
	{GAM12        ,0xaf},
	{GAM13        ,0xc5},
	{GAM14        ,0xd7},
	{GAM15        ,0xe8},
	{SLOP         ,0x20},
	{LC_RADI      ,0x00},
	{LC_COEF      ,0x13},
	{LC_XC        ,0x08},
	{LC_COEFB     ,0x14},
	{LC_COEFR     ,0x17},
	{LC_CTR       ,0x05},
	{BDBase       ,0x99},
	{BDMStep      ,0x03},
    {SDE          ,0x04},
    {BRIGHT       ,0x00},                              
    {CNST         ,0xFF},
    {SIGN         ,0x06},
    {UVADJ0       ,0x11},
    {UVADJ1       ,0x02},
    
};

u8 cfgnum = sizeof(ov7727_reg)/sizeof(ov7727_reg[0]);   /*�ṹ�������Ա��Ŀ*/



/************************************************
 * ��������Ov7725_Init
 * ����  ��Sensor��ʼ��
 * ����  ����
 * ���  ������1�ɹ�������0ʧ��
 * ע��  ����
 ************************************************/
u8 Ov7725_Init(u8 *imgaddr)
{

    IMG_BUFF = imgaddr;
	while(Ov7725_reg_Init() == 0);
	Ov7725_exti_Init();
    return 0;
}

/************************************************
 * ��������Ov7725_reg_Init
 * ����  ��Sensor �Ĵ��� ��ʼ��
 * ����  ����
 * ���  ������1�ɹ�������0ʧ��
 * ע��  ����
 ************************************************/
u8 Ov7725_reg_Init(void)
{
    u16 i = 0;
    u8 Sensor_IDCode = 0;
	SCCB_GPIO_init();
	
	OV7725_Delay_ms(50);
    while( 0 == SCCB_WriteByte ( 0x12, 0x80 ) ) /*��λsensor */
    {
        i++;
		if(i == 20)
		{
			DEBUG_OUT("����:SCCBд���ݴ���");
			//OV7725_Delay_ms(50);
			return 0 ;
		}
		
    }
    OV7725_Delay_ms(50);
    if( 0 == SCCB_ReadByte( &Sensor_IDCode, 1, 0x0b ) )	 /* ��ȡsensor ID��*/
    {
        DEBUG_OUT("����:��ȡIDʧ��");
        return 0;
    }
    DEBUG_OUT("Get ID success��SENSOR ID is 0x%x", Sensor_IDCode);
    DEBUG_OUT("Config Register Number is %d ", cfgnum);
    if(Sensor_IDCode == OV7725_ID)
    {
        for( i = 0 ; i < cfgnum ; i++ )
        {
            if( 0 == SCCB_WriteByte(ov7727_reg[i].Address, ov7727_reg[i].Value) )
            {
                DEBUG_OUT("����:д�Ĵ���0x%xʧ��", ov7727_reg[i].Address);
                return 0;
            }
        }
    }
    else
    {
        return 0;
    }
    DEBUG_OUT("OV7725 Register Config Success!");
    return 1;
}



//ͼ���ѹ
//data1Դ��ַ  data2Ŀ�ĵ�ַ

void img_extract(u8 *data1,u8 *data2,u16 lenth)
{
    u8  color[2] = {0,255};
    u8  temp;
       
    while(lenth--)
    {
        temp = *data1++;
        /*
        *data2++ = color[(temp >> 0) & 0x01];
        *data2++ = color[(temp >> 1) & 0x01];
        *data2++ = color[(temp >> 2) & 0x01];
        *data2++ = color[(temp >> 3) & 0x01];
        *data2++ = color[(temp >> 4) & 0x01];
        *data2++ = color[(temp >> 5) & 0x01];
        *data2++ = color[(temp >> 6) & 0x01];
        *data2++ = color[(temp >> 7) & 0x01];
        */
        
        *data2++ = color[(temp >> 7) & 0x01];
        *data2++ = color[(temp >> 6) & 0x01];
        *data2++ = color[(temp >> 5) & 0x01];
        *data2++ = color[(temp >> 4) & 0x01];
        *data2++ = color[(temp >> 3) & 0x01];
        *data2++ = color[(temp >> 2) & 0x01];
        *data2++ = color[(temp >> 1) & 0x01];
        *data2++ = color[(temp >> 0) & 0x01];
        
    }
}


