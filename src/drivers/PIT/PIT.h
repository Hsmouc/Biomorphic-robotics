/****************************************
 * �ļ���       ��PIT.h
 * ����         �������жϼ�ʱ��ͷ�ļ�
 *

**********************************************************************************/

#ifndef _PIT_H_
#define _PIT_H_



typedef enum PITn
{
    PIT0,
    PIT1,
    PIT2,
    PIT3
} PITn;


void        pit_init(PITn, u32 cnt);                                               //��ʼ��PITn�������ö�ʱʱ��(��λΪbusʱ������)
#define     pit_init_ms(PITn,ms)          pit_init(PITn,ms * bus_clk_khz);         //��ʼ��PITn�������ö�ʱʱ��(��λΪ ms)
#define     pit_init_us(PITn,us)          pit_init(PITn,us * bus_clk_khz/1000);    //��ʼ��PITn�������ö�ʱʱ��(��λΪ us)
#define     pit_init_ns(PITn,ns)          pit_init(PITn,ns * bus_clk_khz/1000000); //��ʼ��PITn�������ö�ʱʱ��(��λΪ ns)



#define     PIT_Flag_Clear(PITn)          PIT_TFLG(PITn)|=PIT_TFLG_TIF_MASK        //���жϱ�־




void        pit_dma_init(PITn pitn, u32 cnt);
#define     pit_dma_init_ms(PITn,ms)     ASSERT( ((u64)(ms * bus_clk_khz)>>32)==0   );\
                                         pit_dma_init(PITn,ms * bus_clk_khz)



void    pit_time_start  (PITn pitn);                                                //PIT��ʼ��ʱ
uint32  pit_time_get    (PITn pitn);                                                //��ȡ PITn��ʱʱ��(��ʱʱ��ر� ��ʱ��)����λΪ busʱ�ӣ�(��ֵΪ 0xFFFFFFFF�����ʾ���)
void    pit_time_close  (PITn pitn);                                                //�ر� PIT ��ʱ

#define pit_time_get_ms(pitn)   (pit_time_get(pitn)/bus_clk_khz)                    //��ȡ��ʱʱ�䣨��λΪ ms��
#define pit_time_get_us(pitn)   (pit_time_get(pitn)/(bus_clk_khz/1000))             //��ȡ��ʱʱ�䣨��λΪ us��


#endif  //_PIT_H_