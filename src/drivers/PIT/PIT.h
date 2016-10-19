/****************************************
 * 文件名       ：PIT.h
 * 描述         ：周期中断计时器头文件
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


void        pit_init(PITn, u32 cnt);                                               //初始化PITn，并设置定时时间(单位为bus时钟周期)
#define     pit_init_ms(PITn,ms)          pit_init(PITn,ms * bus_clk_khz);         //初始化PITn，并设置定时时间(单位为 ms)
#define     pit_init_us(PITn,us)          pit_init(PITn,us * bus_clk_khz/1000);    //初始化PITn，并设置定时时间(单位为 us)
#define     pit_init_ns(PITn,ns)          pit_init(PITn,ns * bus_clk_khz/1000000); //初始化PITn，并设置定时时间(单位为 ns)



#define     PIT_Flag_Clear(PITn)          PIT_TFLG(PITn)|=PIT_TFLG_TIF_MASK        //清中断标志




void        pit_dma_init(PITn pitn, u32 cnt);
#define     pit_dma_init_ms(PITn,ms)     ASSERT( ((u64)(ms * bus_clk_khz)>>32)==0   );\
                                         pit_dma_init(PITn,ms * bus_clk_khz)



void    pit_time_start  (PITn pitn);                                                //PIT开始计时
uint32  pit_time_get    (PITn pitn);                                                //获取 PITn计时时间(超时时会关闭 定时器)（单位为 bus时钟）(若值为 0xFFFFFFFF，则表示溢出)
void    pit_time_close  (PITn pitn);                                                //关闭 PIT 计时

#define pit_time_get_ms(pitn)   (pit_time_get(pitn)/bus_clk_khz)                    //获取计时时间（单位为 ms）
#define pit_time_get_us(pitn)   (pit_time_get(pitn)/(bus_clk_khz/1000))             //获取计时时间（单位为 us）


#endif  //_PIT_H_