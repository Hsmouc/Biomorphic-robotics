/********************  ********************
 * �ļ���       ��sysinit.h
 * ����         ��ϵͳʱ��������غ�����ͷ�ļ�
 * ��ע         ��

**********************************************************************************/

#ifndef    _SYSINIT_H_
#define    _SYSINIT_H_

/********************************************************************/
extern u32 core_clk_khz;
extern u32 core_clk_mhz;
extern u32 bus_clk_khz;



// function prototypes
void sysinit (void);
void trace_clk_init(void);
void fb_clk_init(void);
void enable_abort_button(void);



/********************************************************************/

#endif  //_SYSINIT_H_