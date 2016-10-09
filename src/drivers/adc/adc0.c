/******************** ********************
 * 文件名       ：adc.c
 * 描述         ：adc驱动函数
 *

**********************************************************************************/

#include "common.h"
#include "adc.h"


volatile struct ADC_MemMap *ADCx[2] = {ADC0_BASE_PTR, ADC1_BASE_PTR}; //定义两个指针数组保存 ADCx 的地址


/**********************************************************************************
**Routine:ADC_Init
**input:    ADC---ADC0 and ADC1 module selection
  bits:     0--8bit, 1--12bit, 2--10bit, 3--16bit
  channel:  0~23 in total of 24 channels in single-ended convertion mode
  mode:     INT_MODE, SEARCH_MODE(母语解释一下吧，哈哈，这里分为中断方式和查询方式)
**********************************************************************************/
void ADC_Init(ADCn adcn, uint8 bits, uint8 channel)
{
    if(ADCx[adcn] == ADC0_BASE_PTR)
      SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;              /* turn on the ADC0 clock */ 
    else if(ADCx[adcn] == ADC1_BASE_PTR)
      SIM_SCGC3 |= (SIM_SCGC3_ADC1_MASK );           /* turn on the ADC1 clock */
    
    ADC_CFG1_REG(ADCx[adcn]) |= ADC_CFG1_ADIV(3)            /* normal power, clock rate is (inputclock)/8 */
                         + ADC_CFG1_ADLSMP_MASK      /* long sample time */
                         + ADC_CFG1_MODE(bits)       /* bits range 0 to 3(0--8bit, 1--12bit, 2--10bit, 3--16bit) */
                         + ADC_CFG1_ADICLK(1);       /* inputclock is busclock/2 */
                                                                                                                                              
    ADC_CFG2_REG(ADCx[adcn]) &= ~(ADC_CFG2_MUXSEL_MASK      /* a registers is selected */
                         + ADC_CFG2_ADACKEN_MASK     /* Asynchronous clock output disabled */
                         + ADC_CFG2_ADHSC_MASK       /* Normal conversion sequence selected */
                         + ADC_CFG2_ADLSTS_MASK);    /* Default longest sample time. */               
/***********************************************************************************
**set default status:Software triger(a convertion is initated following a write to 
  SC1A)compare function disabled, DMA is disabled, default voltage reference pin
  (external pins VREFH and VREFL).
***********************************************************************************/
    ADC_SC2_REG(ADCx[adcn]) = 0;
    
    ADC_SC3_REG(ADCx[adcn]) |= ADC_SC3_ADCO_MASK            /* continuous conversions */
                        + ADC_SC3_AVGE_MASK          /* hardware averages enabled. */
                        + ADC_SC3_AVGS(3);           /* 4 samples average */
                 
  //---when in software triger mode,  a conversion is actived after the ADC_SC1A is writed.   
    ADC_SC1_REG(ADCx[adcn],0)  = ADC_SC1_ADCH(channel);     /* single-ended AD20 channel is selected */


/*    //采用中断方式读取
    if(mode == INT_MODE)
    {
      ADC_SC1_REG(ADC,0)  |= ADC_SC1_AIEN_MASK;      // conversion complete interrrupt enabled //
      if(ADC == ADC0_BASE_PTR)
        enable_irq(adc0_isr_no);                     // enable the ADC0 IRQ interrupt //
      else if(ADC == ADC1_BASE_PTR)
        enable_irq(adc1_isr_no);                     // enable the ADC1 IRQ interrupt //
    }
    else if(mode == SEARCH_MODE)
*/      
    //采用查询的方式读取
      ADC_SC1_REG(ADCx[adcn],0)  &= ~ADC_SC1_AIEN_MASK;     /* conversion complete interrrupt disabled */
}
/**********************************************************************************
**Routine:Read_ADC
**input:    ADC---ADC0 and ADC1 module selection
  result:   the pointer of the return value of the convertion result.
**********************************************************************************/
void Read_ADC(ADCn adcn, uint16 * result)
{
    while(!(ADC_SC1_REG(ADCx[adcn],0)&ADC_SC1_COCO_MASK));  /* wait until the selected numbers of convertion(determined by the AGVS bits) */
    *result = ADC_R_REG(ADCx[adcn],0);                      /* clear the COCO flag by reading the corresponding data register. */
}
/**********************************************************************************
**Routine:adc0_isr
**Description: the interrupt service routine of ADC0
**********************************************************************************/  
void adc0_isr(void)
{
    (void) ADC0_RA;   
}
/**********************************************************************************
**Routine:adc1_isr
**Description: the interrupt service routine of ADC1
**********************************************************************************/
void adc1_isr(void)
{
    (void) ADC1_RA;
}