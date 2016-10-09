
#include "common.h"
#include "dac.h"

volatile struct DAC_MemMap *DACx[2] = {DAC0_BASE_PTR, DAC1_BASE_PTR}; //��������ָ�����鱣�� DACx �ĵ�ַ

/*************************************************************************
*
*  �������ƣ�vref_out_init
*  ����˵�����ڲ��ο���ѹ��ʼ��
*  ����˵������
*  �������أ���

*************************************************************************/
void VREF_init(void)
{

    SIM_SCGC4 |= SIM_SCGC4_VREF_MASK ;          //����VREFģ��ʱ��

    VREF_SC = (  0
                 | VREF_SC_MODE_LV(1)             //VREF_OUT�ڲ��ⲿʹ����Ч
                 | VREF_SC_VREFEN_MASK            //ʹ���ڲ��ο���ѹ
                 | VREF_SC_REGEN_MASK             //1.75V ��ѹ������
              ) ;

    while (!(VREF_SC & VREF_SC_VREFST_MASK));    //�ȴ��ڲ���ѹ�ο��ȶ�
}

/*************************************************************************
*
*  �������ƣ�dac_once_init
*  ����˵����DACһ��ת����ʼ��
*  ����˵����DACn        ģ��ţ� DAC0�� DAC1��
*            VREF        �ο���ѹѡ��VREF_OUT��VDDA��
*  �������أ���
********************************************/
void dac_once_init(DACn dacn, VREF vref)
{


    /* ʹ��ʱ�� */
    SIM_SCGC2 |= (SIM_SCGC2_DAC0_MASK << dacn) ;    //ʹ��DACģ��

    if(vref == VREF_OUT)
    {
        VREF_init();
    }

    /*  ����DAC�Ĵ���  */

    //����DAC_C0�Ĵ���
    DAC_C0_REG(DACx[dacn])  = ( 0
                                |  DAC_C0_DACTRGSEL_MASK                //ѡ����������
                                |  (vref << DAC_C0_DACRFS_SHIFT )       //ѡ��ο���ѹ
                                |  DAC_C0_DACEN_MASK                    //ʹ��DACģ��
                              );

    //����DAC_C1�Ĵ���
    DAC_C1_REG(DACx[dacn]) = ( 0
                             ) ;
    //����DAC_C2�Ĵ���
    DAC_C2_REG(DACx[dacn]) = ( 0
                               | DAC_C2_DACBFRP(0)             //���û�������ָ��ָ��0
                             );

    DAC_DATA_REG(DACx[dacn], 0) = 0;   //Ĭ�������͵�ѹ
}

/*************************************************************************

*
*  �������ƣ�dac_once_convert
*  ����˵����DACһ��ת������
*  ����˵����DACn        ģ��ţ� DAC0�� DAC1��
*            val         DACת�����ݣ�12bit��
*  �������أ���

*************************************************************************/
void dac_once_convert(DACn dacn, u16 val)
{
    ASSERT(val < 0x1000);               //val Ϊ 12bit

    DAC_DATA_REG(DACx[dacn], 0) = val;
}

/*************************************************************************
*
*  �������ƣ�dac_once_get
*  ����˵����DACһ��ת�����ȡ��ǰת����������
*  ����˵����DACn        ģ��ţ� DAC0�� DAC1��
*  �������أ���

*************************************************************************/
u16 dac_once_get(DACn dacn)
{
    return DAC_DATA_REG(DACx[dacn], 0);
}