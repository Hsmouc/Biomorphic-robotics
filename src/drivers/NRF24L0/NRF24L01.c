#include "common.h"
#include "gpio.h"
#include "exti.h"
#include "spi.h"
#include "NRF24L01.h"
#include "delay.h"







u8 RX_BUF[RX_PLOAD_WIDTH];		//�������ݻ���
//u8 TX_BUF[TX_PLOAD_WIDTH];		//�������ݻ���



u8 TX_ADDRESS[5] = {0xc0,0x43,0x10,0x10,0x01};  // ����һ����̬���͵�ַ
u8 RX_ADDRESS[5] = {0xc0,0x43,0x10,0x10,0x01};






/*
 * ��������NRF_Init
 * ����  ��SPI�� I/O����
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void NRF_Init(void)
{
    //����NRF�ܽŸ���
//    spi_init(SPI1,MASTER);               //��ʼ��SPI,����ģʽ
    
    gpio_init(PORTC,9, GPO,LOW);           //��ʼ��CE��Ĭ�Ͻ������ģʽ
    gpio_init(PORTC,10, GPO,HIGH);          //��ʼ��PCSN�ܽ�Ϊ������͵�ƽѡ��  
    gpio_init(PORTC,8, GPI,LOW);           //��ʼ��IRQ�ܽ�Ϊ����     
    
    gpio_init(PORTB,16, GPI,LOW);           
    gpio_init(PORTB,17, GPO,LOW); 
    gpio_init(PORTB,11, GPO,LOW);    
    
    NRF_CE_LOW();
    NRF_SCK_LOW();
    NRF_PCSN_HIGH();
}

void DelayUs0(unsigned int us)
{
  int ii,jj;
  if (us<1) us=1;
  for(ii=0;ii<us;ii++)
    for(jj=0;jj<13;jj++);   //50MHz--1us
}
/****IO ��ģ��SPI���� ����**********/

u8 NRF_RW(u8 dat)
{
    u8 bit_ctr;
    for(bit_ctr=0;bit_ctr<8;bit_ctr++)
    {
            if((dat&0x80)==0x80)
                NRF_MO_HIGH();
            else
                NRF_MO_LOW();
            dat=(dat<<1);
            NRF_SCK_HIGH();
            if(NRF_MI())
              dat|=1;
            NRF_SCK_LOW();
    }
    return(dat);
}

/*
 * ��������NRF_WriteReg
 * ����  ��������NRF�ض��ļĴ���д������
 * ����  ��reg:NRF������+�Ĵ�����ַ��
 		   dat:��Ҫ��Ĵ���д�������
 * ���  ��NRF��status�Ĵ�����״̬
 * ����  ���ڲ�����
 */
u8 NRF_WriteReg(u8 reg,u8 dat)
{
      u8 status;
//      NRF_CE_LOW();
      
      /*�õ�CSN��ʹ��SPI����*/
      NRF_PCSN_LOW(); 
      
      /*��������Ĵ����� */
      status = NRF_RW(reg);
             
      /*��Ĵ���д������*/
      NRF_RW(dat); 
      
      /*CSN���ߣ����*/
      NRF_PCSN_HIGH();
      
      /*����״̬�Ĵ�����ֵ*/
      return(status);
}


/*
 * ��������NRF_ReadReg
 * ����  �����ڴ�NRF�ض��ļĴ�����������
 * ����  ��reg:NRF������+�Ĵ�����ַ��
 * ���  ���Ĵ����е�����
 * ����  ���ڲ�����
 */
u8 NRF_ReadReg(u8 reg)
{
      u8 reg_val;
      
      NRF_CE_LOW();
      
       /*�õ�CSN��ʹ��SPI����*/
      NRF_PCSN_LOW(); 
                              
       /*���ͼĴ�����*/
      NRF_RW(reg); 
      
       /*��ȡ�Ĵ�����ֵ */
      reg_val = NRF_RW(NOP);
      
      /*CSN���ߣ����*/
      NRF_PCSN_HIGH();
      
      return reg_val;
}

/*
 * ��������NRF_WriteBuf
 * ����  ��������NRF�ļĴ�����д��һ������
 * ����  ��reg:NRF������+�Ĵ�����ַ��
 		   pBuf���洢�˽�Ҫд��д�Ĵ������ݵ����飬�ⲿ����
		   bytes: pBuf�����ݳ���	
 * ���  ��NRF��status�Ĵ�����״̬
 * ����  ���ⲿ����
 */
u8 NRF_WriteBuf(u8 reg ,u8 *pBuf,u32 len)
{
      u8 status,byte_ctr;
      
      /*�õ�CSN��ʹ��SPI����*/
      NRF_PCSN_LOW(); 
      
      /*���ͼĴ�����*/	
      status= NRF_RW(reg);      //continue,������ȡ��Ƭѡ
      
      /*�򻺳���д������*/
      for(byte_ctr=0;byte_ctr<len;byte_ctr++)
              NRF_RW(*pBuf++);	
      
      /*CSN���ߣ����*/
      NRF_PCSN_HIGH();    
      return (status);	//����NRF24L01��״̬ 		
}


/*
 * ��������NRF_ReadBuf
 * ����  �����ڴ�NRF�ļĴ����ж���һ������
 * ����  ��reg:NRF������+�Ĵ�����ַ��
 		   pBuf�����ڴ洢���������ļĴ������ݵ����飬�ⲿ����
		   bytes: pBuf�����ݳ���	
 * ���  ��NRF��status�Ĵ�����״̬
 * ����  ���ⲿ����
 */
u8 NRF_ReadBuf(u8 reg,u8 *pBuf,u8 len)
{
      u8 status, byte_cnt;
      
      NRF_CE_LOW();
      
       /*�õ�CSN��ʹ��SPI����*/
      NRF_PCSN_LOW(); 
      /*���ͼĴ�����*/	
      status = NRF_RW(reg);
      
      for(byte_cnt = 0 ;byte_cnt < len;byte_cnt++)
      {
          pBuf[byte_cnt] = NRF_RW(NOP);
      }
      
      /*CSN���ߣ����*/
      NRF_PCSN_HIGH();
      
      return status;		//���ؼĴ���״ֵ̬
}


/********************************************************* 
*														 *
*														 *
************NRF����Ϊ����ģʽ����������*******************
*														 *
*														 *
*********************************************************/
/*
 * ��������NRF_RX_Mode
 * ����  �����ò��������ģʽ
 * ����  ����	
 * ���  ����
 * ����  ���ⲿ����
 */
void NRF_RX_Mode(void)
{
    NRF_CE_LOW();	
              
    NRF_WriteReg(NRF_WRITE_REG+RF_CH,0x40);               //����RFͨ��ΪCHANAL

    NRF_WriteReg(NRF_WRITE_REG+RF_SETUP,0x07);              //����TX�������,0db����, 0x07 1Mbps 0x0f 2Mbps,���������濪�� 
    
    NRF_WriteReg(NRF_WRITE_REG+EN_AA,0x01);                 //ʹ��ͨ��0���Զ�Ӧ��  
    
    NRF_WriteReg(NRF_WRITE_REG+EN_RXADDR,0x01);             //ʹ��ͨ��0�Ľ��յ�ַ    
    
    NRF_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ

    NRF_WriteReg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);    //ѡ��ͨ��0����Ч���ݿ��   
    
    NRF_WriteReg(NRF_WRITE_REG+CONFIG, 0x0f);    //CRCʹ�ܣ�16λCRCУ��      
      
    
    /*CE���ߣ��������ģʽ*/	
    NRF_CE_HIGH();
    
    DelayUs0(25);
}
/*
u8 NRF_Rx_Dat(u8 *RxDate_Buf)				 		 //  �������ݺ���
{
    u8 state;                   //����״̬
    u32 aa=0;

    //�ȴ������ж�
    while(NRF_Read_IRQ()!=0&&aa<0xffff)aa++;    
    NRF_CE_LOW();  	 //�������״̬
    state=NRF_ReadReg(NRF_READ_REG+STATUS); //�������ݺ��ȡ״̬�Ĵ���

    if(state & RX_DR)			         	 // �ж��Ƿ���յ����ݣ��ӵ��ʹ�RXȡ��
    {

          NRF_CE_LOW();                         //����

          NRF_ReadBuf(RD_RX_PLOAD,RxDate_Buf,RX_PLOAD_WIDTH);

		 
          NRF_WriteReg(NRF_WRITE_REG+STATUS,0xff);              //���յ����ݺ�RX_DR,TX_DS,MAX_PT���ø�Ϊ1��ͨ��д1������жϱ�־
          NRF_WriteReg(FLUSH_RX,0xff);
          NRF_PCSN_LOW();
          NRF_RW(FLUSH_RX);                                //�������FIFO �����ؼ�������Ȼ��������벻���ĺ����������Ҽ�ס����                   
          NRF_PCSN_HIGH();
          return RX_DR;	 
    }
    else
        return ERROR;    
}
*/

/********************************************************* 
*														 *
*														 *
************NRF����Ϊ����ģʽ����������*******************
*														 *
*														 *
*********************************************************/

/*
 * ��������NRF_TX_Mode
 * ����  �����÷���ģʽ
 * ����  ����	
 * ���  ����
 * ����  ���ⲿ����
 */
void NRF_TX_Mode(void)
{  
   
    
    //����NRF�Ĵ���
    NRF_CE_LOW();	
    
    NRF_WriteReg(NRF_WRITE_REG+EN_RXADDR,0x01);             //ʹ��ͨ��0�Ľ��յ�ַ 
    
    NRF_WriteReg(NRF_WRITE_REG+EN_AA,0x01);                 //ʹ��ͨ��0���Զ�Ӧ��  
 
    NRF_WriteReg(NRF_WRITE_REG+SETUP_RETR,0x0f);     //���õ�ַ����Ϊ TX_ADR_WIDTH
    
    NRF_WriteReg(NRF_WRITE_REG+RF_CH,0x40);               //����RFͨ��ΪCHANAL  2.4+64/1000  0x40 = 64
 
    NRF_WriteReg(NRF_WRITE_REG+RF_SETUP,0x07);              //����TX�������,0db����,1Mbps,���������濪�� 
    
    NRF_WriteReg(NRF_WRITE_REG+CONFIG,0x0e);            //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�  
    
    /*CE���ߣ����뷢��ģʽ*/
    NRF_CE_HIGH();
	
    DelayUs0(25); 

    
}


 /*
 * ��������NRF_Tx_Dat
 * ����  ��������NRF�ķ��ͻ�������д������
 * ����  ��txBuf���洢�˽�Ҫ���͵����ݵ����飬�ⲿ����	
 * ���  �����ͽ�����ɹ�����TXDS,ʧ�ܷ���MAXRT��ERROR
 * ����  ���ⲿ����
 */ 
void NRF_Tx_Dat(u8 *txbuf)
{
     //����NRF�Ĵ���
    NRF_CE_LOW();	
    
    NRF_WriteBuf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH);    //дTX�ڵ��ַ 

    NRF_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH); //����RX�ڵ��ַ ,��ҪΪ��ʹ��ACK   
    
    NRF_WriteReg(FLUSH_TX,0xff);
    
    NRF_WriteBuf(WR_TX_PLOAD , txbuf , TX_PLOAD_WIDTH);                 //д������ 
    
    NRF_WriteReg(NRF_WRITE_REG+CONFIG,0x0e);      // IRQ�շ�����ж���Ӧ��16λCRC��������
    
    NRF_CE_HIGH();
    
    DelayUs0(25); 
/*
    while(NRF_Read_IRQ()!=0&&aa<0xffff)aa++;
    state=NRF_ReadReg(NRF_READ_REG+STATUS); //�������ݺ��ȡ״̬�Ĵ���
    NRF_WriteReg(NRF_WRITE_REG+STATUS,0xff);                // ���TX_DS��MAX_RT�жϱ�־
    NRF_WriteReg(FLUSH_TX,0xff);
*/    
    
//    return state;
 
}

 //���ڷ���ģʽ����Ӧ���ź� 
u8 NRF_CheckACK()
{ 	
      u8 a;
      u8 state;
      
      state = NRF_ReadReg(NRF_READ_REG+STATUS);                    // ����״̬�Ĵ���
      if(state&&TX_DS||state&&MAX_RT)                                      //��������жϣ����յ�Ӧ���ź� ���� �Զ��ط����������
      {
            a = NRF_ReadReg(NRF_READ_REG+OBSERVE_TX);			      //�Զ��ط�����
            a = a & 0x0f;
            
            NRF_WriteReg(NRF_WRITE_REG+STATUS,0xff);                // ���TX_DS��MAX_RT�жϱ�־
            
         //   NRF_PCSN_LOW();                   
         //   NRF_RW(FLUSH_TX);           
         //   NRF_PCSN_HIGH();       
            NRF_WriteReg(FLUSH_TX,0xff);                                //�������FIFO �����ؼ�������Ȼ��������벻���ĺ����������Ҽ�ס���� 
            return(state);
      }
      else
            return(0);
}







