/********************  ********************
 * �ļ���       ��NRF24L01.h
 * ����         ��NRF24L01������
 *

**********************************************************************************/
#ifndef _NRF24L01_H_
#define _NRF24L01_H_     1

#include "spi.h"



//�������û����õ�ѡ��

#define MAX_ONCE_TX_NUM     32      //һ�δ�������֧�ֵ��ֽ�����0~32��
#define ADR_WIDTH           5       //�����ַ���ȣ�3~5��


//���õ��������


#define MAX_RT                  0x10    //�ﵽ����ط������жϱ�־λ
#define TX_DS		        0x20    //��������жϱ�־λ	  // 
#define RX_DR		        0x40    //���յ������жϱ�־λ


#define NRF_CE_HIGH()	    PTC9_OUT=1
#define NRF_CE_LOW()	    PTC9_OUT=0			  //CE�õ�
#define NRF_PCSN_LOW()      PTC10_OUT=0
#define NRF_PCSN_HIGH()     PTC10_OUT=1
#define NRF_Read_IRQ()	    PTC8_IN

#define NRF_MI()            PTB16_IN
#define NRF_MO_HIGH()       PTB17_OUT=1
#define NRF_MO_LOW()        PTB17_OUT=0
#define NRF_SCK_HIGH()      PTB11_OUT=1
#define NRF_SCK_LOW()       PTB11_OUT=0




#define TX_ADR_WIDTH 	ADR_WIDTH  	//�����ַ���
#define TX_PLOAD_WIDTH  MAX_ONCE_TX_NUM   //��������ͨ����Ч���ݿ��0~32Byte 

#define RX_ADR_WIDTH    ADR_WIDTH
#define RX_PLOAD_WIDTH  MAX_ONCE_TX_NUM




#define CHANAL 40	//Ƶ��ѡ�� 

// SPI(nRF24L01) commands ,	NRF��SPI����궨�壬���NRF����ʹ���ĵ�
#define NRF_READ_REG    0x00  // Define read command to register
#define NRF_WRITE_REG   0x20  // Define write command to register
#define RD_RX_PLOAD     0x61  // Define RX payload register address
#define WR_TX_PLOAD     0xA0  // Define TX payload register address
#define FLUSH_TX        0xE1  // Define flush TX register command
#define FLUSH_RX        0xE2  // Define flush RX register command
#define REUSE_TX_PL     0xE3  // Define reuse TX payload register command
#define NOP             0xFF  // Define No Operation, might be used to read status register

// SPI(nRF24L01) registers(addresses) ��NRF24L01 ��ؼĴ�����ַ�ĺ궨��
#define CONFIG      0x00  // 'Config' register address
#define EN_AA       0x01  // 'Enable Auto Acknowledgment' register address
#define EN_RXADDR   0x02  // 'Enabled RX addresses' register address
#define SETUP_AW    0x03  // 'Setup address width' register address
#define SETUP_RETR  0x04  // 'Setup Auto. Retrans' register address
#define RF_CH       0x05  // 'RF channel' register address
#define RF_SETUP    0x06  // 'RF setup' register address
#define STATUS      0x07  // 'Status' register address
#define OBSERVE_TX  0x08  // 'Observe TX' register address
#define CD          0x09  // 'Carrier Detect' register address
#define RX_ADDR_P0  0x0A  // 'RX address pipe0' register address
#define RX_ADDR_P1  0x0B  // 'RX address pipe1' register address
#define RX_ADDR_P2  0x0C  // 'RX address pipe2' register address
#define RX_ADDR_P3  0x0D  // 'RX address pipe3' register address
#define RX_ADDR_P4  0x0E  // 'RX address pipe4' register address
#define RX_ADDR_P5  0x0F  // 'RX address pipe5' register address
#define TX_ADDR     0x10  // 'TX address' register address
#define RX_PW_P0    0x11  // 'RX payload width, pipe0' register address
#define RX_PW_P1    0x12  // 'RX payload width, pipe1' register address
#define RX_PW_P2    0x13  // 'RX payload width, pipe2' register address
#define RX_PW_P3    0x14  // 'RX payload width, pipe3' register address
#define RX_PW_P4    0x15  // 'RX payload width, pipe4' register address
#define RX_PW_P5    0x16  // 'RX payload width, pipe5' register address
#define FIFO_STATUS 0x17  // 'FIFO Status Register' register address








extern  void NRF_Init(void);
extern  void DelayUs0(unsigned int us);
extern  u8 NRF_RW(u8 dat);
extern  u8 NRF_WriteReg(u8 reg,u8 dat);
extern  u8 NRF_ReadReg(u8 reg);
extern  u8 NRF_WriteBuf(u8 reg ,u8 *pBuf,u32 len);
extern  u8 NRF_ReadBuf(u8 reg,u8 *pBuf,u8 len);
extern  void NRF_RX_Mode(void);
//extern  u8 NRF_Rx_Dat(u8 *rxbuf); 
extern  void NRF_TX_Mode(void);
extern  void NRF_Tx_Dat(u8 *txbuf);
extern  u8 NRF_CheckACK();







#endif      //_NRF24L0_H_