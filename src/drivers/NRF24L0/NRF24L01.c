#include "common.h"
#include "gpio.h"
#include "exti.h"
#include "spi.h"
#include "NRF24L01.h"
#include "delay.h"







u8 RX_BUF[RX_PLOAD_WIDTH];		//接收数据缓存
//u8 TX_BUF[TX_PLOAD_WIDTH];		//发射数据缓存



u8 TX_ADDRESS[5] = {0xc0,0x43,0x10,0x10,0x01};  // 定义一个静态发送地址
u8 RX_ADDRESS[5] = {0xc0,0x43,0x10,0x10,0x01};






/*
 * 函数名：NRF_Init
 * 描述  ：SPI的 I/O配置
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void NRF_Init(void)
{
    //配置NRF管脚复用
//    spi_init(SPI1,MASTER);               //初始化SPI,主机模式
    
    gpio_init(PORTC,9, GPO,LOW);           //初始化CE，默认进入待机模式
    gpio_init(PORTC,10, GPO,HIGH);          //初始化PCSN管脚为输出，低电平选中  
    gpio_init(PORTC,8, GPI,LOW);           //初始化IRQ管脚为输入     
    
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
/****IO 口模拟SPI总线 代码**********/

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
 * 函数名：NRF_WriteReg
 * 描述  ：用于向NRF特定的寄存器写入数据
 * 输入  ：reg:NRF的命令+寄存器地址。
 		   dat:将要向寄存器写入的数据
 * 输出  ：NRF的status寄存器的状态
 * 调用  ：内部调用
 */
u8 NRF_WriteReg(u8 reg,u8 dat)
{
      u8 status;
//      NRF_CE_LOW();
      
      /*置低CSN，使能SPI传输*/
      NRF_PCSN_LOW(); 
      
      /*发送命令及寄存器号 */
      status = NRF_RW(reg);
             
      /*向寄存器写入数据*/
      NRF_RW(dat); 
      
      /*CSN拉高，完成*/
      NRF_PCSN_HIGH();
      
      /*返回状态寄存器的值*/
      return(status);
}


/*
 * 函数名：NRF_ReadReg
 * 描述  ：用于从NRF特定的寄存器读出数据
 * 输入  ：reg:NRF的命令+寄存器地址。
 * 输出  ：寄存器中的数据
 * 调用  ：内部调用
 */
u8 NRF_ReadReg(u8 reg)
{
      u8 reg_val;
      
      NRF_CE_LOW();
      
       /*置低CSN，使能SPI传输*/
      NRF_PCSN_LOW(); 
                              
       /*发送寄存器号*/
      NRF_RW(reg); 
      
       /*读取寄存器的值 */
      reg_val = NRF_RW(NOP);
      
      /*CSN拉高，完成*/
      NRF_PCSN_HIGH();
      
      return reg_val;
}

/*
 * 函数名：NRF_WriteBuf
 * 描述  ：用于向NRF的寄存器中写入一串数据
 * 输入  ：reg:NRF的命令+寄存器地址。
 		   pBuf：存储了将要写入写寄存器数据的数组，外部定义
		   bytes: pBuf的数据长度	
 * 输出  ：NRF的status寄存器的状态
 * 调用  ：外部调用
 */
u8 NRF_WriteBuf(u8 reg ,u8 *pBuf,u32 len)
{
      u8 status,byte_ctr;
      
      /*置低CSN，使能SPI传输*/
      NRF_PCSN_LOW(); 
      
      /*发送寄存器号*/	
      status= NRF_RW(reg);      //continue,即不会取消片选
      
      /*向缓冲区写入数据*/
      for(byte_ctr=0;byte_ctr<len;byte_ctr++)
              NRF_RW(*pBuf++);	
      
      /*CSN拉高，完成*/
      NRF_PCSN_HIGH();    
      return (status);	//返回NRF24L01的状态 		
}


/*
 * 函数名：NRF_ReadBuf
 * 描述  ：用于从NRF的寄存器中读出一串数据
 * 输入  ：reg:NRF的命令+寄存器地址。
 		   pBuf：用于存储将被读出的寄存器数据的数组，外部定义
		   bytes: pBuf的数据长度	
 * 输出  ：NRF的status寄存器的状态
 * 调用  ：外部调用
 */
u8 NRF_ReadBuf(u8 reg,u8 *pBuf,u8 len)
{
      u8 status, byte_cnt;
      
      NRF_CE_LOW();
      
       /*置低CSN，使能SPI传输*/
      NRF_PCSN_LOW(); 
      /*发送寄存器号*/	
      status = NRF_RW(reg);
      
      for(byte_cnt = 0 ;byte_cnt < len;byte_cnt++)
      {
          pBuf[byte_cnt] = NRF_RW(NOP);
      }
      
      /*CSN拉高，完成*/
      NRF_PCSN_HIGH();
      
      return status;		//返回寄存器状态值
}


/********************************************************* 
*														 *
*														 *
************NRF设置为接收模式并接收数据*******************
*														 *
*														 *
*********************************************************/
/*
 * 函数名：NRF_RX_Mode
 * 描述  ：配置并进入接收模式
 * 输入  ：无	
 * 输出  ：无
 * 调用  ：外部调用
 */
void NRF_RX_Mode(void)
{
    NRF_CE_LOW();	
              
    NRF_WriteReg(NRF_WRITE_REG+RF_CH,0x40);               //设置RF通道为CHANAL

    NRF_WriteReg(NRF_WRITE_REG+RF_SETUP,0x07);              //设置TX发射参数,0db增益, 0x07 1Mbps 0x0f 2Mbps,低噪声增益开启 
    
    NRF_WriteReg(NRF_WRITE_REG+EN_AA,0x01);                 //使能通道0的自动应答  
    
    NRF_WriteReg(NRF_WRITE_REG+EN_RXADDR,0x01);             //使能通道0的接收地址    
    
    NRF_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址

    NRF_WriteReg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);    //选择通道0的有效数据宽度   
    
    NRF_WriteReg(NRF_WRITE_REG+CONFIG, 0x0f);    //CRC使能，16位CRC校验      
      
    
    /*CE拉高，进入接收模式*/	
    NRF_CE_HIGH();
    
    DelayUs0(25);
}
/*
u8 NRF_Rx_Dat(u8 *RxDate_Buf)				 		 //  接收数据函数
{
    u8 state;                   //接收状态
    u32 aa=0;

    //等待接收中断
    while(NRF_Read_IRQ()!=0&&aa<0xffff)aa++;    
    NRF_CE_LOW();  	 //进入待机状态
    state=NRF_ReadReg(NRF_READ_REG+STATUS); //发送数据后读取状态寄存器

    if(state & RX_DR)			         	 // 判断是否接收到数据，接到就从RX取出
    {

          NRF_CE_LOW();                         //待机

          NRF_ReadBuf(RD_RX_PLOAD,RxDate_Buf,RX_PLOAD_WIDTH);

		 
          NRF_WriteReg(NRF_WRITE_REG+STATUS,0xff);              //接收到数据后RX_DR,TX_DS,MAX_PT都置高为1，通过写1来清楚中断标志
          NRF_WriteReg(FLUSH_RX,0xff);
          NRF_PCSN_LOW();
          NRF_RW(FLUSH_RX);                                //用于清空FIFO ！！关键！！不然会出现意想不到的后果！！！大家记住！！                   
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
************NRF设置为发送模式并发送数据*******************
*														 *
*														 *
*********************************************************/

/*
 * 函数名：NRF_TX_Mode
 * 描述  ：配置发送模式
 * 输入  ：无	
 * 输出  ：无
 * 调用  ：外部调用
 */
void NRF_TX_Mode(void)
{  
   
    
    //配置NRF寄存器
    NRF_CE_LOW();	
    
    NRF_WriteReg(NRF_WRITE_REG+EN_RXADDR,0x01);             //使能通道0的接收地址 
    
    NRF_WriteReg(NRF_WRITE_REG+EN_AA,0x01);                 //使能通道0的自动应答  
 
    NRF_WriteReg(NRF_WRITE_REG+SETUP_RETR,0x0f);     //设置地址长度为 TX_ADR_WIDTH
    
    NRF_WriteReg(NRF_WRITE_REG+RF_CH,0x40);               //设置RF通道为CHANAL  2.4+64/1000  0x40 = 64
 
    NRF_WriteReg(NRF_WRITE_REG+RF_SETUP,0x07);              //设置TX发射参数,0db增益,1Mbps,低噪声增益开启 
    
    NRF_WriteReg(NRF_WRITE_REG+CONFIG,0x0e);            //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,发射模式,开启所有中断  
    
    /*CE拉高，进入发送模式*/
    NRF_CE_HIGH();
	
    DelayUs0(25); 

    
}


 /*
 * 函数名：NRF_Tx_Dat
 * 描述  ：用于向NRF的发送缓冲区中写入数据
 * 输入  ：txBuf：存储了将要发送的数据的数组，外部定义	
 * 输出  ：发送结果，成功返回TXDS,失败返回MAXRT或ERROR
 * 调用  ：外部调用
 */ 
void NRF_Tx_Dat(u8 *txbuf)
{
     //配置NRF寄存器
    NRF_CE_LOW();	
    
    NRF_WriteBuf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH);    //写TX节点地址 

    NRF_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH); //设置RX节点地址 ,主要为了使能ACK   
    
    NRF_WriteReg(FLUSH_TX,0xff);
    
    NRF_WriteBuf(WR_TX_PLOAD , txbuf , TX_PLOAD_WIDTH);                 //写入数据 
    
    NRF_WriteReg(NRF_WRITE_REG+CONFIG,0x0e);      // IRQ收发完成中断响应，16位CRC，主发送
    
    NRF_CE_HIGH();
    
    DelayUs0(25); 
/*
    while(NRF_Read_IRQ()!=0&&aa<0xffff)aa++;
    state=NRF_ReadReg(NRF_READ_REG+STATUS); //发送数据后读取状态寄存器
    NRF_WriteReg(NRF_WRITE_REG+STATUS,0xff);                // 清除TX_DS或MAX_RT中断标志
    NRF_WriteReg(FLUSH_TX,0xff);
*/    
    
//    return state;
 
}

 //用于发射模式接收应答信号 
u8 NRF_CheckACK()
{ 	
      u8 a;
      u8 state;
      
      state = NRF_ReadReg(NRF_READ_REG+STATUS);                    // 返回状态寄存器
      if(state&&TX_DS||state&&MAX_RT)                                      //发送完毕中断（接收到应答信号 或者 自动重发计数溢出）
      {
            a = NRF_ReadReg(NRF_READ_REG+OBSERVE_TX);			      //自动重发次数
            a = a & 0x0f;
            
            NRF_WriteReg(NRF_WRITE_REG+STATUS,0xff);                // 清除TX_DS或MAX_RT中断标志
            
         //   NRF_PCSN_LOW();                   
         //   NRF_RW(FLUSH_TX);           
         //   NRF_PCSN_HIGH();       
            NRF_WriteReg(FLUSH_TX,0xff);                                //用于清空FIFO ！！关键！！不然会出现意想不到的后果！！！大家记住！！ 
            return(state);
      }
      else
            return(0);
}







