#ifndef __PCA9685_H
#define __PCA9685_H

/* Devices */
#define LEDDRV1    0x80
#define LEDDRV2    0x82

/* Registers */
#define PCA9685_BASE          	 0x01
#define PCA9685_MODE1            0x00
#define PCA9685_MODE2            0x01
#define PCA9685_SUBADR1            0x02
#define PCA9685_SUBADR2            0x03
#define PCA9685_SUBADR3            0x04
#define PCA9685_ALLCALLADR        0x05
#define PCA9685_LED0_ON_L        0x06
#define PCA9685_LED0_ON_H        0x07
#define PCA9685_LED0_OFF_L        0x08
#define PCA9685_LED0_OFF_H        0x09
#define PCA9685_LED1_ON_L        0x0a
#define PCA9685_LED1_ON_H        0x0b
#define PCA9685_LED1_OFF_L        0x0c
#define PCA9685_LED1_OFF_H        0x0d
#define PCA9685_LED2_ON_L        0x0e
#define PCA9685_LED2_ON_H        0x0f
#define PCA9685_LED2_OFF_L        0x10
#define PCA9685_LED2_OFF_H        0x11
#define PCA9685_LED3_ON_L        0x12
#define PCA9685_LED3_ON_H        0x13
#define PCA9685_LED3_OFF_L        0x14
#define PCA9685_LED3_OFF_H        0x15
#define PCA9685_LED4_ON_L        0x16
#define PCA9685_LED4_ON_H        0x17
#define PCA9685_LED4_OFF_L        0x18
#define PCA9685_LED4_OFF_H        0x19
#define PCA9685_LED5_ON_L        0x1a
#define PCA9685_LED5_ON_H        0x1b
#define PCA9685_LED5_OFF_L        0x1c
#define PCA9685_LED5_OFF_H        0x1d
#define PCA9685_LED6_ON_L        0x1e
#define PCA9685_LED6_ON_H        0x1f
#define PCA9685_LED6_OFF_L        0x20
#define PCA9685_LED6_OFF_H        0x21
#define PCA9685_LED7_ON_L        0x22
#define PCA9685_LED7_ON_H        0x23
#define PCA9685_LED7_OFF_L        0x24
#define PCA9685_LED7_OFF_H        0x25
#define PCA9685_LED8_ON_L        0x26
#define PCA9685_LED8_ON_H        0x27
#define PCA9685_LED8_OFF_L        0x28
#define PCA9685_LED8_OFF_H        0x29
#define PCA9685_LED9_ON_L        0x2a
#define PCA9685_LED9_ON_H        0x2b
#define PCA9685_LED9_OFF_L        0x2c
#define PCA9685_LED9_OFF_H        0x2d
#define PCA9685_LED10_ON_L        0x2e
#define PCA9685_LED10_ON_H        0x2f
#define PCA9685_LED10_OFF_L        0x30
#define PCA9685_LED10_OFF_H        0x31
#define PCA9685_LED11_ON_L        0x32
#define PCA9685_LED11_ON_H        0x33
#define PCA9685_LED11_OFF_L        0x34
#define PCA9685_LED11_OFF_H        0x35
#define PCA9685_LED12_ON_L        0x36
#define PCA9685_LED12_ON_H        0x37
#define PCA9685_LED12_OFF_L        0x38
#define PCA9685_LED12_OFF_H        0x39
#define PCA9685_LED13_ON_L        0x3a
#define PCA9685_LED13_ON_H        0x3b
#define PCA9685_LED13_OFF_L        0x3c
#define PCA9685_LED13_OFF_H        0x3d
#define PCA9685_LED14_ON_L        0x3e
#define PCA9685_LED14_ON_H        0x3f
#define PCA9685_LED14_OFF_L        0x40
#define PCA9685_LED14_OFF_H        0x41
#define PCA9685_LED15_ON_L        0x42
#define PCA9685_LED15_ON_H        0x43
#define PCA9685_LED15_OFF_L        0x44
#define PCA9685_LED15_OFF_H        0x45
#define PCA9685_ALL_LED_ON_L       0xfa
#define PCA9685_ALL_LED_ON_H       0xfb
#define PCA9685_ALL_LED_OFF_L      0xfc
#define PCA9685_ALL_LED_OFF_H      0xfd
#define PCA9685_PRE_SCALE      0xfe     //要赋值为30，输出pwm频率200赫兹。

/* MODE1 bits */
#define PCA9685_RESTART    0x80  // 重启位，1重启，0无效。
/*如果PCA9685操作和用户决定把芯片睡觉（设置MODE1
第4位）没有停止任何的PWM通道，重启位（MODE1第7位）
的PWM刷新周期结束时，设置为逻辑1。每个PWM寄存器的内容是
持有有效的时脉关闭。
要重新启动所有以前的活动几I2C总线周期的PWM通道，做
步骤如下：
1。阅读MODE1寄存器。
2。检查的第7位（重新启动）为逻辑1。如果是的话，清晰的第4位（休眠）。等待一段时间
振荡器稳定（500微秒）。
3。下来撰写MODE1寄存器逻辑1到第的第7位。所有的PWM通道将重新启动，
RESTART位将清除。*/
#define PCA9685_EXTCLK    0x40   //晶振选择0内部1外部
#define PCA9685_AI    0x20       //寄存器自动增加位0不使用，1使用
#define PCA9685_SLEEP    0x10    //睡眠模式，0普通1睡眠晶振停止
#define PCA9685_SUB1    0x08     //0PCA9685不响应I2C总线子地址1。1PCA9685响应I2C总线子地址1。
#define PCA9685_SUB2    0x04     //0PCA9685不响应I2C总线子地址2。1PCA9685响应I2C总线子地址2?
#define PCA9685_SUB3    0x02     //0PCA9685不响应I2C总线子地址3。1PCA9685响应I2C总线子地址3?
#define PCA9685_ALLCALL    0x01  //0PCA9685不响应LED所有呼叫I2C总线地址,1PCA9685响应LED的所有呼叫I2C总线地址。



/* MODE2 bits */
#define PCA9685_INVRT    0x10    //0输出不翻转1输出反转。
#define PCA9685_OCH    0x08      //0输出改变在STOP命令，1 输出改变在ACK.应该理解是设置1，改变速度快。
#define PCA9685_OUTDRV    0x04   //0输出配置为开漏，1输出配置为输出配置图腾柱结构。
#define PCA9685_OUTNE1    0x02
#define PCA9685_OUTNE0    0x01   /*这里和OUTDRV配置有关若配置00，那么输出就是逻辑低(当OE=1时)，
                                  配置01若OUTDRV=1输出就是逻辑高,OUTDRV = 0，输出就是高阻(当OE=1时)。
                                  配置10和11输出都是高阻(当OE=1时)。
                                    00, *当OE = 1（输出驱动器未启用），LEDN=0。
                                    01，当OE = 1（输出驱动器未启用）：
                                        LEDN=1当OUTDRV= 1
                                        LEDN=，高阻抗OUTDRV=0（同为OUTNE[1:0] = 10）
                                    1X,当OE= 1（输出驱动器未启用），LEDN=高阻抗。*/

extern void PCA9685PWM_convert(void);
extern u8 AutoWritePCA9685(u8 tap[]);
extern u8 WritePCA9685REG_1(u8 address,u8 REG,u8 value);

#endif