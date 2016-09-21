#include "uscib0_i2c.h"
#include "BspDef.h"

#ifdef HW_I2C_DEV

#define I2C_SLAVE_ADDRESS  0x1E

void init_uscib0_i2c(void)
{
  P3SEL |= BIT1 | BIT2;                     //将P3.1和P3.2配置为UCB0SDA模式和UCB0SCL模式
  UCB0CTL1 |= UCSWRST;                     //使USCI保持在RESET状态
  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;    //将P3.1和P3.2设置为同步I2C通信主模式
  UCB0CTL1 = UCSSEL_2+UCSWRST;             //使用SMCLK作为时钟源并将USCI保持在RESET状态
  UCB0BR0 = 18;                            //fSCL = SMCLK/(UCBxBR0 + UCBxBR1 * 256) = 8M/(20 + 0 * 256) = 8M/80 ~= 100K
  UCB0BR1 = 0;
  UCB0I2CSA = I2C_SLAVE_ADDRESS;           //I2C从设备写地址为0x1C,I2C从设备读地址为0x1D
  UCB0CTL1 &= ~UCSWRST;                    //清除RESET状态位使系统恢复I2C通信
}

/*******************************************
函数名称：Ucb0I2c_Start(void)
功    能：I2C主机模式，发送写起始条件
参    数：无
         
返回值  ：无
********************************************/
void Ucb0I2c_Start(void)
{
   while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
   UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition
}

/*******************************************
函数名称：unsigned char Ucb0I2c_Read1byte(void)
功    能：I2C主机模式，读单个字节函数
参    数：无
         
返回值  ：读出的数
********************************************/
uint8_t Ucb0I2c_Read1byte(void)
{
  uint8_t Rdata;
   UCB0CTL1 &= ~UCTR ;                      // I2C 接收方式
   UCB0CTL1 |= UCTXSTT;                    // I2C start condition
  while(!(IFG2&UCB0RXIFG));                  //等待接收完成
  IFG2&=~UCB0RXIFG;
  UCB0CTL1 |= UCTXSTP;                   // I2C stop condition
  Rdata=UCB0RXBUF;
  return Rdata ;
}

/*******************************************
函数名称：void I2C_Write1byte(unsigned char wdata,unsigned int dataaddress,enum eepromtype EepromType)
功    能：I2C主机模式，写单个字节函数
参    数：wdata--要写的数；dataaddress--数据在设备中地址；EepromType--选用那个24XX
         
返回值  ：无
********************************************/
void I2C_Write1byte(uint8_t datareg,uint8_t data)
{
  uint8_t fWriteIIC = 1; 
  while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
  UCB0CTL1 |= UCTR;             // I2C TX
  UCB0CTL1 |= UCTXSTT;             //start condition
  __delay_cycles(150);          // 停止发出前要延时，否则直接发停止了，装载寄存器的步骤还是挺慢的
  UCB0TXBUF = datareg;                     // Load TX buffer
  while(UCB0CTL1&UCTXSTT);                  //等待应答
  IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag

  while((!(IFG2 & UCB0TXIFG)) && fWriteIIC)       //等待得到应答
  {
   if(UCB0STAT&UCNACKIFG)                     //没有得到应答处理
   {     
      fWriteIIC = 0;
      UCB0CTL1 |= UCTXSTP;                     // I2C stop condition
   }
  }                 //等待传送完成 TODO差非应答处理
  IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
  
  __delay_cycles(50);          // 两次数据传输中间也要延时，否则就出错，断了
  UCB0TXBUF = data;                     // Load TX buffer
  __delay_cycles(150);          // 停止发出前要延时，否则直接发停止了，装载寄存器的步骤还是挺慢的


  UCB0CTL1 |= UCTXSTP;                     // I2C stop condition
  while(UCB0CTL1&UCTXSTP);                  //等待应答
  IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
}

/*******************************************
函数名称：unsigned char I2C_Read1byte(unsigned int dataaddress,enum eepromtype EepromType)
功    能：I2C主机模式，读单个字节函数
参    数：datareg--数据在设备中地址
         
返回值  ：设备地址对应的数据
********************************************/
uint8_t I2C_Read1byte(uint8_t datareg)
{
    volatile uint8_t Rdata;
    Ucb0I2c_Start();
   //Ucb0I2c_Write1byte(datareg);          //写低8位数据地址
   Rdata=Ucb0I2c_Read1byte();
   
   return Rdata;
}

/*******************************************
函数名称：void I2C_ReadNbyte(unsigned char *index,unsigned char n,unsigned char datareg)
功    能：I2C主机模式，读N个字节函数
参    数：index--N个字节函数起始地址；n--要写的字节个数；datareg--写数据时设备中起始地址
         
返回值  ：无
********************************************/
void I2C_ReadNbyte(uint8_t *index,uint8_t n,uint8_t datareg)
{
    uint8_t i;
    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
    UCB0CTL1 |= UCTR;             // I2C TX
    UCB0CTL1 |= UCTXSTT;             //start condition
    __delay_cycles(150);          // 停止发出前要延时，否则直接发停止了，装载寄存器的步骤还是挺慢的
    UCB0TXBUF = datareg;                     // Load TX buffer
    while(UCB0CTL1&UCTXSTT);                  //等待应答
    IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
    __delay_cycles(50);          // 停止发出前要延时，否则直接发停止了，装载寄存器的步骤还是挺慢的

     *index=UCB0RXBUF;

    UCB0CTL1 &= ~UCTR ;                      // I2C 接收方式
    UCB0CTL1 |= UCTXSTT;                     // restart
    __delay_cycles(150);          // 开始发出后要延时等待一下
    while(UCB0CTL1&UCTXSTT);                     //等待应答
    
    for(i=0;i<n;i++)
    {
        while(!(IFG2&UCB0RXIFG));                  //等待接收完成
        *index=UCB0RXBUF;
        //    __delay_cycles(50);          // 
        IFG2&=~UCB0RXIFG;
        index++;    
        if(i == n-1)
        UCB0CTL1 |= UCTXSTP;                   // I2C stop condition
    }
    __delay_cycles(150);          // 开始发出后要延时等待一下
    while(UCB0CTL1&UCTXSTP);                     //等待应答
    IFG2&=~UCB0RXIFG;
}

#else

//#define DELAY_US_BSP(x) __delay_cycles((uint32_t)(((double)8000000)*(double)x/1000000.0))
#define BIT_DELAY_TIME(x) __delay_cycles(x)

void init_uscib0_i2c(void)
{
  SDA_SET_OUTPUT;
  SCL_SET_OUTPUT;
  SDA_OUTPUT_HIGH;
  SCL_OUTPUT_HIGH;
}

void uscib0_i2c_start(void)
{
  SDA_OUTPUT_HIGH;
  SCL_OUTPUT_HIGH;
  BIT_DELAY_TIME(DELAY_TIME);
  SDA_OUTPUT_LOW;
  BIT_DELAY_TIME(DELAY_TIME);
}

void uscib0_i2c_restart(void)
{
  SCL_OUTPUT_LOW;
  BIT_DELAY_TIME(DELAY_TIME);
  SDA_SET_OUTPUT;
  SDA_OUTPUT_HIGH;
  BIT_DELAY_TIME(DELAY_TIME);
  SCL_OUTPUT_HIGH;
  BIT_DELAY_TIME(DELAY_TIME);
  SDA_OUTPUT_LOW;
  BIT_DELAY_TIME(DELAY_TIME);
}

void uscib0_i2c_stop(void)
{
  SCL_OUTPUT_LOW;
  BIT_DELAY_TIME(DELAY_TIME);
  SDA_OUTPUT_LOW;
  BIT_DELAY_TIME(DELAY_TIME);
  SCL_OUTPUT_HIGH;
  BIT_DELAY_TIME(DELAY_TIME);
  SDA_OUTPUT_HIGH;
  BIT_DELAY_TIME(DELAY_TIME);
}

uint8_t uscib0_i2c_get_ack(void)
{
  uint8_t btErrorBit;
  SCL_OUTPUT_LOW;
  BIT_DELAY_TIME(DELAY_TIME);
  SDA_SET_INPUT;
  BIT_DELAY_TIME(DELAY_TIME);
  SCL_OUTPUT_HIGH;
  BIT_DELAY_TIME(DELAY_TIME);
  btErrorBit = SDA_DATA_IN;
  return btErrorBit;
}

void uscib0_i2c_ack(void)
{
   SDA_OUTPUT_LOW;
   BIT_DELAY_TIME(DELAY_TIME);
   SCL_OUTPUT_HIGH;
   BIT_DELAY_TIME(DELAY_TIME);
   SCL_OUTPUT_LOW;
   BIT_DELAY_TIME(DELAY_TIME);
}

void uscib0_i2c_nack(void) 
{
   SDA_OUTPUT_HIGH;
   BIT_DELAY_TIME(DELAY_TIME);
   SCL_OUTPUT_HIGH;
   BIT_DELAY_TIME(DELAY_TIME);
   SCL_OUTPUT_LOW;
   BIT_DELAY_TIME(DELAY_TIME);
}

uint8_t uscib0_i2c_write_byte(uint8_t btData) 
{
  uint8_t btDataBitCount = 0;
  SDA_SET_OUTPUT;
  for(btDataBitCount = 0; btDataBitCount < 8; btDataBitCount++)
  {
    SCL_OUTPUT_LOW;
    BIT_DELAY_TIME(DELAY_TIME);
    if(btData & 0x80)
      SDA_OUTPUT_HIGH;
    else
      SDA_OUTPUT_LOW;
    BIT_DELAY_TIME(DELAY_TIME);
    SCL_OUTPUT_HIGH;
    BIT_DELAY_TIME(DELAY_TIME);
    btData = btData << 1;
  }
  return 0;
}

uint8_t uscib0_i2c_read_byte(void)
{
  uint8_t btSDADataIn = 0, btData = 0, btDataBitCount = 0;
  SCL_OUTPUT_LOW;
  SDA_SET_INPUT;
  BIT_DELAY_TIME(DELAY_TIME);
  for(btDataBitCount = 0; btDataBitCount < 8;btDataBitCount++)  //目前MSB最高位为无用位，原因尚待进一步调试，目前采用的策略是抛弃MSB最高位
  {
    SCL_OUTPUT_HIGH;
    BIT_DELAY_TIME(DELAY_TIME);
    btSDADataIn = SDA_DATA_IN;
    btSDADataIn >>= (SDA_DATA_BIT -1);
    btData = btData << 1;
    btData = btData | btSDADataIn;
    BIT_DELAY_TIME(DELAY_TIME);
    SCL_OUTPUT_LOW;
    BIT_DELAY_TIME(DELAY_TIME);
  }
  SDA_SET_OUTPUT;
  BIT_DELAY_TIME(DELAY_TIME);
  SCL_OUTPUT_HIGH;
  BIT_DELAY_TIME(DELAY_TIME);
  return btData;     
}

#endif