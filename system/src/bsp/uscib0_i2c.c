#include "uscib0_i2c.h"
#include "BspDef.h"

#ifdef HW_I2C_DEV

#define I2C_SLAVE_ADDRESS  0x1E

void init_uscib0_i2c(void)
{
  P3SEL |= BIT1 | BIT2;                     //��P3.1��P3.2����ΪUCB0SDAģʽ��UCB0SCLģʽ
  UCB0CTL1 |= UCSWRST;                     //ʹUSCI������RESET״̬
  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;    //��P3.1��P3.2����Ϊͬ��I2Cͨ����ģʽ
  UCB0CTL1 = UCSSEL_2+UCSWRST;             //ʹ��SMCLK��Ϊʱ��Դ����USCI������RESET״̬
  UCB0BR0 = 18;                            //fSCL = SMCLK/(UCBxBR0 + UCBxBR1 * 256) = 8M/(20 + 0 * 256) = 8M/80 ~= 100K
  UCB0BR1 = 0;
  UCB0I2CSA = I2C_SLAVE_ADDRESS;           //I2C���豸д��ַΪ0x1C,I2C���豸����ַΪ0x1D
  UCB0CTL1 &= ~UCSWRST;                    //���RESET״̬λʹϵͳ�ָ�I2Cͨ��
}

/*******************************************
�������ƣ�Ucb0I2c_Start(void)
��    �ܣ�I2C����ģʽ������д��ʼ����
��    ������
         
����ֵ  ����
********************************************/
void Ucb0I2c_Start(void)
{
   while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
   UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition
}

/*******************************************
�������ƣ�unsigned char Ucb0I2c_Read1byte(void)
��    �ܣ�I2C����ģʽ���������ֽں���
��    ������
         
����ֵ  ����������
********************************************/
uint8_t Ucb0I2c_Read1byte(void)
{
  uint8_t Rdata;
   UCB0CTL1 &= ~UCTR ;                      // I2C ���շ�ʽ
   UCB0CTL1 |= UCTXSTT;                    // I2C start condition
  while(!(IFG2&UCB0RXIFG));                  //�ȴ��������
  IFG2&=~UCB0RXIFG;
  UCB0CTL1 |= UCTXSTP;                   // I2C stop condition
  Rdata=UCB0RXBUF;
  return Rdata ;
}

/*******************************************
�������ƣ�void I2C_Write1byte(unsigned char wdata,unsigned int dataaddress,enum eepromtype EepromType)
��    �ܣ�I2C����ģʽ��д�����ֽں���
��    ����wdata--Ҫд������dataaddress--�������豸�е�ַ��EepromType--ѡ���Ǹ�24XX
         
����ֵ  ����
********************************************/
void I2C_Write1byte(uint8_t datareg,uint8_t data)
{
  uint8_t fWriteIIC = 1; 
  while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
  UCB0CTL1 |= UCTR;             // I2C TX
  UCB0CTL1 |= UCTXSTT;             //start condition
  __delay_cycles(150);          // ֹͣ����ǰҪ��ʱ������ֱ�ӷ�ֹͣ�ˣ�װ�ؼĴ����Ĳ��軹��ͦ����
  UCB0TXBUF = datareg;                     // Load TX buffer
  while(UCB0CTL1&UCTXSTT);                  //�ȴ�Ӧ��
  IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag

  while((!(IFG2 & UCB0TXIFG)) && fWriteIIC)       //�ȴ��õ�Ӧ��
  {
   if(UCB0STAT&UCNACKIFG)                     //û�еõ�Ӧ����
   {     
      fWriteIIC = 0;
      UCB0CTL1 |= UCTXSTP;                     // I2C stop condition
   }
  }                 //�ȴ�������� TODO���Ӧ����
  IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
  
  __delay_cycles(50);          // �������ݴ����м�ҲҪ��ʱ������ͳ�������
  UCB0TXBUF = data;                     // Load TX buffer
  __delay_cycles(150);          // ֹͣ����ǰҪ��ʱ������ֱ�ӷ�ֹͣ�ˣ�װ�ؼĴ����Ĳ��軹��ͦ����


  UCB0CTL1 |= UCTXSTP;                     // I2C stop condition
  while(UCB0CTL1&UCTXSTP);                  //�ȴ�Ӧ��
  IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
}

/*******************************************
�������ƣ�unsigned char I2C_Read1byte(unsigned int dataaddress,enum eepromtype EepromType)
��    �ܣ�I2C����ģʽ���������ֽں���
��    ����datareg--�������豸�е�ַ
         
����ֵ  ���豸��ַ��Ӧ������
********************************************/
uint8_t I2C_Read1byte(uint8_t datareg)
{
    volatile uint8_t Rdata;
    Ucb0I2c_Start();
   //Ucb0I2c_Write1byte(datareg);          //д��8λ���ݵ�ַ
   Rdata=Ucb0I2c_Read1byte();
   
   return Rdata;
}

/*******************************************
�������ƣ�void I2C_ReadNbyte(unsigned char *index,unsigned char n,unsigned char datareg)
��    �ܣ�I2C����ģʽ����N���ֽں���
��    ����index--N���ֽں�����ʼ��ַ��n--Ҫд���ֽڸ�����datareg--д����ʱ�豸����ʼ��ַ
         
����ֵ  ����
********************************************/
void I2C_ReadNbyte(uint8_t *index,uint8_t n,uint8_t datareg)
{
    uint8_t i;
    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
    UCB0CTL1 |= UCTR;             // I2C TX
    UCB0CTL1 |= UCTXSTT;             //start condition
    __delay_cycles(150);          // ֹͣ����ǰҪ��ʱ������ֱ�ӷ�ֹͣ�ˣ�װ�ؼĴ����Ĳ��軹��ͦ����
    UCB0TXBUF = datareg;                     // Load TX buffer
    while(UCB0CTL1&UCTXSTT);                  //�ȴ�Ӧ��
    IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
    __delay_cycles(50);          // ֹͣ����ǰҪ��ʱ������ֱ�ӷ�ֹͣ�ˣ�װ�ؼĴ����Ĳ��軹��ͦ����

     *index=UCB0RXBUF;

    UCB0CTL1 &= ~UCTR ;                      // I2C ���շ�ʽ
    UCB0CTL1 |= UCTXSTT;                     // restart
    __delay_cycles(150);          // ��ʼ������Ҫ��ʱ�ȴ�һ��
    while(UCB0CTL1&UCTXSTT);                     //�ȴ�Ӧ��
    
    for(i=0;i<n;i++)
    {
        while(!(IFG2&UCB0RXIFG));                  //�ȴ��������
        *index=UCB0RXBUF;
        //    __delay_cycles(50);          // 
        IFG2&=~UCB0RXIFG;
        index++;    
        if(i == n-1)
        UCB0CTL1 |= UCTXSTP;                   // I2C stop condition
    }
    __delay_cycles(150);          // ��ʼ������Ҫ��ʱ�ȴ�һ��
    while(UCB0CTL1&UCTXSTP);                     //�ȴ�Ӧ��
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
  for(btDataBitCount = 0; btDataBitCount < 8;btDataBitCount++)  //ĿǰMSB���λΪ����λ��ԭ���д���һ�����ԣ�Ŀǰ���õĲ���������MSB���λ
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