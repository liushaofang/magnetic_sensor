#include "uscib0_i2c.h"
#include "hmc5883l.h"

#define RAW_MAGNET_DATA_LEN         6
#define MAGNET_DATA_BUF_SIZE        32
#define MAGNET_DATA_BUF_SIZE_BITS   5   /*      表示计算地磁平均值时通过移位运算代替除法运算的位数，需要和上一个宏同时修改   */

//2015.04.12:liusf add to optimize traffic detection algorithm
#define REFERENCE_MAGNET_DATA_BUF_SIZE             8
#define REFERENCE_MAGNET_DATA_BUF_SIZE_BITS        3       /*      取32个地磁数据缓冲区中的前8个作为地磁数据缓冲区基准值  */
static uint8_t g_nReferenceMagnetDataStartIndex = 0;
//2015.04.12:liusf add end

#ifdef HW_I2C_DEV

static void init_uscib0_hmc5883l_int(void);
static void Single_Write_HMC5883(int8_t Address,int8_t Dat);
static void Multiple_Read_HMC5883(void);

/*      记录地磁传感器数据原始数据值  */
volatile int8_t gRawMagnetDataBuf[RAW_MAGNET_DATA_LEN] = { 0 };
/*      背景磁场采样次数统计值   */
static volatile uint8_t g_background_magnetic_sample_count = 0;
/*      标识是否停止地磁传感器标志位  */
static volatile uint8_t g_suspend_magnet_flag = 0;
/*      标识地磁传感器工作状态 1表示正在工作，0表示停止       */
static volatile uint8_t g_is_magnet_sampling = 1;
/*      记录地磁传感器X轴地磁数据背景磁场值缓冲区      */
static int16_t g_x_magnet_data_buf[MAGNET_DATA_BUF_SIZE] = { 0 };
/*      记录地磁传感器Y轴地磁数据背景磁场值缓冲区      */
static int16_t g_y_magnet_data_buf[MAGNET_DATA_BUF_SIZE] = { 0 };
/*      记录地磁传感器Z轴地磁数据背景磁场值缓冲区      */
static int16_t g_z_magnet_data_buf[MAGNET_DATA_BUF_SIZE] = { 0 };
/*      记录地磁传感器地磁数据变化量可容忍的抖动门限值      */
int16_t g_low_delta_threshold = -20;   //15;
/*      记录地磁传感器地磁数据变化量不可容忍的抖动门限值      */
int16_t g_high_delta_threshold = 20;   //210;
/*      记录当前需要更新的背景磁场数据索引       */
static uint8_t g_n_cur_data_buf_index = 0;
/*      统计目前地磁数据增加抖动变化次数  */
uint8_t g_n_inc_fluctuation_count = 0;
/*      记录目前地磁数据增加抖动变化次数表示有车的界限  */
static uint8_t g_n_inc_fluctuation_limit = 2;
/*      统计目前地磁数据减少抖动变化次数  */
uint8_t g_n_dec_fluctuation_count = 0;
/*      记录目前地磁数据增加抖动变化次数表示无车的界限  */
static uint8_t g_n_dec_fluctuation_limit = 2;
/*      统计目前地磁数据在抖动门限值内的变化次数  */
uint8_t g_n_no_fluctuation_count = 0;
/*      记录目前地磁数据在抖动门限值内的界限  */
static uint8_t g_n_no_fluctuation_limit = 2;

static int16_t g_n_prev_x_magnet_value = 0;

uint8_t g_nDeltaXYZData = 0;  //2014.07.06:liusf add for magnet test
uint16_t g_u16DeltaXYZData = 0;  //2014.07.06:liusf add for magnet test
/*      记录是否有车标志        */
uint8_t g_is_car_parked = 0;

//初始化HMC5883，根据需要请参考pdf进行修改****
int init_mag_sensor(void)
{
    __delay_cycles(150000);
    //初始化HMC5883L中断
    init_uscib0_hmc5883l_int();
    //初始化MSP430硬件I2C
    init_uscib0_i2c();
    //初始化HMC5883L模块
    //Single_Write_HMC5883(0x00,0x60); 
    Single_Write_HMC5883(0x00,0x18);       //75HZ，目前工作正常
    //Single_Write_HMC5883(0x00,0x14);      //30HZ，目前工作正常
    //Single_Write_HMC5883(0x00,0x10);      //15HZ，目前工作正常
    //Single_Write_HMC5883(0x00,0x0C);      //7.5HZ
    //Single_Write_HMC5883(0x00,0x08);      //3HZ，目前工作正常
    //Single_Write_HMC5883(0x00,0x04);      //1.5HZ，目前工作正常
    //Single_Write_HMC5883(0x00,0x0);      //0.75HZ，目前工作正常
    
//    Single_Write_HMC5883(MODE_REGISTER,MODE_IDLE);        //将HMC5883L地磁传感器配置为空闲模式
    Single_Write_HMC5883(0x02,0x00);
    //HMC5883_Send_Byte(0x00);
    __delay_cycles(150000);
    
    //g_magnetic_sample_count = 0;
    //reset_background_magnetic();
    
    return SUCCESS;
}

void init_uscib0_hmc5883l_int(void)
{
  P2SEL &= ~P2BIT_MAGNET_BIT; //将P2.3设置为I/O功能模式
  P2DIR &= ~P2BIT_MAGNET_BIT; //将P2.3配置为输入模式
  P2REN &= ~P2BIT_MAGNET_BIT; //禁止内部电阻
  P2IES |= P2BIT_MAGNET_BIT; //将P2.3设置为从高电平到低电平的沿触发模式
  P2IFG &= ~P2BIT_MAGNET_BIT; //清除P2.3中断标志位
  P2IE |= P2BIT_MAGNET_BIT;   //使能P2.3端口中断
}

void disable_hmc5883l_int(void)
{
  P2IE &= ~P2BIT_MAGNET_BIT;   //禁止P2.3端口地磁中断
}

void enable_hmc5883l_int(void)
{
  P2IE |= P2BIT_MAGNET_BIT;   //使能P2.3端口地磁中断
}

/*单字节写HMC5883*/
static void Single_Write_HMC5883(int8_t Address,int8_t Dat)
{
    I2C_Write1byte(Address,Dat); 
}

/*多字节读HMC5883*/
void Multiple_Read_HMC5883(void)
{
    I2C_ReadNbyte((uint8_t *)gRawMagnetDataBuf,RAW_MAGNET_DATA_LEN,OUT_X_MSB_ADDR);
}

/* 将HMC5883L地磁传感器设置为IDLE模式        */
__monitor void suspend_hmc5883l(void)
{
	CLEAR_MAGNET_INT_FLAG();
	Single_Write_HMC5883(MODE_REGISTER,MODE_IDLE);        //将HMC5883L地磁传感器配置为空闲模式
        //__delay_cycles(150000);       //等待若干个时钟周期
	g_is_magnet_sampling = 0;
}

/* 恢复HMC5883L地磁传感器连续采样模式        */
__monitor uint8_t resume_hmc5883l(void)
{
	CLEAR_MAGNET_INT_FLAG();
	Single_Write_HMC5883(MODE_REGISTER, MODE_CONTINOUS_MEASUREMENT); //将HMC5883L地磁传感器配置为连续采样模式
	//__delay_cycles(150000);      //等待若干个时钟周期
	g_is_magnet_sampling = 1;
	return SUCCESS;
}

__monitor uint8_t set_hmc5883l_odr(uint8_t nOutputDataRate)
{
	CLEAR_MAGNET_INT_FLAG();
	Single_Write_HMC5883(0x00,nOutputDataRate); //将HMC5883L地磁传感器采样速率配置为nOutputDataRate
	//__delay_cycles(150000);      //等待若干个时钟周期
	return SUCCESS;
}

/* 该函数在调用之前应该将全局变量g_background_magnetic_sample_count置为初始值0，并且使能全局中断和地磁中断    */
void reset_background_magnetic(void)
{
    uint8_t nIndex = 0;
    
    int8_t btXDataMSB = 0;
    uint8_t btXDataLSB = 0;
    int8_t btYDataMSB = 0;
    uint8_t btYDataLSB = 0;
    int8_t btZDataMSB = 0;
    uint8_t btZDataLSB = 0;
    
    int16_t n16XData = 0;
    int16_t n16YData = 0;
    int16_t n16ZData = 0;
    
    CLEAR_MAGNET_INT_FLAG();
    g_background_magnetic_sample_count = 0;
    
    while(g_is_magnet_sampling && (g_background_magnetic_sample_count < 3 * MAGNET_DATA_BUF_SIZE))  
    {
        FEED_WDT;
        
        btXDataMSB = gRawMagnetDataBuf[0];
        btXDataLSB = gRawMagnetDataBuf[1];
        btYDataMSB = gRawMagnetDataBuf[2];
        btYDataLSB = gRawMagnetDataBuf[3];
        btZDataMSB = gRawMagnetDataBuf[4];
        btZDataLSB = gRawMagnetDataBuf[5];
        
        n16XData = (int16_t) (btXDataMSB << 7) + (int16_t) (btXDataLSB >> 1);
        n16XData = (n16XData < -1024)?-1024:n16XData;
        n16XData = (n16XData > 1024)?1024:n16XData;
        n16YData = (int16_t) (btYDataMSB << 7) + (int16_t) (btYDataLSB >> 1);
        n16YData = (n16YData < -1024)?-1024:n16YData;
        n16YData = (n16YData > 1024)?1024:n16YData;
        n16ZData = (int16_t) (btZDataMSB << 7) + (int16_t) (btZDataLSB >> 1);
        n16ZData = (n16ZData < -1024)?-1024:n16ZData;
        n16ZData = (n16ZData > 1024)?1024:n16ZData;
        
        nIndex = g_background_magnetic_sample_count % MAGNET_DATA_BUF_SIZE;
        g_x_magnet_data_buf[nIndex] = n16XData;
        g_y_magnet_data_buf[nIndex] = n16YData;
        g_z_magnet_data_buf[nIndex] = n16ZData;
        
        DISABLE_WDT;    //2015.02.01:liusf add watchdog function
        LPM3;
    }
}

void set_threshold_values(uint8_t uSetFlag, int16_t n_high_threshold_new_value, int16_t n_low_threshold_new_value, uint8_t n_inc_fluctuation_limit, uint8_t n_dec_fluctuation_limit)
{
  if(uSetFlag & SET_HIGH_THRESHOLD)
  {
    g_high_delta_threshold = n_high_threshold_new_value;
  }
  if(uSetFlag & SET_LOW_THRESHOLD)
  {
    g_low_delta_threshold = n_low_threshold_new_value;
  }
  if(uSetFlag & SET_INC_COUNT_LIMIT)
  {
    g_n_inc_fluctuation_limit = n_inc_fluctuation_limit;
  }
  if(uSetFlag & SET_DEC_COUNT_LIMIT)
  {
    g_n_dec_fluctuation_limit = n_dec_fluctuation_limit;
  }
}

#if 1

uint16_t getValue(uint8_t volatile * pCurFlucState, uint8_t * raw_data, uint8_t nRawDataLen) 
{
	int8_t nIndex = 0;
        
        /*      地磁传感器X轴低字节原始数据与高字节原始数据变量  */
        int8_t btXDataMSB = 0;
        uint8_t btXDataLSB = 0;
        
        /*      地磁传感器X轴低字节右移1位和高字节左移7位得到的地磁数据值    */
        int16_t n16XData = 0;
        
        /*      地磁传感器数据X轴数据计算和    */
        int16_t n16XDataSum = 0;
        
        /*      根据地磁传感器地磁数据计算得到的背景磁场值   */
        int16_t n16XRefValue = 0;
        
        /*      记录地磁传感器X轴的地磁数据变化量 */
        int16_t n16DeltaXData = 0;
        
        /*      对传入函数的参数合法性进行验证 */
        if (pCurFlucState == 0 || raw_data == 0 || nRawDataLen < 6)
		return -1;
        
        /*      将由地磁传感器中断处理函数中获取的地磁数据原始值通过参数传出    */
        raw_data[0] = gRawMagnetDataBuf[0]; //获取X轴数据高字节
        raw_data[1] = gRawMagnetDataBuf[1]; //获取X轴数据低字节
        raw_data[2] = gRawMagnetDataBuf[2]; //获取Y轴数据高字节
        raw_data[3] = gRawMagnetDataBuf[3]; //获取Y轴数据低字节
        raw_data[4] = gRawMagnetDataBuf[4]; //获取Z轴数据高字节
        raw_data[5] = gRawMagnetDataBuf[5]; //获取Z轴数据低字节
        
        /*   地磁传感器X轴、Y轴、Z轴低字节原始数据与高字节原始数据变量     */
        btXDataMSB = gRawMagnetDataBuf[0];
        btXDataLSB = gRawMagnetDataBuf[1];
        
        /*      根据地磁传感器数据原始值通过将低低字节右移1位、将高字节左移7位得到X轴、Y轴和Z轴整体数据右移1位的地磁数据值        */
        n16XData = (int16_t) (btXDataMSB << 7) + (int16_t) (btXDataLSB >> 1);
        g_n_prev_x_magnet_value = n16XData;
        /*      增加对当前地磁值为-4096，采样结果为4096的情况的处理，同时对当前地磁值为4096，采样结果为-4096的情况的处理        */
        if(n16XData == -2048 && g_n_prev_x_magnet_value > 0)
        {
          n16XData = 2048;
          g_n_prev_x_magnet_value = 2048;
        }
        if(n16XData == 2048 && g_n_prev_x_magnet_value < 0)
        {
          n16XData = -2048;
          g_n_prev_x_magnet_value = -2048;
        }
        n16XData = (n16XData < -1024)?-1024:n16XData;
        n16XData = (n16XData > 1024)?1024:n16XData;
        
        /*      根据背景地磁数据计算参考地磁数据，将X轴、Y轴和Z轴的背景地磁数据值相加求和并且求平均值得到参考地磁数据值     */
        for(n16XDataSum = 0, nIndex = 0; nIndex < REFERENCE_MAGNET_DATA_BUF_SIZE; nIndex++)
        {
          n16XDataSum += g_x_magnet_data_buf[(g_nReferenceMagnetDataStartIndex + nIndex) % MAGNET_DATA_BUF_SIZE];
        }
        n16XRefValue = (int16_t)(n16XDataSum >> REFERENCE_MAGNET_DATA_BUF_SIZE_BITS);
        
        /*      计算X轴实际地磁数据值与背景地磁数据值的差值       */
        n16DeltaXData = n16XData - n16XRefValue;
        /*      X轴、Y轴与Z轴地磁数据变化量的绝对值的和小于变化门限，表示背景磁场发生变化，需要更新背景地磁数据值      */
        if(n16DeltaXData >= g_low_delta_threshold && n16DeltaXData <= g_high_delta_threshold)
        {
            g_n_no_fluctuation_count = 0;
            g_n_inc_fluctuation_count = 0;
            g_n_dec_fluctuation_count = 0;
            * pCurFlucState = FLUCTUATION_NONE;
            //2015.04.12:liusf add to update reference magnetic data
            g_x_magnet_data_buf[g_n_cur_data_buf_index] = n16XData;
            g_n_cur_data_buf_index = (g_n_cur_data_buf_index + 1) % MAGNET_DATA_BUF_SIZE;
            g_nReferenceMagnetDataStartIndex = (g_nReferenceMagnetDataStartIndex + 1) % MAGNET_DATA_BUF_SIZE;
            //2015.04.12:liusf add end
        }
        else if(n16DeltaXData > g_high_delta_threshold)
        {
          g_n_inc_fluctuation_count++;
          /*    实际地磁数据值连续若干次与背景地磁数据值的差值小于一定门限，表示目前没有车辆  */
          if(g_n_inc_fluctuation_count > g_n_inc_fluctuation_limit)
          {
            g_n_no_fluctuation_count = 0;
            g_n_inc_fluctuation_count = 0;
            g_n_dec_fluctuation_count = 0;
            * pCurFlucState = FLUCTUATION_UPWARD;
          }
        }
        else if(n16DeltaXData < g_low_delta_threshold)
        {
          g_n_dec_fluctuation_count++;
          /*    实际地磁数据值连续若干次与背景地磁数据值的差值小于一定门限，表示目前没有车辆  */
          if(g_n_dec_fluctuation_count > g_n_dec_fluctuation_limit)
          {
            g_n_no_fluctuation_count = 0;
            g_n_inc_fluctuation_count = 0;
            g_n_dec_fluctuation_count = 0;
            * pCurFlucState = FLUCTUATION_DOWNWARD;
          }
        }
        
	return 0;
}

#else

uint16_t getValue(uint8_t volatile * pIsCarParked, uint8_t *raw_data, uint8_t nRawDataLen) 
{
	int8_t nIndex = 0;
        
        /*      地磁传感器X轴、Y轴、Z轴低字节原始数据与高字节原始数据变量  */
        int8_t btXDataMSB = 0;
        uint8_t btXDataLSB = 0;
        int8_t btYDataMSB = 0;
        uint8_t btYDataLSB = 0;
        int8_t btZDataMSB = 0;
        uint8_t btZDataLSB = 0;
        
        /*      地磁传感器X轴、Y轴、Z轴低字节右移1位和高字节左移7位得到的地磁数据值    */
        int16_t n16XData = 0;
        int16_t n16YData = 0;
        int16_t n16ZData = 0;
        
        /*      地磁传感器数据X轴、Y轴和Z轴数据计算和    */
        int16_t n16XDataSum = 0;
        int16_t n16YDataSum = 0;
        int16_t n16ZDataSum = 0;
        
        /*      根据地磁传感器地磁数据计算得到的背景磁场值   */
        int16_t n16XRefValue = 0;
        int16_t n16YRefValue = 0;
        int16_t n16ZRefValue = 0;
        
        /*      记录地磁传感器X轴、Y轴和Z轴的地磁数据变化量及变化量之和 */
        int16_t n16DeltaXData = 0;
        int16_t n16DeltaYData = 0;
        int16_t n16DeltaZData = 0;
        int16_t n16DeltaXYZData = 0;
        
        /*      对传入函数的参数合法性进行验证 */
        if (pIsCarParked == 0 || raw_data == 0 || nRawDataLen < 6)
		return -1;
        
        /*      将由地磁传感器中断处理函数中获取的地磁数据原始值通过参数传出    */
        raw_data[0] = gRawMagnetDataBuf[0]; //获取X轴数据高字节
        raw_data[1] = gRawMagnetDataBuf[1]; //获取X轴数据低字节
        raw_data[2] = gRawMagnetDataBuf[2]; //获取Y轴数据高字节
        raw_data[3] = gRawMagnetDataBuf[3]; //获取Y轴数据低字节
        raw_data[4] = gRawMagnetDataBuf[4]; //获取Z轴数据高字节
        raw_data[5] = gRawMagnetDataBuf[5]; //获取Z轴数据低字节
        
        /*   地磁传感器X轴、Y轴、Z轴低字节原始数据与高字节原始数据变量     */
        btXDataMSB = gRawMagnetDataBuf[0];
        btXDataLSB = gRawMagnetDataBuf[1];
        btYDataMSB = gRawMagnetDataBuf[2];
        btYDataLSB = gRawMagnetDataBuf[3];
        btZDataMSB = gRawMagnetDataBuf[4];
        btZDataLSB = gRawMagnetDataBuf[5];
        
        /*      根据地磁传感器数据原始值通过将低低字节右移1位、将高字节左移7位得到X轴、Y轴和Z轴整体数据右移1位的地磁数据值        */
        n16XData = (int16_t) (btXDataMSB << 7) + (int16_t) (btXDataLSB >> 1);
        n16XData = (n16XData < -1024)?-1024:n16XData;
        n16XData = (n16XData > 1024)?1024:n16XData;
        n16YData = (int16_t) (btYDataMSB << 7) + (int16_t) (btYDataLSB >> 1);
        n16YData = (n16YData < -1024)?-1024:n16YData;
        n16YData = (n16YData > 1024)?1024:n16YData;
        n16ZData = (int16_t) (btZDataMSB << 7) + (int16_t) (btZDataLSB >> 1);
        n16ZData = (n16ZData < -1024)?-1024:n16ZData;
        n16ZData = (n16ZData > 1024)?1024:n16ZData;
        
        /*      根据背景地磁数据计算参考地磁数据，将X轴、Y轴和Z轴的背景地磁数据值相加求和并且求平均值得到参考地磁数据值     */
        for(n16XDataSum = 0, n16YDataSum = 0, n16ZDataSum = 0, nIndex = 0; nIndex < MAGNET_DATA_BUF_SIZE; nIndex++)
        {
          n16XDataSum += g_x_magnet_data_buf[nIndex];
          n16YDataSum += g_y_magnet_data_buf[nIndex];
          n16ZDataSum += g_z_magnet_data_buf[nIndex];
        }
        /*n16XRefValue = (int16_t)(n16XDataSum / MAGNET_DATA_BUF_SIZE);
        n16YRefValue = (int16_t)(n16YDataSum / MAGNET_DATA_BUF_SIZE);
        n16ZRefValue = (int16_t)(n16ZDataSum / MAGNET_DATA_BUF_SIZE);*/
        n16XRefValue = (int16_t)(n16XDataSum >> MAGNET_DATA_BUF_SIZE_BITS);
        n16YRefValue = (int16_t)(n16YDataSum >> MAGNET_DATA_BUF_SIZE_BITS);
        n16ZRefValue = (int16_t)(n16ZDataSum >> MAGNET_DATA_BUF_SIZE_BITS);
        
        /*      计算实际地磁数据值与背景地磁数据值的差值的绝对值        */
        n16DeltaXData = (n16XData > n16XRefValue)?(n16XData - n16XRefValue):(n16XRefValue - n16XData);
        n16DeltaYData = (n16YData > n16YRefValue)?(n16YData - n16YRefValue):(n16YRefValue - n16YData);
        n16DeltaZData = (n16ZData > n16ZRefValue)?(n16ZData - n16ZRefValue):(n16ZRefValue - n16ZData);
        
        /*      计算X轴、Y轴和Z轴地磁数据值与背景地磁数据值差值绝对值的和  */
        n16DeltaXYZData = n16DeltaXData + n16DeltaYData + n16DeltaZData;
        
        g_u16DeltaXYZData = n16DeltaXYZData;
        g_nDeltaXYZData = (uint8_t)(n16DeltaXYZData & 0xFF);    //2014.07.06:liusf add
        
        /*      X轴、Y轴与Z轴地磁数据变化量的绝对值的和小于变化门限，表示背景磁场发生变化，需要更新背景地磁数据值      */
        if(n16DeltaXYZData <= g_low_delta_threshold)
        {
          /*    无车情况地磁变化小于阈值，直接更新地磁数据，有车时需要等到无车时再更新地磁数据   */
            //2014.07.06: liusf comment it off to delete manetic update
//          if(g_is_car_parked == 0)
//          {
//            g_x_magnet_data_buf[g_n_cur_data_buf_index] = n16XData;
//            g_y_magnet_data_buf[g_n_cur_data_buf_index] = n16YData;
//            g_z_magnet_data_buf[g_n_cur_data_buf_index] = n16ZData;
//            g_n_cur_data_buf_index = (g_n_cur_data_buf_index + 1) % MAGNET_DATA_BUF_SIZE;
//          }
          g_n_dec_fluctuation_count++;
          /*    实际地磁数据值连续若干次与背景地磁数据值的差值小于一定门限，表示目前没有车辆  */
          if(g_n_dec_fluctuation_count > g_n_dec_fluctuation_limit)
          {
            g_n_inc_fluctuation_count = 0;
            g_n_dec_fluctuation_count = 0;
            g_is_car_parked = 0;
            * pIsCarParked = 0;
          }
        }
        else if(n16DeltaXYZData >= g_high_delta_threshold)      /*      X轴、Y轴与Z轴地磁数据变化量的绝对值的和大于变化门限值，表示有车或者抖动导致地磁数据变化超过门限值      */
        {
          g_n_inc_fluctuation_count++;
          /*    实际地磁数据值连续若干次与背景地磁数据值的差值大于一定门限，表示目前有车辆  */
          if(g_n_inc_fluctuation_count > g_n_inc_fluctuation_limit)
          {
            g_n_inc_fluctuation_count = 0;
            g_n_dec_fluctuation_count = 0;
            g_is_car_parked = 1;
            * pIsCarParked = 1;
          }
        }
        
	return 0;
}

#endif

#ifdef MAG_TEST
uint16_t getMagTestValue(uint8_t *mag_data, uint8_t nMagDataLen, uint8_t * bg_mag_data, uint8_t nBgMagDataLen, int16_t * n16VariantData) 
{
	int8_t nIndex = 0;
        
        /*      地磁传感器X轴、Y轴、Z轴低字节原始数据与高字节原始数据变量  */
        int8_t btXDataMSB = 0;
        uint8_t btXDataLSB = 0;
        int8_t btYDataMSB = 0;
        uint8_t btYDataLSB = 0;
        int8_t btZDataMSB = 0;
        uint8_t btZDataLSB = 0;
        
        /*      地磁传感器X轴、Y轴、Z轴低字节右移1位和高字节左移7位得到的地磁数据值    */
        int16_t n16XData = 0;
        int16_t n16YData = 0;
        int16_t n16ZData = 0;
        
        /*      地磁传感器数据X轴、Y轴和Z轴数据计算和    */
        int16_t n16XDataSum = 0;
        int16_t n16YDataSum = 0;
        int16_t n16ZDataSum = 0;
        
        /*      根据地磁传感器地磁数据计算得到的背景磁场值   */
        int16_t n16XRefValue = 0;
        int16_t n16YRefValue = 0;
        int16_t n16ZRefValue = 0;
        
        /*      记录地磁传感器X轴、Y轴和Z轴的地磁数据变化量及变化量之和 */
        int16_t n16DeltaXData = 0;
        int16_t n16DeltaYData = 0;
        int16_t n16DeltaZData = 0;
        int16_t n16DeltaXYZData = 0;
        
        /*      对传入函数的参数合法性进行验证 */
        if (!mag_data|| nMagDataLen < 6 || !bg_mag_data || nBgMagDataLen < 6 || !n16VariantData)
		return -1;
        
        /*   地磁传感器X轴、Y轴、Z轴低字节原始数据与高字节原始数据变量     */
        btXDataMSB = gRawMagnetDataBuf[0];
        btXDataLSB = gRawMagnetDataBuf[1];
        btYDataMSB = gRawMagnetDataBuf[2];
        btYDataLSB = gRawMagnetDataBuf[3];
        btZDataMSB = gRawMagnetDataBuf[4];
        btZDataLSB = gRawMagnetDataBuf[5];
        
        /*      根据地磁传感器数据原始值通过将低低字节右移1位、将高字节左移7位得到X轴、Y轴和Z轴整体数据右移1位的地磁数据值        */
        n16XData = (int16_t) (btXDataMSB << 7) | (btXDataLSB >> 1);
        n16XData = (n16XData < -1024)?-1024:n16XData;
        n16XData = (n16XData > 1024)?1024:n16XData;
        n16YData = (int16_t) (((int16_t)btYDataMSB) << 7) | (btYDataLSB >> 1);
        n16YData = (n16YData < -1024)?-1024:n16YData;
        n16YData = (n16YData > 1024)?1024:n16YData;
        n16ZData = (int16_t) (((int16_t)btZDataMSB) << 7) | (btZDataLSB >> 1);
        n16ZData = (n16ZData < -1024)?-1024:n16ZData;
        n16ZData = (n16ZData > 1024)?1024:n16ZData;
        
        mag_data[0] = (uint8_t)((n16XData >> 8) & 0xFF);
        mag_data[1] = (uint8_t)(n16XData & 0xFF);
        mag_data[2] = (uint8_t)((n16YData >> 8) & 0xFF);
        mag_data[3] = (uint8_t)(n16YData & 0xFF);
        mag_data[4] = (uint8_t)((n16ZData >> 8) & 0xFF);
        mag_data[5] = (uint8_t)(n16ZData & 0xFF);
        
        /*      根据背景地磁数据计算参考地磁数据，将X轴、Y轴和Z轴的背景地磁数据值相加求和并且求平均值得到参考地磁数据值     */
        for(n16XDataSum = 0, n16YDataSum = 0, n16ZDataSum = 0, nIndex = 0; nIndex < MAGNET_DATA_BUF_SIZE; nIndex++)
        {
          n16XDataSum += g_x_magnet_data_buf[nIndex];
          n16YDataSum += g_y_magnet_data_buf[nIndex];
          n16ZDataSum += g_z_magnet_data_buf[nIndex];
        }
        /*n16XRefValue = (int16_t)(n16XDataSum / MAGNET_DATA_BUF_SIZE);
        n16YRefValue = (int16_t)(n16YDataSum / MAGNET_DATA_BUF_SIZE);
        n16ZRefValue = (int16_t)(n16ZDataSum / MAGNET_DATA_BUF_SIZE);*/
        n16XRefValue = (int16_t)(n16XDataSum >> MAGNET_DATA_BUF_SIZE_BITS);
        n16YRefValue = (int16_t)(n16YDataSum >> MAGNET_DATA_BUF_SIZE_BITS);
        n16ZRefValue = (int16_t)(n16ZDataSum >> MAGNET_DATA_BUF_SIZE_BITS);
        
        bg_mag_data[0] = (uint8_t)((n16XRefValue >> 8) & 0xFF);
        bg_mag_data[1] = (uint8_t)(n16XRefValue & 0xFF);
        bg_mag_data[2] = (uint8_t)((n16YRefValue >> 8) & 0xFF);
        bg_mag_data[3] = (uint8_t)(n16YRefValue & 0xFF);
        bg_mag_data[4] = (uint8_t)((n16ZRefValue >> 8) & 0xFF);
        bg_mag_data[5] = (uint8_t)(n16ZRefValue & 0xFF);
        
        /*      计算实际地磁数据值与背景地磁数据值的差值的绝对值        */
        n16DeltaXData = (n16XData > n16XRefValue)?(n16XData - n16XRefValue):(n16XRefValue - n16XData);
        n16DeltaYData = (n16YData > n16YRefValue)?(n16YData - n16YRefValue):(n16YRefValue - n16YData);
        n16DeltaZData = (n16ZData > n16ZRefValue)?(n16ZData - n16ZRefValue):(n16ZRefValue - n16ZData);
        
        /*      计算X轴、Y轴和Z轴地磁数据值与背景地磁数据值差值绝对值的和  */
        n16DeltaXYZData = n16DeltaXData + n16DeltaYData + n16DeltaZData;
        
        * n16VariantData = n16DeltaXYZData;
        
        /*      X轴、Y轴与Z轴地磁数据变化量的绝对值的和小于变化门限，表示背景磁场发生变化，需要更新背景地磁数据值      */
        if(n16DeltaXYZData <= g_low_delta_threshold)
        {
          g_x_magnet_data_buf[g_n_cur_data_buf_index] = n16XData;
          g_y_magnet_data_buf[g_n_cur_data_buf_index] = n16YData;
          g_z_magnet_data_buf[g_n_cur_data_buf_index] = n16ZData;
          g_n_cur_data_buf_index = (g_n_cur_data_buf_index + 1) % MAGNET_DATA_BUF_SIZE;
          g_n_dec_fluctuation_count++;
          /*    实际地磁数据值连续若干次与背景地磁数据值的差值小于一定门限，表示目前没有车辆  */
          if(g_n_dec_fluctuation_count > g_n_dec_fluctuation_limit)
          {
            g_n_dec_fluctuation_count = 0;
          }
        }
        else if(n16DeltaXYZData >= g_high_delta_threshold)      /*      X轴、Y轴与Z轴地磁数据变化量的绝对值的和大于变化门限值，表示有车或者抖动导致地磁数据变化超过门限值      */
        {
          g_n_inc_fluctuation_count++;
          /*    实际地磁数据值连续若干次与背景地磁数据值的差值大于一定门限，表示目前有车辆  */
          if(g_n_inc_fluctuation_count > g_n_inc_fluctuation_limit)
          {
            g_n_inc_fluctuation_count = 0;
          }
        }
        
	return 0;
}
#endif

#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void) 
{
        if(IS_MAGNET_INT_FLAG_SET())
        {
          CLEAR_MAGNET_INT_FLAG();
          g_background_magnetic_sample_count += 1;
          Multiple_Read_HMC5883();//连续读出数据，存储在Rec_Data[]中
          executeAtUpdateMagValue();

          LPM3_EXIT;
        }
}

#endif
