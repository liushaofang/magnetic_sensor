#include "uscib0_i2c.h"
#include "hmc5883l.h"

#define RAW_MAGNET_DATA_LEN         6
#define MAGNET_DATA_BUF_SIZE        32
#define MAGNET_DATA_BUF_SIZE_BITS   5   /*      ��ʾ����ش�ƽ��ֵʱͨ����λ���������������λ������Ҫ����һ����ͬʱ�޸�   */

//2015.04.12:liusf add to optimize traffic detection algorithm
#define REFERENCE_MAGNET_DATA_BUF_SIZE             8
#define REFERENCE_MAGNET_DATA_BUF_SIZE_BITS        3       /*      ȡ32���ش����ݻ������е�ǰ8����Ϊ�ش����ݻ�������׼ֵ  */
static uint8_t g_nReferenceMagnetDataStartIndex = 0;
//2015.04.12:liusf add end

#ifdef HW_I2C_DEV

static void init_uscib0_hmc5883l_int(void);
static void Single_Write_HMC5883(int8_t Address,int8_t Dat);
static void Multiple_Read_HMC5883(void);

/*      ��¼�شŴ���������ԭʼ����ֵ  */
volatile int8_t gRawMagnetDataBuf[RAW_MAGNET_DATA_LEN] = { 0 };
/*      �����ų���������ͳ��ֵ   */
static volatile uint8_t g_background_magnetic_sample_count = 0;
/*      ��ʶ�Ƿ�ֹͣ�شŴ�������־λ  */
static volatile uint8_t g_suspend_magnet_flag = 0;
/*      ��ʶ�شŴ���������״̬ 1��ʾ���ڹ�����0��ʾֹͣ       */
static volatile uint8_t g_is_magnet_sampling = 1;
/*      ��¼�شŴ�����X��ش����ݱ����ų�ֵ������      */
static int16_t g_x_magnet_data_buf[MAGNET_DATA_BUF_SIZE] = { 0 };
/*      ��¼�شŴ�����Y��ش����ݱ����ų�ֵ������      */
static int16_t g_y_magnet_data_buf[MAGNET_DATA_BUF_SIZE] = { 0 };
/*      ��¼�شŴ�����Z��ش����ݱ����ų�ֵ������      */
static int16_t g_z_magnet_data_buf[MAGNET_DATA_BUF_SIZE] = { 0 };
/*      ��¼�شŴ������ش����ݱ仯�������̵Ķ�������ֵ      */
int16_t g_low_delta_threshold = -20;   //15;
/*      ��¼�شŴ������ش����ݱ仯���������̵Ķ�������ֵ      */
int16_t g_high_delta_threshold = 20;   //210;
/*      ��¼��ǰ��Ҫ���µı����ų���������       */
static uint8_t g_n_cur_data_buf_index = 0;
/*      ͳ��Ŀǰ�ش��������Ӷ����仯����  */
uint8_t g_n_inc_fluctuation_count = 0;
/*      ��¼Ŀǰ�ش��������Ӷ����仯������ʾ�г��Ľ���  */
static uint8_t g_n_inc_fluctuation_limit = 2;
/*      ͳ��Ŀǰ�ش����ݼ��ٶ����仯����  */
uint8_t g_n_dec_fluctuation_count = 0;
/*      ��¼Ŀǰ�ش��������Ӷ����仯������ʾ�޳��Ľ���  */
static uint8_t g_n_dec_fluctuation_limit = 2;
/*      ͳ��Ŀǰ�ش������ڶ�������ֵ�ڵı仯����  */
uint8_t g_n_no_fluctuation_count = 0;
/*      ��¼Ŀǰ�ش������ڶ�������ֵ�ڵĽ���  */
static uint8_t g_n_no_fluctuation_limit = 2;

static int16_t g_n_prev_x_magnet_value = 0;

uint8_t g_nDeltaXYZData = 0;  //2014.07.06:liusf add for magnet test
uint16_t g_u16DeltaXYZData = 0;  //2014.07.06:liusf add for magnet test
/*      ��¼�Ƿ��г���־        */
uint8_t g_is_car_parked = 0;

//��ʼ��HMC5883��������Ҫ��ο�pdf�����޸�****
int init_mag_sensor(void)
{
    __delay_cycles(150000);
    //��ʼ��HMC5883L�ж�
    init_uscib0_hmc5883l_int();
    //��ʼ��MSP430Ӳ��I2C
    init_uscib0_i2c();
    //��ʼ��HMC5883Lģ��
    //Single_Write_HMC5883(0x00,0x60); 
    Single_Write_HMC5883(0x00,0x18);       //75HZ��Ŀǰ��������
    //Single_Write_HMC5883(0x00,0x14);      //30HZ��Ŀǰ��������
    //Single_Write_HMC5883(0x00,0x10);      //15HZ��Ŀǰ��������
    //Single_Write_HMC5883(0x00,0x0C);      //7.5HZ
    //Single_Write_HMC5883(0x00,0x08);      //3HZ��Ŀǰ��������
    //Single_Write_HMC5883(0x00,0x04);      //1.5HZ��Ŀǰ��������
    //Single_Write_HMC5883(0x00,0x0);      //0.75HZ��Ŀǰ��������
    
//    Single_Write_HMC5883(MODE_REGISTER,MODE_IDLE);        //��HMC5883L�شŴ���������Ϊ����ģʽ
    Single_Write_HMC5883(0x02,0x00);
    //HMC5883_Send_Byte(0x00);
    __delay_cycles(150000);
    
    //g_magnetic_sample_count = 0;
    //reset_background_magnetic();
    
    return SUCCESS;
}

void init_uscib0_hmc5883l_int(void)
{
  P2SEL &= ~P2BIT_MAGNET_BIT; //��P2.3����ΪI/O����ģʽ
  P2DIR &= ~P2BIT_MAGNET_BIT; //��P2.3����Ϊ����ģʽ
  P2REN &= ~P2BIT_MAGNET_BIT; //��ֹ�ڲ�����
  P2IES |= P2BIT_MAGNET_BIT; //��P2.3����Ϊ�Ӹߵ�ƽ���͵�ƽ���ش���ģʽ
  P2IFG &= ~P2BIT_MAGNET_BIT; //���P2.3�жϱ�־λ
  P2IE |= P2BIT_MAGNET_BIT;   //ʹ��P2.3�˿��ж�
}

void disable_hmc5883l_int(void)
{
  P2IE &= ~P2BIT_MAGNET_BIT;   //��ֹP2.3�˿ڵش��ж�
}

void enable_hmc5883l_int(void)
{
  P2IE |= P2BIT_MAGNET_BIT;   //ʹ��P2.3�˿ڵش��ж�
}

/*���ֽ�дHMC5883*/
static void Single_Write_HMC5883(int8_t Address,int8_t Dat)
{
    I2C_Write1byte(Address,Dat); 
}

/*���ֽڶ�HMC5883*/
void Multiple_Read_HMC5883(void)
{
    I2C_ReadNbyte((uint8_t *)gRawMagnetDataBuf,RAW_MAGNET_DATA_LEN,OUT_X_MSB_ADDR);
}

/* ��HMC5883L�شŴ���������ΪIDLEģʽ        */
__monitor void suspend_hmc5883l(void)
{
	CLEAR_MAGNET_INT_FLAG();
	Single_Write_HMC5883(MODE_REGISTER,MODE_IDLE);        //��HMC5883L�شŴ���������Ϊ����ģʽ
        //__delay_cycles(150000);       //�ȴ����ɸ�ʱ������
	g_is_magnet_sampling = 0;
}

/* �ָ�HMC5883L�شŴ�������������ģʽ        */
__monitor uint8_t resume_hmc5883l(void)
{
	CLEAR_MAGNET_INT_FLAG();
	Single_Write_HMC5883(MODE_REGISTER, MODE_CONTINOUS_MEASUREMENT); //��HMC5883L�شŴ���������Ϊ��������ģʽ
	//__delay_cycles(150000);      //�ȴ����ɸ�ʱ������
	g_is_magnet_sampling = 1;
	return SUCCESS;
}

__monitor uint8_t set_hmc5883l_odr(uint8_t nOutputDataRate)
{
	CLEAR_MAGNET_INT_FLAG();
	Single_Write_HMC5883(0x00,nOutputDataRate); //��HMC5883L�شŴ�����������������ΪnOutputDataRate
	//__delay_cycles(150000);      //�ȴ����ɸ�ʱ������
	return SUCCESS;
}

/* �ú����ڵ���֮ǰӦ�ý�ȫ�ֱ���g_background_magnetic_sample_count��Ϊ��ʼֵ0������ʹ��ȫ���жϺ͵ش��ж�    */
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
        
        /*      �شŴ�����X����ֽ�ԭʼ��������ֽ�ԭʼ���ݱ���  */
        int8_t btXDataMSB = 0;
        uint8_t btXDataLSB = 0;
        
        /*      �شŴ�����X����ֽ�����1λ�͸��ֽ�����7λ�õ��ĵش�����ֵ    */
        int16_t n16XData = 0;
        
        /*      �شŴ���������X�����ݼ����    */
        int16_t n16XDataSum = 0;
        
        /*      ���ݵشŴ������ش����ݼ���õ��ı����ų�ֵ   */
        int16_t n16XRefValue = 0;
        
        /*      ��¼�شŴ�����X��ĵش����ݱ仯�� */
        int16_t n16DeltaXData = 0;
        
        /*      �Դ��뺯���Ĳ����Ϸ��Խ�����֤ */
        if (pCurFlucState == 0 || raw_data == 0 || nRawDataLen < 6)
		return -1;
        
        /*      ���ɵشŴ������жϴ������л�ȡ�ĵش�����ԭʼֵͨ����������    */
        raw_data[0] = gRawMagnetDataBuf[0]; //��ȡX�����ݸ��ֽ�
        raw_data[1] = gRawMagnetDataBuf[1]; //��ȡX�����ݵ��ֽ�
        raw_data[2] = gRawMagnetDataBuf[2]; //��ȡY�����ݸ��ֽ�
        raw_data[3] = gRawMagnetDataBuf[3]; //��ȡY�����ݵ��ֽ�
        raw_data[4] = gRawMagnetDataBuf[4]; //��ȡZ�����ݸ��ֽ�
        raw_data[5] = gRawMagnetDataBuf[5]; //��ȡZ�����ݵ��ֽ�
        
        /*   �شŴ�����X�ᡢY�ᡢZ����ֽ�ԭʼ��������ֽ�ԭʼ���ݱ���     */
        btXDataMSB = gRawMagnetDataBuf[0];
        btXDataLSB = gRawMagnetDataBuf[1];
        
        /*      ���ݵشŴ���������ԭʼֵͨ�����͵��ֽ�����1λ�������ֽ�����7λ�õ�X�ᡢY���Z��������������1λ�ĵش�����ֵ        */
        n16XData = (int16_t) (btXDataMSB << 7) + (int16_t) (btXDataLSB >> 1);
        g_n_prev_x_magnet_value = n16XData;
        /*      ���ӶԵ�ǰ�ش�ֵΪ-4096���������Ϊ4096������Ĵ���ͬʱ�Ե�ǰ�ش�ֵΪ4096���������Ϊ-4096������Ĵ���        */
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
        
        /*      ���ݱ����ش����ݼ���ο��ش����ݣ���X�ᡢY���Z��ı����ش�����ֵ�����Ͳ�����ƽ��ֵ�õ��ο��ش�����ֵ     */
        for(n16XDataSum = 0, nIndex = 0; nIndex < REFERENCE_MAGNET_DATA_BUF_SIZE; nIndex++)
        {
          n16XDataSum += g_x_magnet_data_buf[(g_nReferenceMagnetDataStartIndex + nIndex) % MAGNET_DATA_BUF_SIZE];
        }
        n16XRefValue = (int16_t)(n16XDataSum >> REFERENCE_MAGNET_DATA_BUF_SIZE_BITS);
        
        /*      ����X��ʵ�ʵش�����ֵ�뱳���ش�����ֵ�Ĳ�ֵ       */
        n16DeltaXData = n16XData - n16XRefValue;
        /*      X�ᡢY����Z��ش����ݱ仯���ľ���ֵ�ĺ�С�ڱ仯���ޣ���ʾ�����ų������仯����Ҫ���±����ش�����ֵ      */
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
          /*    ʵ�ʵش�����ֵ�������ɴ��뱳���ش�����ֵ�Ĳ�ֵС��һ�����ޣ���ʾĿǰû�г���  */
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
          /*    ʵ�ʵش�����ֵ�������ɴ��뱳���ش�����ֵ�Ĳ�ֵС��һ�����ޣ���ʾĿǰû�г���  */
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
        
        /*      �شŴ�����X�ᡢY�ᡢZ����ֽ�ԭʼ��������ֽ�ԭʼ���ݱ���  */
        int8_t btXDataMSB = 0;
        uint8_t btXDataLSB = 0;
        int8_t btYDataMSB = 0;
        uint8_t btYDataLSB = 0;
        int8_t btZDataMSB = 0;
        uint8_t btZDataLSB = 0;
        
        /*      �شŴ�����X�ᡢY�ᡢZ����ֽ�����1λ�͸��ֽ�����7λ�õ��ĵش�����ֵ    */
        int16_t n16XData = 0;
        int16_t n16YData = 0;
        int16_t n16ZData = 0;
        
        /*      �شŴ���������X�ᡢY���Z�����ݼ����    */
        int16_t n16XDataSum = 0;
        int16_t n16YDataSum = 0;
        int16_t n16ZDataSum = 0;
        
        /*      ���ݵشŴ������ش����ݼ���õ��ı����ų�ֵ   */
        int16_t n16XRefValue = 0;
        int16_t n16YRefValue = 0;
        int16_t n16ZRefValue = 0;
        
        /*      ��¼�شŴ�����X�ᡢY���Z��ĵش����ݱ仯�����仯��֮�� */
        int16_t n16DeltaXData = 0;
        int16_t n16DeltaYData = 0;
        int16_t n16DeltaZData = 0;
        int16_t n16DeltaXYZData = 0;
        
        /*      �Դ��뺯���Ĳ����Ϸ��Խ�����֤ */
        if (pIsCarParked == 0 || raw_data == 0 || nRawDataLen < 6)
		return -1;
        
        /*      ���ɵشŴ������жϴ������л�ȡ�ĵش�����ԭʼֵͨ����������    */
        raw_data[0] = gRawMagnetDataBuf[0]; //��ȡX�����ݸ��ֽ�
        raw_data[1] = gRawMagnetDataBuf[1]; //��ȡX�����ݵ��ֽ�
        raw_data[2] = gRawMagnetDataBuf[2]; //��ȡY�����ݸ��ֽ�
        raw_data[3] = gRawMagnetDataBuf[3]; //��ȡY�����ݵ��ֽ�
        raw_data[4] = gRawMagnetDataBuf[4]; //��ȡZ�����ݸ��ֽ�
        raw_data[5] = gRawMagnetDataBuf[5]; //��ȡZ�����ݵ��ֽ�
        
        /*   �شŴ�����X�ᡢY�ᡢZ����ֽ�ԭʼ��������ֽ�ԭʼ���ݱ���     */
        btXDataMSB = gRawMagnetDataBuf[0];
        btXDataLSB = gRawMagnetDataBuf[1];
        btYDataMSB = gRawMagnetDataBuf[2];
        btYDataLSB = gRawMagnetDataBuf[3];
        btZDataMSB = gRawMagnetDataBuf[4];
        btZDataLSB = gRawMagnetDataBuf[5];
        
        /*      ���ݵشŴ���������ԭʼֵͨ�����͵��ֽ�����1λ�������ֽ�����7λ�õ�X�ᡢY���Z��������������1λ�ĵش�����ֵ        */
        n16XData = (int16_t) (btXDataMSB << 7) + (int16_t) (btXDataLSB >> 1);
        n16XData = (n16XData < -1024)?-1024:n16XData;
        n16XData = (n16XData > 1024)?1024:n16XData;
        n16YData = (int16_t) (btYDataMSB << 7) + (int16_t) (btYDataLSB >> 1);
        n16YData = (n16YData < -1024)?-1024:n16YData;
        n16YData = (n16YData > 1024)?1024:n16YData;
        n16ZData = (int16_t) (btZDataMSB << 7) + (int16_t) (btZDataLSB >> 1);
        n16ZData = (n16ZData < -1024)?-1024:n16ZData;
        n16ZData = (n16ZData > 1024)?1024:n16ZData;
        
        /*      ���ݱ����ش����ݼ���ο��ش����ݣ���X�ᡢY���Z��ı����ش�����ֵ�����Ͳ�����ƽ��ֵ�õ��ο��ش�����ֵ     */
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
        
        /*      ����ʵ�ʵش�����ֵ�뱳���ش�����ֵ�Ĳ�ֵ�ľ���ֵ        */
        n16DeltaXData = (n16XData > n16XRefValue)?(n16XData - n16XRefValue):(n16XRefValue - n16XData);
        n16DeltaYData = (n16YData > n16YRefValue)?(n16YData - n16YRefValue):(n16YRefValue - n16YData);
        n16DeltaZData = (n16ZData > n16ZRefValue)?(n16ZData - n16ZRefValue):(n16ZRefValue - n16ZData);
        
        /*      ����X�ᡢY���Z��ش�����ֵ�뱳���ش�����ֵ��ֵ����ֵ�ĺ�  */
        n16DeltaXYZData = n16DeltaXData + n16DeltaYData + n16DeltaZData;
        
        g_u16DeltaXYZData = n16DeltaXYZData;
        g_nDeltaXYZData = (uint8_t)(n16DeltaXYZData & 0xFF);    //2014.07.06:liusf add
        
        /*      X�ᡢY����Z��ش����ݱ仯���ľ���ֵ�ĺ�С�ڱ仯���ޣ���ʾ�����ų������仯����Ҫ���±����ش�����ֵ      */
        if(n16DeltaXYZData <= g_low_delta_threshold)
        {
          /*    �޳�����شű仯С����ֵ��ֱ�Ӹ��µش����ݣ��г�ʱ��Ҫ�ȵ��޳�ʱ�ٸ��µش�����   */
            //2014.07.06: liusf comment it off to delete manetic update
//          if(g_is_car_parked == 0)
//          {
//            g_x_magnet_data_buf[g_n_cur_data_buf_index] = n16XData;
//            g_y_magnet_data_buf[g_n_cur_data_buf_index] = n16YData;
//            g_z_magnet_data_buf[g_n_cur_data_buf_index] = n16ZData;
//            g_n_cur_data_buf_index = (g_n_cur_data_buf_index + 1) % MAGNET_DATA_BUF_SIZE;
//          }
          g_n_dec_fluctuation_count++;
          /*    ʵ�ʵش�����ֵ�������ɴ��뱳���ش�����ֵ�Ĳ�ֵС��һ�����ޣ���ʾĿǰû�г���  */
          if(g_n_dec_fluctuation_count > g_n_dec_fluctuation_limit)
          {
            g_n_inc_fluctuation_count = 0;
            g_n_dec_fluctuation_count = 0;
            g_is_car_parked = 0;
            * pIsCarParked = 0;
          }
        }
        else if(n16DeltaXYZData >= g_high_delta_threshold)      /*      X�ᡢY����Z��ش����ݱ仯���ľ���ֵ�ĺʹ��ڱ仯����ֵ����ʾ�г����߶������µش����ݱ仯��������ֵ      */
        {
          g_n_inc_fluctuation_count++;
          /*    ʵ�ʵش�����ֵ�������ɴ��뱳���ش�����ֵ�Ĳ�ֵ����һ�����ޣ���ʾĿǰ�г���  */
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
        
        /*      �شŴ�����X�ᡢY�ᡢZ����ֽ�ԭʼ��������ֽ�ԭʼ���ݱ���  */
        int8_t btXDataMSB = 0;
        uint8_t btXDataLSB = 0;
        int8_t btYDataMSB = 0;
        uint8_t btYDataLSB = 0;
        int8_t btZDataMSB = 0;
        uint8_t btZDataLSB = 0;
        
        /*      �شŴ�����X�ᡢY�ᡢZ����ֽ�����1λ�͸��ֽ�����7λ�õ��ĵش�����ֵ    */
        int16_t n16XData = 0;
        int16_t n16YData = 0;
        int16_t n16ZData = 0;
        
        /*      �شŴ���������X�ᡢY���Z�����ݼ����    */
        int16_t n16XDataSum = 0;
        int16_t n16YDataSum = 0;
        int16_t n16ZDataSum = 0;
        
        /*      ���ݵشŴ������ش����ݼ���õ��ı����ų�ֵ   */
        int16_t n16XRefValue = 0;
        int16_t n16YRefValue = 0;
        int16_t n16ZRefValue = 0;
        
        /*      ��¼�شŴ�����X�ᡢY���Z��ĵش����ݱ仯�����仯��֮�� */
        int16_t n16DeltaXData = 0;
        int16_t n16DeltaYData = 0;
        int16_t n16DeltaZData = 0;
        int16_t n16DeltaXYZData = 0;
        
        /*      �Դ��뺯���Ĳ����Ϸ��Խ�����֤ */
        if (!mag_data|| nMagDataLen < 6 || !bg_mag_data || nBgMagDataLen < 6 || !n16VariantData)
		return -1;
        
        /*   �شŴ�����X�ᡢY�ᡢZ����ֽ�ԭʼ��������ֽ�ԭʼ���ݱ���     */
        btXDataMSB = gRawMagnetDataBuf[0];
        btXDataLSB = gRawMagnetDataBuf[1];
        btYDataMSB = gRawMagnetDataBuf[2];
        btYDataLSB = gRawMagnetDataBuf[3];
        btZDataMSB = gRawMagnetDataBuf[4];
        btZDataLSB = gRawMagnetDataBuf[5];
        
        /*      ���ݵشŴ���������ԭʼֵͨ�����͵��ֽ�����1λ�������ֽ�����7λ�õ�X�ᡢY���Z��������������1λ�ĵش�����ֵ        */
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
        
        /*      ���ݱ����ش����ݼ���ο��ش����ݣ���X�ᡢY���Z��ı����ش�����ֵ�����Ͳ�����ƽ��ֵ�õ��ο��ش�����ֵ     */
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
        
        /*      ����ʵ�ʵش�����ֵ�뱳���ش�����ֵ�Ĳ�ֵ�ľ���ֵ        */
        n16DeltaXData = (n16XData > n16XRefValue)?(n16XData - n16XRefValue):(n16XRefValue - n16XData);
        n16DeltaYData = (n16YData > n16YRefValue)?(n16YData - n16YRefValue):(n16YRefValue - n16YData);
        n16DeltaZData = (n16ZData > n16ZRefValue)?(n16ZData - n16ZRefValue):(n16ZRefValue - n16ZData);
        
        /*      ����X�ᡢY���Z��ش�����ֵ�뱳���ش�����ֵ��ֵ����ֵ�ĺ�  */
        n16DeltaXYZData = n16DeltaXData + n16DeltaYData + n16DeltaZData;
        
        * n16VariantData = n16DeltaXYZData;
        
        /*      X�ᡢY����Z��ش����ݱ仯���ľ���ֵ�ĺ�С�ڱ仯���ޣ���ʾ�����ų������仯����Ҫ���±����ش�����ֵ      */
        if(n16DeltaXYZData <= g_low_delta_threshold)
        {
          g_x_magnet_data_buf[g_n_cur_data_buf_index] = n16XData;
          g_y_magnet_data_buf[g_n_cur_data_buf_index] = n16YData;
          g_z_magnet_data_buf[g_n_cur_data_buf_index] = n16ZData;
          g_n_cur_data_buf_index = (g_n_cur_data_buf_index + 1) % MAGNET_DATA_BUF_SIZE;
          g_n_dec_fluctuation_count++;
          /*    ʵ�ʵش�����ֵ�������ɴ��뱳���ش�����ֵ�Ĳ�ֵС��һ�����ޣ���ʾĿǰû�г���  */
          if(g_n_dec_fluctuation_count > g_n_dec_fluctuation_limit)
          {
            g_n_dec_fluctuation_count = 0;
          }
        }
        else if(n16DeltaXYZData >= g_high_delta_threshold)      /*      X�ᡢY����Z��ش����ݱ仯���ľ���ֵ�ĺʹ��ڱ仯����ֵ����ʾ�г����߶������µش����ݱ仯��������ֵ      */
        {
          g_n_inc_fluctuation_count++;
          /*    ʵ�ʵش�����ֵ�������ɴ��뱳���ش�����ֵ�Ĳ�ֵ����һ�����ޣ���ʾĿǰ�г���  */
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
          Multiple_Read_HMC5883();//�����������ݣ��洢��Rec_Data[]��
          executeAtUpdateMagValue();

          LPM3_EXIT;
        }
}

#endif
