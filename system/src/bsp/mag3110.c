#include "mag3110.h"
#include "uscib0_i2c.h"
#include "uart.h"

#define DR_STATUS_ADDR  0x00  /*  只读  */
#define OUT_X_MSB_ADDR	0X01  /*  只读  */
#define OUT_X_LSB_ADDR	0X02  /*  只读  */
#define OUT_Y_MSB_ADDR	0X03  /*  只读  */
#define OUT_Y_LSB_ADDR	0X04  /*  只读  */
#define OUT_Z_MSB_ADDR	0X05  /*  只读  */
#define OUT_Z_LSB_ADDR	0X06  /*  只读  */
#define WHO_AM_I_ADDR	0X07  /*  只读  */
#define SYSMOD_ADDR	0X08  /*  只读  */
#define OFF_X_MSB_ADDR	0X09  /*  读写  */
#define OFF_X_LSB_ADDR	0X0A  /*  读写  */
#define OFF_Y_MSB_ADDR	0X0B  /*  读写  */
#define OFF_Y_LSB_ADDR	0X0C  /*  读写  */
#define OFF_Z_MSB_ADDR	0X0D  /*  读写  */
#define OFF_Z_LSB_ADDR	0X0E  /*  读写  */
#define DIE_TEMP_ADDR	0X0F  /*  读写  */
#define CTRL_REG1_ADDR	0X10  /*  读写  */
#define CTRL_REG2_ADDR	0X11  /*  读写  */

#define MAG3110_DEVICE_ID 0xC4
#define MAG3110_DEVICE_WRITE_ADDRESS  0x1C
#define MAG3110_DEVICE_READ_ADDRESS   0x1D

#define ZYX_DATA_READY_MASK           0x08

static uint8_t g_magnetic_sample_count = 0;
static uint32_t g_traffic_flow_value = 0;

static int16_t g_z_data_calcu_buf[20] = { 0 };
static int16_t g_low_data_value = 0;
static uint8_t g_high_z_value_count = 0;
static uint8_t g_low_z_value_count = 0;
static uint8_t g_vihicle_exist = 0;

void write_one_byte_mga3110(uint8_t btAddr, uint8_t btVal);
uint8_t read_one_byte_mga3110(uint8_t btAddr);

//static uint8_t pre_geomagnetic_raw_data_x = 0;
//static uint8_t pre_geomagnetic_raw_data_y = 0;
//static uint8_t pre_geomagnetic_raw_data_z = 0;

//static uint8_t cur_geomagnetic_raw_data_x = 0;
//static uint8_t cur_geomagnetic_raw_data_y = 0;
//static uint8_t cur_geomagnetic_raw_data_z = 0;

void mag3110_delay_milli_seconds(int32_t nMS) {
	int32_t i, j;
	for (i = 0; i < nMS; i++) {
		for (j = 0; j < 800; j++)
			;
	}
}

void init_uscib0_mag3110_int(void)
{
  //P1OUT |= BIT4;  //将P1.4设置为输出低电平
  //P1OUT &= ~BIT4;  //将P1.4设置为输出低电平
  
  P1SEL &= ~BIT4;  //将P1.4设置为I/O功能模式
  //P1DIR |= BIT4;   //将P1.4配置为输出模式
  //P1REN |= BIT4;  //禁止内部电阻
  //P1OUT &= ~BIT4;  //将P1.4设置为输出低电平
  //P1OUT |= BIT4;  //将P1.4设置为输出低电平
  //P1OUT &= ~BIT4;  //将P1.4设置为输出低电平
  
  P1DIR &= ~BIT4; //将P1.4配置为输入模式
  P1REN &= ~BIT4;  //禁止内部电阻
  //P1OUT |= BIT4;  //将P1.4设置为下拉
  P1IES &= ~BIT4; //将P1.4设置为从高电平到低电平的沿触发模式
  P1IFG &= ~BIT4; //清除P1.4中断标志位
  P1IE |= BIT4;   //使能P1.4端口中断
}

int init_mag_sensor() {
        uint8_t btRetVal = 0;
	uint8_t btCtrlReg1 = 0;
	uint8_t btDeviceID = 0;
	//初始化MAG3110中断控制
	//init_uscib0_mag3110_int();
	//使MAG3110进入Standby模式
	write_one_byte_mga3110(CTRL_REG2_ADDR, 0x80);
//	btCtrlReg1 = read_one_byte_mga3110(CTRL_REG1_ADDR);
//	btCtrlReg1 &= 0xFC;
//	write_one_byte_mga3110(CTRL_REG1_ADDR, btCtrlReg1);
//	write_one_byte_mga3110(CTRL_REG2_ADDR, 0x80);
	//write_one_byte_mga3110(CTRL_REG1_ADDR, 0x19);
	write_one_byte_mga3110(CTRL_REG1_ADDR, 0x61); //80HZ * 16 = 1280HZ
        //write_one_byte_mga3110(CTRL_REG1_ADDR, 0x05); //80HZ * 16 = 1280HZ fast read mode
        //write_one_byte_mga3110(CTRL_REG1_ADDR, 0x61);   //10HZ * 16 = 160HZ
        
	
        
	//初始化MAG3110中断控制
	init_uscib0_mag3110_int();    //2013.09.22:liusf comment it off for debug
        //延迟100ms
//	mag3110_delay_milli_seconds(100);
        //获取MAG3110芯片ID标识
        btDeviceID = read_one_byte_mga3110(WHO_AM_I_ADDR);
        if (btDeviceID == MAG3110_DEVICE_ID)
            btRetVal = 0;
        else
            btRetVal = -1;
        //延迟100ms
//	mag3110_delay_milli_seconds(100);
        //清除地磁中断
        read_one_byte_mga3110(OUT_X_MSB_ADDR);
        //读取地磁数据初始值
        //pre_geomagnetic_raw_data_x = read_one_byte_mga3110(OUT_X_LSB_ADDR); //读取X轴数据低字节
        //pre_geomagnetic_raw_data_y = read_one_byte_mga3110(OUT_Y_LSB_ADDR); //读取Y轴数据低字节
        //pre_geomagnetic_raw_data_z = read_one_byte_mga3110(OUT_Z_LSB_ADDR); //读取Z轴数据低字节
        //pre_geomagnetic_raw_data_x = read_one_byte_mga3110(OUT_X_MSB_ADDR); //读取X轴数据低字节
        //pre_geomagnetic_raw_data_y = read_one_byte_mga3110(OUT_Y_MSB_ADDR); //读取Y轴数据低字节
        //pre_geomagnetic_raw_data_z = read_one_byte_mga3110(OUT_Z_MSB_ADDR); //读取Z轴数据低字节
        //read_one_byte_mga3110(OUT_X_MSB_ADDR);
        //返回初始化成功信息
        return btRetVal;
}

void write_one_byte_mga3110(uint8_t btAddr, uint8_t btVal) {
	uscib0_i2c_start();
	uscib0_i2c_write_byte(MAG3110_DEVICE_WRITE_ADDRESS);
	uscib0_i2c_get_ack();
	uscib0_i2c_write_byte(btAddr);
	uscib0_i2c_get_ack();
	uscib0_i2c_write_byte(btVal);
	uscib0_i2c_get_ack();
	uscib0_i2c_stop();
}

uint8_t read_one_byte_mga3110(uint8_t btAddr) {
	uint8_t btData;
	uscib0_i2c_start();
	uscib0_i2c_write_byte(MAG3110_DEVICE_WRITE_ADDRESS);
	uscib0_i2c_get_ack();
	uscib0_i2c_write_byte(btAddr);
	uscib0_i2c_get_ack();
	uscib0_i2c_restart();
	uscib0_i2c_write_byte(MAG3110_DEVICE_READ_ADDRESS);
	btData = uscib0_i2c_read_byte();
	//uscib0_i2c_nack();
	uscib0_i2c_stop();
	return btData;
}

uint16_t getValue(uint32_t *flow_value, uint8_t *raw_data, uint8_t nRawDataLen) {
	int8_t nIndex = 0;
	uint8_t btZDataLSB = 0;
	uint8_t btZDataMSB = 0;
	int16_t n16ZData = 0;
	int16_t lZDataCalcuSum = 0;
	int16_t n16DeltaZData = 0;

#ifdef MAG_TEST
    raw_data[0] = read_one_byte_mga3110(OUT_X_MSB_ADDR); //读取X轴数据低字节，高字节目前不需要读取
    raw_data[1] = read_one_byte_mga3110(OUT_X_LSB_ADDR); //读取X轴数据低字节，高字节目前不需要读取
    raw_data[2] = read_one_byte_mga3110(OUT_Y_MSB_ADDR); //读取Y轴数据低字节，高字节目前不需要读取
    raw_data[3] = read_one_byte_mga3110(OUT_Y_LSB_ADDR); //读取Y轴数据低字节，高字节目前不需要读取
    raw_data[4] = read_one_byte_mga3110(OUT_Z_MSB_ADDR); //读取Z轴数据高字节，高字节目前不需要读取
    raw_data[5] = read_one_byte_mga3110(OUT_Z_LSB_ADDR); //读取Z轴数据低字节，高字节目前不需要读取
    return 0;
#endif

	if (flow_value == 0 || raw_data == 0 || nRawDataLen < 3)
		return -1;
        
        //cur_geomagnetic_raw_data_x = read_one_byte_mga3110(OUT_X_LSB_ADDR); //读取X轴数据低字节
        //cur_geomagnetic_raw_data_y = read_one_byte_mga3110(OUT_Y_LSB_ADDR); //读取Y轴数据低字节
        //cur_geomagnetic_raw_data_z = read_one_byte_mga3110(OUT_Z_LSB_ADDR); //读取Z轴数据低字节
        
        read_one_byte_mga3110(OUT_X_MSB_ADDR); //读取Z轴数据低字节，高字节目前不需要读取
        
	raw_data[0] = read_one_byte_mga3110(OUT_X_LSB_ADDR); //读取X轴数据低字节，高字节目前不需要读取
        //延迟100ms
	//mag3110_delay_milli_seconds(100);
        
	raw_data[1] = read_one_byte_mga3110(OUT_Y_LSB_ADDR); //读取Y轴数据低字节，高字节目前不需要读取
        //延迟100ms
	//mag3110_delay_milli_seconds(100);
        
        btZDataMSB = read_one_byte_mga3110(OUT_Z_MSB_ADDR); //读取Z轴数据高字节，高字节目前不需要读取
        
	raw_data[2] = read_one_byte_mga3110(OUT_Z_LSB_ADDR); //读取Z轴数据低字节，高字节目前不需要读取
        
        read_one_byte_mga3110(OUT_X_MSB_ADDR);
        
        //延迟100ms
	//mag3110_delay_milli_seconds(5);
        
#if 1
        //writeToUart_BSP(raw_data, 3);
        
	btZDataLSB = raw_data[2];
	//btZDataMSB = read_one_byte_mga3110(OUT_Z_MSB_ADDR); //读取Z轴数据高字节，高字节目前不需要读取
#if 1
	n16ZData = (int16_t) (btZDataMSB << 3) + (int16_t) (btZDataLSB >> 5);
        //n16ZData = n16ZData >> 5;
	if (g_magnetic_sample_count < 100)     //需要对Z值数组进行初始化
			{
		g_magnetic_sample_count++;
		if (g_magnetic_sample_count < 53) {
			for (nIndex = 0; nIndex < 18; nIndex++) {
				g_z_data_calcu_buf[nIndex] = g_z_data_calcu_buf[nIndex + 1];
			}
			g_z_data_calcu_buf[18] = n16ZData;			  //没有发生异常，把进来的数值，传给低值数列
		}
	}
#endif

#if 1
	lZDataCalcuSum = 0;
	for (nIndex = 0; nIndex < 8; nIndex++) {
		lZDataCalcuSum += g_z_data_calcu_buf[nIndex];
	}
	g_low_data_value = (int16_t) (lZDataCalcuSum / 8);
#endif
        

#if 1
	if (n16ZData < g_low_data_value) {
		n16DeltaZData = g_low_data_value - n16ZData;
	} else {
		n16DeltaZData = n16ZData - g_low_data_value;
	}
#endif
        
//#else
        
	if (n16DeltaZData < 30) {
		for (nIndex = 0; nIndex < 18; nIndex++) {
			g_z_data_calcu_buf[nIndex] = g_z_data_calcu_buf[nIndex + 1];
		}
		g_z_data_calcu_buf[18] = n16ZData;
	}
        


	if (g_magnetic_sample_count > 55) {
		if (n16DeltaZData > 70) {
			g_high_z_value_count++;
			if (g_high_z_value_count > 4) {
				g_high_z_value_count = 4 + 2;
				g_vihicle_exist = 1;
				g_low_z_value_count = 0;
				for (nIndex = 8; nIndex < 19; nIndex++)   //尾部数据刷新
						{
					g_z_data_calcu_buf[nIndex] = g_low_data_value;
				}
			}
		} else {
			g_high_z_value_count = 0;
		}
	}

	if (g_vihicle_exist == 1) {
		if (n16DeltaZData < 30) {
			g_low_z_value_count++;
			if (g_low_z_value_count > 8) {
				g_low_z_value_count = 8 + 3;
				g_vihicle_exist = 0;
				g_traffic_flow_value += 1;
				g_high_z_value_count = 0;
			}
		} else {
			g_low_z_value_count = 0;
		}
	}
	*flow_value = g_traffic_flow_value;
#endif
	return 0;
}

/**
 * Port1 Interrupt service routine
 *
 * 00h No interrupt pending -
 * 02h Port1.0 interrupt
 * 04h Port1.1 interrupt
 * 06h Port1.2 interrupt
 * 08h Port1.3 interrupt
 * 0Ah Port1.4 interrupt, 目前MAG3110使用该中断
 * 0Ch Port1.5 interrupt
 * 0Eh Port1.6 interrupt
 * 10h Port1.7 interrupt
 */
/*#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void) {
	switch (__even_in_range(P1IV, 16)) {
	case 0: //无中断
		break;
	case 2: //P1.0中断
		break;
	case 4: //P1.1中断
		break;
	case 6: //P1.2中断
		break;
	case 8: //P1.3中断
		break;
	case 10: //P1.4中断
		executeAtUpdateMag3110Value();
		LPM3_EXIT;
		break;
	case 12: //P1.5中断
		break;
	case 14: //P1.6中断
		break;
	case 16: //P1.7中断
		break;
	}
}
*/
