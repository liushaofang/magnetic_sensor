/*
 * BITBANG-MODE I2C MASTER (simulated by GPIO PIN)
 * 时钟周期为10us
 *
 * @author Hua Cao, Taurus Ning
 *
 */
#include "GPIOSimI2CMaster.h"

#define QUARTER_OF_PERIOD		2
#define HALF_OF_PERIOD			5

#define SDA_1       P4OUT |=  BIT4
#define SDA_0       P4OUT &= ~BIT4
#define SCL_1        P4OUT |=  BIT5
#define SCL_0        P4OUT &= ~BIT5

#define SCL_HIGH_QUARTER_OF_PERIOD	st(SCL_1;DELAY_US_BSP(QUARTER_OF_PERIOD);)
#define SCL_LOW_QUARTER_OF_PERIOD	st(SCL_0;DELAY_US_BSP(QUARTER_OF_PERIOD);)
#define SCL_HIGH_HALF_OF_PERIOD			st(SCL_1;DELAY_US_BSP(HALF_OF_PERIOD);)
#define SCL_LOW_HALF_OF_PERIOD			st(SCL_0;DELAY_US_BSP(HALF_OF_PERIOD);)

#define SDA_HIGH_QUARTER_OF_PERIOD  st(SDA_1;DELAY_US_BSP(QUARTER_OF_PERIOD);)
#define SDA_LOW_QUARTER_OF_PERIOD  	st(SDA_0;DELAY_US_BSP(QUARTER_OF_PERIOD);)

/* control SDA mode */
#define DIR_IN      				P4DIR &= ~BIT4
#define DIR_OUT     			P4DIR |=  BIT4
#define DIR_OUT_SCL		P4DIR |=  BIT5
#define DIR_IN_SCL			P4DIR &= ~BIT5

/* read 1 bit from SDA */
#define SDA_IN      ((P4IN >> 4) & 0x01)

static void start_GSI2CM(void);
static void stop_GSI2CM(void);
static uint8_t writeByte_GSI2CM(uint8_t data);
static uint8_t readByte_GSI2CM(void);
static uint8_t receiveAck_GSI2CM(void);
static void ack_GSI2CM(void);

/**
 * Initiate for i2c usage at very beginning.
 */
void init_GSI2CM(void) {
	DIR_OUT_SCL;
	DIR_OUT;

	SDA_1;
	SCL_HIGH_QUARTER_OF_PERIOD;
}

/**
 * I2C START信号
 */
static void start_GSI2CM(void) {
	DIR_OUT_SCL;
	DIR_OUT;
	SDA_1;
	SCL_HIGH_QUARTER_OF_PERIOD;
	SDA_LOW_QUARTER_OF_PERIOD;
	SCL_LOW_QUARTER_OF_PERIOD;
	SDA_1;
}

/**
 * I2C STOP信号
 */
static void stop_GSI2CM(void) {
	SDA_LOW_QUARTER_OF_PERIOD;
	SCL_HIGH_QUARTER_OF_PERIOD;
	SDA_HIGH_QUARTER_OF_PERIOD;
	DIR_IN;
	DIR_IN_SCL;
}

/**
 * 向I2C总线写1byte数据
 * @Param data 要写的数据
 * @Return
 * FASLE 写操作失败 No ack returned.
 * TRUE 收到ACK，写操作成功
 */
static uint8_t writeByte_GSI2CM(uint8_t data) {
	for (register uint8_t i = 0; i < 8; i++) {
		if ((data >> 7) & 0x01) {
			SDA_HIGH_QUARTER_OF_PERIOD;
		} else {
			SDA_LOW_QUARTER_OF_PERIOD;
		}

		SCL_HIGH_HALF_OF_PERIOD;

		data = data << 1;

		if ( i == 7 )
			break;

		SCL_0;
	}

	/*when the slave finish receiving 1 byte, it will give ack back*/
	return receiveAck_GSI2CM();
}

/**
 * 从I2C总线读1byte数据
 * @Return 读到的数据
 */
static uint8_t readByte_GSI2CM(void) {
	uint8_t tempBit = 0;
	uint8_t tempData = 0;

	SCL_LOW_QUARTER_OF_PERIOD;
	DIR_IN;

	for (register uint8_t i = 0; i < 8; i++) {
		SCL_HIGH_QUARTER_OF_PERIOD;
		if (SDA_IN) {
			tempBit = 1;
		} else {
			tempBit = 0;
		}

		tempData = (tempData << 1) | tempBit;

		if ( i == 7)
			break;

		SCL_LOW_HALF_OF_PERIOD;
	}

	SCL_LOW_QUARTER_OF_PERIOD;
	DIR_OUT;
	return (tempData);
}

/**
 * receive ack from Slave??
 * 接收ACK
 * @Return
 * FALSE 没有收到ACK
 * TRUE 收到ACK
 */
static uint8_t receiveAck_GSI2CM(void) {
	SCL_LOW_QUARTER_OF_PERIOD;
	DIR_IN;
	SCL_HIGH_QUARTER_OF_PERIOD;

	if (SDA_IN) {
		SCL_LOW_QUARTER_OF_PERIOD;
		DIR_OUT;
		return FALSE;
	}

	SCL_LOW_QUARTER_OF_PERIOD;
	DIR_OUT;
	return TRUE;
}

/**
 * send ACK signal to the slave
 */
static void ack_GSI2CM(void) {
	DIR_OUT;
	SDA_LOW_QUARTER_OF_PERIOD;
	SCL_HIGH_HALF_OF_PERIOD;
	SCL_LOW_QUARTER_OF_PERIOD;
}

/**
 * send NACK signal to the slave
 */
static void nack_GSI2CM(void) {
	DIR_OUT;
	SDA_HIGH_QUARTER_OF_PERIOD;
	SCL_HIGH_HALF_OF_PERIOD;
	SCL_LOW_QUARTER_OF_PERIOD;
}

/**
 * Read data from specified slave on the I2C bus.
 *
 * @param addrOfTargetSlave 从设备地址
 * @param buffer 存放读数的指针
 * @param bufferLength 读数的个数
 * @return SUCCESS/FAILURE
 */
uint8_t read_GSI2CM(uint8_t addrOfTargetSlave, uint8_t* buffer,
		uint16_t bufferLength) {
	/* 读写位置1，为读操作 */
	addrOfTargetSlave |= 0x01;

	/* 发送START信号 */
	start_GSI2CM();

	/* 发送从设备地址，如果没回应则说明从设备不在线 */
	if (writeByte_GSI2CM(addrOfTargetSlave) != TRUE) {
		stop_GSI2CM();
		return FAILURE;
	}

	/* 读除最后一个byte外的数，发送的是ACK信号 */
	for (register uint8_t i = 0; i < bufferLength - 1; i++) {
		buffer[i] = readByte_GSI2CM();
		ack_GSI2CM();
	}

	buffer[bufferLength - 1] = readByte_GSI2CM();
	nack_GSI2CM();

	/* 发送STOP信号 */
	stop_GSI2CM();
	return SUCCESS;
}

/**
 * Write data to specified slave on the I2C bus.
 *
 * TODO add comment about result
 *
 * @param addrOfTargetSlave 从设备地址
 * @param buffer 存放写内容的指针
 * @param bufferLength 写数的个数
 * @return：SUCCESS/FAILURE
 */
uint8_t write_GSI2CM(uint8_t addrOfTargetSlave, uint8_t* buffer,
		uint16_t bufferLength) {
	/* 读写位置0，为写操作 */
	addrOfTargetSlave &= ~0x01;

	/* 发送START信号 */
	start_GSI2CM();

	/* 发送从设备地址，如果NoACK，表示从设备不存在 */
	if (writeByte_GSI2CM(addrOfTargetSlave) != TRUE) {
		stop_GSI2CM();
		return FAILURE;
	}

	/* 写入前nBufLen-1字节 ，if error occurs while write any byte, return false immediately. */
	for (register uint8_t i = 0; i < bufferLength - 1; i++) {
		if (writeByte_GSI2CM(buffer[i]) != TRUE) {
			stop_GSI2CM();
			return FAILURE;
		}
	}

	/* 写入最后一个字节，不管SLAVE返回NACK或ACK均认为发送成功 */
	writeByte_GSI2CM(buffer[bufferLength - 1]);

	/* 发送STOP信号 */
	stop_GSI2CM();
	return SUCCESS;
}
