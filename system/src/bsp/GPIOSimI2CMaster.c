/*
 * BITBANG-MODE I2C MASTER (simulated by GPIO PIN)
 * ʱ������Ϊ10us
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
 * I2C START�ź�
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
 * I2C STOP�ź�
 */
static void stop_GSI2CM(void) {
	SDA_LOW_QUARTER_OF_PERIOD;
	SCL_HIGH_QUARTER_OF_PERIOD;
	SDA_HIGH_QUARTER_OF_PERIOD;
	DIR_IN;
	DIR_IN_SCL;
}

/**
 * ��I2C����д1byte����
 * @Param data Ҫд������
 * @Return
 * FASLE д����ʧ�� No ack returned.
 * TRUE �յ�ACK��д�����ɹ�
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
 * ��I2C���߶�1byte����
 * @Return ����������
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
 * ����ACK
 * @Return
 * FALSE û���յ�ACK
 * TRUE �յ�ACK
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
 * @param addrOfTargetSlave ���豸��ַ
 * @param buffer ��Ŷ�����ָ��
 * @param bufferLength �����ĸ���
 * @return SUCCESS/FAILURE
 */
uint8_t read_GSI2CM(uint8_t addrOfTargetSlave, uint8_t* buffer,
		uint16_t bufferLength) {
	/* ��дλ��1��Ϊ������ */
	addrOfTargetSlave |= 0x01;

	/* ����START�ź� */
	start_GSI2CM();

	/* ���ʹ��豸��ַ�����û��Ӧ��˵�����豸������ */
	if (writeByte_GSI2CM(addrOfTargetSlave) != TRUE) {
		stop_GSI2CM();
		return FAILURE;
	}

	/* �������һ��byte����������͵���ACK�ź� */
	for (register uint8_t i = 0; i < bufferLength - 1; i++) {
		buffer[i] = readByte_GSI2CM();
		ack_GSI2CM();
	}

	buffer[bufferLength - 1] = readByte_GSI2CM();
	nack_GSI2CM();

	/* ����STOP�ź� */
	stop_GSI2CM();
	return SUCCESS;
}

/**
 * Write data to specified slave on the I2C bus.
 *
 * TODO add comment about result
 *
 * @param addrOfTargetSlave ���豸��ַ
 * @param buffer ���д���ݵ�ָ��
 * @param bufferLength д���ĸ���
 * @return��SUCCESS/FAILURE
 */
uint8_t write_GSI2CM(uint8_t addrOfTargetSlave, uint8_t* buffer,
		uint16_t bufferLength) {
	/* ��дλ��0��Ϊд���� */
	addrOfTargetSlave &= ~0x01;

	/* ����START�ź� */
	start_GSI2CM();

	/* ���ʹ��豸��ַ�����NoACK����ʾ���豸������ */
	if (writeByte_GSI2CM(addrOfTargetSlave) != TRUE) {
		stop_GSI2CM();
		return FAILURE;
	}

	/* д��ǰnBufLen-1�ֽ� ��if error occurs while write any byte, return false immediately. */
	for (register uint8_t i = 0; i < bufferLength - 1; i++) {
		if (writeByte_GSI2CM(buffer[i]) != TRUE) {
			stop_GSI2CM();
			return FAILURE;
		}
	}

	/* д�����һ���ֽڣ�����SLAVE����NACK��ACK����Ϊ���ͳɹ� */
	writeByte_GSI2CM(buffer[bufferLength - 1]);

	/* ����STOP�ź� */
	stop_GSI2CM();
	return SUCCESS;
}
