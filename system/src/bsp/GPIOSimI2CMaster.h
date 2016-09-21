/*
 * BITBANG-MODE I2C MASTER (simulated by GPIO PIN)
 *
 * TODO add the usage in sungari of GPIOI2CMaster
 *
 * we use the post-fix _GSI2CM to tag this driver.
 *
 * @author Taurus Ning
 *
 */
#ifndef __GPIO_SIMI2C_MASTER_H__
#define __GPIO_SIMI2C_MASTER_H__

#include "BspDef.h"

/**
 * Read data from specified slave on the I2C bus.
 *
 * @param addrOfTargetSlave ���豸��ַ
 * @param buffer ��Ŷ�����ָ��
 * @param bufferLength �����ĸ���
 * @return SUCCESS/FAILURE
 */
uint8_t read_GSI2CM(uint8_t addrOfTargetSlave, uint8_t* buffer,
		uint16_t bufferLength);


/**
 * Write data to specified slave on the I2C bus.
 *
 * @param addrOfTargetSlave ���豸��ַ
 * @param buffer ���д���ݵ�ָ��
 * @param bufferLength д���ĸ���
 * @return��SUCCESS/FAILURE
 */
uint8_t write_GSI2CM(uint8_t addrOfTargetSlave, uint8_t* buffer,
		uint16_t bufferLength);

void init_GSI2CM(void);

#endif
