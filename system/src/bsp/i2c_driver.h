#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_

#include "BspDef.h"

/**
 * Read data from specified slave on the I2C bus.
 *
 * @param addrOfTargetSlave ���豸��ַ
 * @param buffer ��Ŷ�����ָ��
 * @param bufferLength �����ĸ���
 * @return SUCCESS/FAILURE
 */
uint8_t read_I2CM(uint8_t addrOfTargetSlave, uint8_t* buffer,
		uint16_t bufferLength);


/**
 * Write data to specified slave on the I2C bus.
 *
 * @param addrOfTargetSlave ���豸��ַ
 * @param buffer ���д���ݵ�ָ��
 * @param bufferLength д���ĸ���
 * @return��SUCCESS/FAILURE
 */
uint8_t write_I2CM(uint8_t addrOfTargetSlave, uint8_t* buffer,
		uint16_t bufferLength);

void init_I2CM(void);

#endif /* I2C_DRIVER_H_ */
