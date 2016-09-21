/*
 * Main driver for the radio.
 *
 * @author CaoHua
 *
 */

#ifndef __MRFI_H__
#define __MRFI_H__

#include "Led.h"
#include "mrfi_CC1101_Concept.h"

#define FREQUENCY_HOP_STEP      6

/**
 * initiate the mrfi driver.
 * @param txpower the power of TX
 * @param channel
 * @param addrHighByte MSByte of address of this node, will be used as HW filtering.
 * @param addrLowByte LSByte of address of this node
 */
void init_MRFI(uint8_t txpower, uint8_t channel, uint8_t addrHighByte, uint8_t addrLowByte);

void reset_MRFI(uint8_t txpower, uint8_t channel, uint8_t addrHighByte, uint8_t addrLowByte);

/**
 * go into WAR state
 */
void turnOnWAR_MRFI(void);

void turnOnRX_MRFI();

/**
 * let the radio go to sleep state.
 */
void turnOffRadio_MRFI();

/**
 * Sending a packet, with concurrency avoiding.
 *
 * @param *packet pointer of the packet to send
 * @return SUCCESS/FAILURE
 * SUCCESS: the packet starts sending.
 * FAILURE: the sending is canceled, for others are sending now.
 */
uint8_t transmitPacket_MRFI(mrfiPacket_t* packet);

uint8_t broadcastPacket_MRFI(mrfiPacket_t* packet);

void changeChannelNum_MRFI(uint8_t chann_num);
void changeTxPower_MRFI(uint8_t tx_power);
void changeNodeAddr_MRFI(uint8_t addrHighByte, uint8_t addrLowByte);

int8_t rssi_value_2_power(uint8_t nRSSIValue);

__monitor uint8_t getRFState_MRFI();

/**
 * 留给上层写收包后处理的函数
 * @param packet the pointer to the 1st byte of the packets radio received.
 */
void executeAtReceivePacketISR_MRFI(const mrfiPacket_t *packet);

/**
 * 留给上层写发包后处理的函数
 */
void executeAtTransmitPacketISR_MRFI();

/**
 * 留给上层写收到同步字后处理的函数
 */
void executeAtReceiveSyncISR_MRFI();

/**
 * 留给上层写收发包结束后的处理函数，无论包的内容是否正确
 */
void executeAtRFFinishedISR_MRFI();


#endif
