/*
 * MagTest is a tool for test mag3110 driver, MagTest_Tx reads data from sensor and then transmits all raw data to MagTest_RX.
 * This is the main program for MagTest TX.
 *
 * 上电后开始读取地磁数据，每次读到数据都通过射频发送出去
 *
 * @author CaoHua
 *
 */

#include "MagTest_Node.h"
#include <string.h>
   
#define RAW_DATA_LEN 6

#define FLAG_GEOMAGNETIC_DATA_PREPARED 1

static uint16_t sgriFlags = 0;

#define SGRI_IS_SET(x)  (sgriFlags & (0x01 << x))
#define SGRI_CLEAR(x)  (sgriFlags &= ~(0x01 << x))
#define SGRI_SET(x)    (sgriFlags |= (0x01 << x))

/* end */
/////////////////////////////////////////////////////////////


//static uint16_t receivedPacketCounter = 0;
//static mrfiPacket_t msg;

static uint8_t flow_value;
static uint8_t geomagnetic_raw_data[RAW_DATA_LEN];

void executeAtTransmitPacketISR_MRFI() {
	//no action required for SimpleTest.
}

void executeAtReceiveSyncISR_MRFI() {
	//no action required for SimpleTest.
}

void executeAtReceivePacketISR_MRFI(const mrfiPacket_t *packet) {

}

void executeAtUpdateMagValue() {
	SGRI_SET(FLAG_GEOMAGNETIC_DATA_PREPARED);
}

static void transmit() {
	mrfiPacket_t packet; //assign mem or not?
	memset(geomagnetic_raw_data, 0x0, RAW_DATA_LEN);
	memset(&packet, 0x0, sizeof(mrfiPacket_t));

	getValue(&flow_value, geomagnetic_raw_data, RAW_DATA_LEN);
	packet.highAddr = ST_RX_NODE_HIGH_ADDR;
	packet.lowAddr = ST_NODE_LOW_ADDR;

	packet.frameLength = PACKET_LEN -3;
	packet.frame[0] = ST_TX_NODE_HIGH_ADDR;
	packet.frame[1] = ST_NODE_LOW_ADDR;
	packet.frame[2] = 0xA1;
	for(int i = 0; i < RAW_DATA_LEN; i++){
		packet.frame[3 + i] = geomagnetic_raw_data[i];
	}

	transmitPacket_MRFI(&packet);
//	while(transmitPacket_MRFI(&packet) == FAILURE) {
//		startTimerAsDelaying_TimerB(10);
//		LPM3;
//	}

	/* Wait the transmit finish ISR to wake up */
	LPM3;
}

void executeAtRFFinishedISR_MRFI()
{
}

void main() {
	initBSP_BSP();
	DISABLE_INTERRUPTS_SGR();

	initLeds_LED();
	init_MRFI(TXPOWER, CHANNEL, ST_TX_NODE_HIGH_ADDR, ST_NODE_LOW_ADDR);
	init_mag_sensor();
   	showVersion_LED(3);
	ENABLE_INTERRUPTS_SGR();

	while (1) {
		LPM3;

		if (SGRI_IS_SET(FLAG_GEOMAGNETIC_DATA_PREPARED)) {
			SGRI_CLEAR(FLAG_GEOMAGNETIC_DATA_PREPARED);
			transmit();
		}
	}
}
