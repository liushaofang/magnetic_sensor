/*
 * MagTest is a tool for test mag3110 driver, MagTest_Tx reads data from sensor and then transmits all raw data to MagTest_RX.
 * This is the main program for MagTest TX.
 *
 * 上电后开始读取地磁数据，每次读到数据都通过射频发送出去
 *
 * @author CaoHua
 *
 */

#include "PMSUtil.h"
#include "hmc5883l.h"

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

//static uint8_t flow_value;
static uint8_t geomagnetic_raw_data[RAW_DATA_LEN];

extern uint8_t g_n_inc_fluctuation_count;
extern uint8_t g_n_dec_fluctuation_count;

extern uint8_t g_nDeltaXYZData;
extern uint16_t g_u16DeltaXYZData;
extern uint8_t g_is_car_parked;

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

	//getValue(&flow_value, geomagnetic_raw_data, RAW_DATA_LEN);
        getValue(&g_is_car_parked, geomagnetic_raw_data, RAW_DATA_LEN);
	packet.highAddr = ST_RX_NODE_HIGH_ADDR;
	packet.lowAddr = ST_NODE_LOW_ADDR;

	packet.frameLength = PACKET_LEN -3;
        //2014.07.24:liusf modified for magnet algorithm debug
        //packet.frame[0] = ST_RX_NODE_HIGH_ADDR;
	//packet.frame[1] = ST_NODE_LOW_ADDR;
        //packet.frame[2] = 0xA1;
        packet.frame[0] = (uint8_t)(g_u16DeltaXYZData & 0xFF);  //可以上传参数0
        packet.frame[1] = (uint8_t)(g_u16DeltaXYZData >> 8 & 0xFF);     //可以上传参数1
        packet.frame[2] = OP_UPLOAD_RAW_MAGNETIC_DATA;
        packet.frame[3] = g_n_inc_fluctuation_count;    //可以上传参数2
        packet.frame[4] = g_n_dec_fluctuation_count;    //可以上传参数3
        //packet.frame[4] = (uint8_t)((g_u16BatteryVoltage >> 4) & 0xFF);
        /*    增加原始地磁数据值   */
        packet.frame[5] = geomagnetic_raw_data[0];
        packet.frame[6] = geomagnetic_raw_data[1];
        packet.frame[7] = geomagnetic_raw_data[4];
        packet.frame[8] = geomagnetic_raw_data[5];
        packet.frame[9] = geomagnetic_raw_data[3];
        packet.frame[10] = geomagnetic_raw_data[2];
        //2014.07.24:liusf modified end

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
                        //twinkleLed_LED(LED2, 1, 5);
		}
	}
}
