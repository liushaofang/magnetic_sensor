/*
 * Main driver for the radio.
 *
 * @author CaoHua
 *
 */

#include <string.h>

#include "mrfi.h"
#include "mrfi_CC1101_Defs.h"
#include "mag3110.h"

/**
 * strobe IDLE command at first,
 * then poll CHIP STATE till it is IDLE.
 * TODO It has risk! if the radio is in WOR, it may return far before the IDLE command take effects.
 */
#define STROBE_AND_ENSURE_IDLE()   st(            			\
		strobeCmd_MRFISPI(SIDLE);                       	\
		while (strobeCmd_MRFISPI(SNOP) != CHIP_STATE_IDLE); \
)

static mrfiPacket_t mrfiIncomingPacket;
static uint8_t addrLowByteOfNode = 0;

/**
 * 标记是发过程还是收过程，用于在中断函数中区分
 * 发包开始时置1，确认发包结束时置0.
 */
static uint8_t inTxProcess = 0;

/**
 * 初始化MRFI层
 * This function must be executed before all the RF operations, and GIE must be turned off.
 */
void init_MRFI(uint8_t txpower, uint8_t channel, uint8_t addrHighByte, uint8_t addrLowByte) {
	/* HW configuration */
	INIT_MRFISPI();
	INIT_GDO0_PIN_MRFI();
	INIT_GDO2_PIN_MRFI();

        /* for safely update the config-registers. */
	STROBE_AND_ENSURE_IDLE();
        
	strobeCmd_MRFISPI(SRES);
        
	/* attributes initiation */
	addrLowByteOfNode = addrLowByte;
	memset(&mrfiIncomingPacket, 0x00, sizeof(mrfiIncomingPacket));

	/* Radio Configuration */
	//TODO After CC1101 power on, which state is it in?
	writeReg_MRFISPI(mrfiRadioCfg, sizeof(mrfiRadioCfg) / sizeof(mrfiRadioCfg[0]));

	uint8_t regs[][2] = {{PA_TABLE0, txpower},{CHANNR, (channel + FREQUENCY_HOP_STEP)},{ADDR, addrHighByte}};
	writeReg_MRFISPI(regs,sizeof(regs) / sizeof(regs[0]));

	CLEAR_GDO2_FLAG_MRFI();
	CLEAR_GDO0_FLAG_MRFI();
}

void reset_MRFI(uint8_t txpower, uint8_t channel, uint8_t addrHighByte, uint8_t addrLowByte){
	DISABLE_INTERRUPTS_SGR();
	/* for safely update the config-registers. */
	STROBE_AND_ENSURE_IDLE();

	strobeCmd_MRFISPI(SRES);

	memset(&mrfiIncomingPacket, 0x00, sizeof(mrfiIncomingPacket));

	writeReg_MRFISPI(mrfiRadioCfg, sizeof(mrfiRadioCfg) / sizeof(mrfiRadioCfg[0]));

	uint8_t regs[][2] = {{PA_TABLE0, txpower},{CHANNR, (channel + FREQUENCY_HOP_STEP)},{ADDR, addrHighByte}};
	writeReg_MRFISPI(regs,sizeof(regs) / sizeof(regs[0]));

	CLEAR_GDO2_FLAG_MRFI();
	CLEAR_GDO0_FLAG_MRFI();
	ENABLE_INTERRUPTS_SGR();
}

void changePacketLen_MRFI(uint8_t length) {
	uint8_t regs[][2] = {{PKTLEN, length}};
	writeReg_MRFISPI(regs, 1);
}

void changeChannelNum_MRFI(uint8_t chann_num) {
	/* for safely update the config-registers. */
	STROBE_AND_ENSURE_IDLE();

	uint8_t regs[][2] = {{CHANNR, (chann_num + FREQUENCY_HOP_STEP)}};
	writeReg_MRFISPI(regs, 1);
}

void changeTxPower_MRFI(uint8_t tx_power) {
	/* for safely update the config-registers. */
	STROBE_AND_ENSURE_IDLE();

	uint8_t regs[][2] = {{PA_TABLE0, tx_power}};
	writeReg_MRFISPI(regs, 1);
}

void changeNodeAddr_MRFI(uint8_t addrHighByte, uint8_t addrLowByte) {
	/* for safely update the config-registers. */
	STROBE_AND_ENSURE_IDLE();

	uint8_t regs[][2] = {{ADDR, addrHighByte}};
	writeReg_MRFISPI(regs, 1);
	addrLowByteOfNode = addrLowByte;
}

/**
**@brief:将RSSI值转换为射频信号强度信息
**/
int8_t rssi_value_2_power(uint8_t nRSSIValue)
{
  int8_t nRSSIPower = 0;
  if(nRSSIValue >= 128)
    nRSSIPower = (nRSSIValue - 256)/2 - RSSI_OFFSET;
  else
    nRSSIPower = nRSSIValue/2 - RSSI_OFFSET;
  return nRSSIPower;
}

__monitor uint8_t  getRFState_MRFI(){
	uint8_t rf_state = CHIP_STATE_IDLE;
	rf_state = strobeCmd_MRFISPI(SNOP);
	return rf_state;
}

/**
 * Sending a packet, with concurrency avoiding.
 *
 * @param *packet pointer of the packet to send
 * @return SUCCESS/FAILURE
 * SUCCESS: the packet starts sending.
 * FAILURE: the sending is canceled, for others are sending now.
 */
uint8_t transmitPacket_MRFI(mrfiPacket_t* packet) {
	inTxProcess = 1;
        
	/* for safely update the config-registers. */
	STROBE_AND_ENSURE_IDLE();
        
        FEED_WDT;       //2015.02.01:liusf add watchdog function

	/* turn off rssi-off for CCA */
	uint8_t reg[][2] = {{MCSM2, BV(3)}};
	writeReg_MRFISPI(reg, 1);

	/* length write into packet should be payload+address.*/
	uint8_t packetLength = packet->frameLength + 2;
        
        FEED_WDT;       //2015.02.01:liusf add watchdog function

	writeTxFifo_MRFISPI(&(packetLength), 1);
	writeTxFifo_MRFISPI(&(packet->highAddr), 1); //WILL BE HW FILTER
	writeTxFifo_MRFISPI(&(packet->lowAddr), 1);
        
        FEED_WDT;       //2015.02.01:liusf add watchdog function
        
	writeTxFifo_MRFISPI(&(packet->frame[0]), packet->frameLength);
        
        FEED_WDT;       //2015.02.01:liusf add watchdog function
        
	/* go to RX at first for CCA required. */
	strobeCmd_MRFISPI(SRX);
        
        FEED_WDT;       //2015.02.01:liusf add watchdog function
        
	while(strobeCmd_MRFISPI(SNOP) != CHIP_STATE_RX)
		;
        
        FEED_WDT;       //2015.02.01:liusf add watchdog function

	/**
	 * For CCA request: RX needs the time to get a valid RSSI.
	 * If call STX before that, it will surely return FAILURE.
	 */
        DISABLE_WDT;    //2015.02.01:liusf add watchdog function
	delayInMs_BSP(1);
        FEED_WDT;       //2015.02.01:liusf add watchdog function
        
	strobeCmd_MRFISPI(STX);
        
        FEED_WDT;       //2015.02.01:liusf add watchdog function

	/* 等待从RX切到TX，datasheet中给的典型值是30us */
	DELAY_US_BSP(50);
                
        if(strobeCmd_MRFISPI(SNOP) != CHIP_STATE_TX) {
		/* CCA check fail, radio still remains at RX */
		/* give up this transmitting */
		STROBE_AND_ENSURE_IDLE();
		strobeCmd_MRFISPI(SFTX);
		return FAILURE;
	}
        
        FEED_WDT;       //2015.02.01:liusf add watchdog function
        
	return SUCCESS;
}

/**
 * 切WOR状态
 * 如果1101当前状态不是IDLE，先置其为IDLE再切换。
 * 注意：mrfi层尽量不要调用此方法，为了效率。可以直接strobe cmd时就不要用
 *
 * TODO This function CANNOT ensure the radio is WOR when it quit.
 */
void turnOnWAR_MRFI(void) {
	STROBE_AND_ENSURE_IDLE();

	/* turn on rssi-off switch */
	uint8_t regOriginal[][2] = {{MCSM2, BV(4) | BV(3)}};
	writeReg_MRFISPI(regOriginal, 1);
	strobeCmd_MRFISPI(SWOR);
}

void turnOnRX_MRFI(){
	STROBE_AND_ENSURE_IDLE();
	strobeCmd_MRFISPI(SRX);
}

void turnOffRadio_MRFI() {
	STROBE_AND_ENSURE_IDLE();
	strobeCmd_MRFISPI(SPWD);
}

#pragma vector=PORT1_VECTOR
__interrupt void doInISR_GPIO_P1(void) {
	if(IS_GDO2_FLAG_SET_MRFI()) {
		CLEAR_GDO2_FLAG_MRFI();
		executeAtReceiveSyncISR_MRFI();
		if (inTxProcess) {
                        DISABLE_WDT;       //2015.04.19:liusf add watchdog function
			return;
		}

//		LPM3_EXIT;
                DISABLE_WDT;       //2015.04.19:liusf add watchdog function
		return;
	}

	if(IS_GDO0_FLAG_SET_MRFI()) {
		CLEAR_GDO0_FLAG_MRFI();
		executeAtRFFinishedISR_MRFI();
		if(inTxProcess) {
			/* 发包结束*/
			inTxProcess = 0;
			executeAtTransmitPacketISR_MRFI();
                        DISABLE_WDT;    //2015.04.19:liusf add watchdog function
			LPM3_EXIT;
			return;
		}

		/* 收包结束的处理 */
                
                FEED_WDT;       //2015.02.01:liusf add watchdog function

		/* special code due to cc1101 ERR */
		uint8_t rxRegByte;
		{
			uint8_t rxBytesVerify;
			rxBytesVerify = readReg_MRFISPI(RXBYTES);
			do {
                                FEED_WDT;       //2015.02.01:liusf add watchdog function
				rxRegByte = rxBytesVerify;
				rxBytesVerify = readReg_MRFISPI(RXBYTES);
			} while (rxRegByte != rxBytesVerify);
		}

		if (rxRegByte == 0) {
			/* TODO is it only when HW ADDR filter occurred? */
			strobeCmd_MRFISPI(SWOR);
                        DISABLE_WDT;    //2015.04.19:liusf add watchdog function
			return;
		}

		//TODO not strict
		memset(&mrfiIncomingPacket, 0x00, PACKET_LEN);

		uint8_t packetLength = 0;
		readRxFifo_MRFISPI(&(packetLength), 1);

		ASSERT_SGR(packetLength > 2, 7);
		mrfiIncomingPacket.frameLength = packetLength - 2; //Exclude 2 bytes of ADDR.

#ifndef NODE_SIMPLETEST
		if(mrfiIncomingPacket.frameLength != PACKET_LEN - 3){
			strobeCmd_MRFISPI(SFRX);
			STROBE_AND_ENSURE_IDLE();
#ifndef MAG_TEST
			strobeCmd_MRFISPI(SWOR);
#else
			strobeCmd_MRFISPI(SRX);
#endif
                        DISABLE_WDT;    //2015.04.19:liusf add watchdog function
			return;
		}
#endif
                
                FEED_WDT;       //2015.02.01:liusf add watchdog function
                
		readRxFifo_MRFISPI(&(mrfiIncomingPacket.highAddr), 1);
		readRxFifo_MRFISPI(&(mrfiIncomingPacket.lowAddr), 1);
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
		readRxFifo_MRFISPI(&(mrfiIncomingPacket.frame[0]), mrfiIncomingPacket.frameLength);
                FEED_WDT;       //2015.02.01:liusf add watchdog function
		readRxFifo_MRFISPI(&(mrfiIncomingPacket.rxMetrics[0]), RX_METRICS_LEN);
                
                FEED_WDT;       //2015.02.01:liusf add watchdog function
                
		/* SW ADDR filtering, check the LSByte of ADDR */
		if (mrfiIncomingPacket.lowAddr == addrLowByteOfNode) {
			/* Receive packet successfully.  call the upper layer operation for the packet */
			executeAtReceivePacketISR_MRFI(&mrfiIncomingPacket);
                        DISABLE_WDT;       //2015.04.19:liusf add watchdog function
			LPM3_EXIT;
		}
                
                FEED_WDT;       //2015.02.01:liusf add watchdog function

		/* return to WAR state, no matter if ADDR SW filtered, or received packet successfully */
#ifndef MAG_TEST
		strobeCmd_MRFISPI(SWOR);
                DISABLE_WDT;       //2015.04.19:liusf add watchdog function
#else
		strobeCmd_MRFISPI(SRX);
#endif
                
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
                
		return;
	}
}

/**
 *  These asserts happen if there is extraneous compiler padding of arrays.
 *  Modify compiler settings for no padding, or, if that is not possible,
 *  comment out the offending asserts.
 */
STATIC_ASSERT_SGR(sizeof(mrfiRadioCfg) == ((sizeof(mrfiRadioCfg)/sizeof(mrfiRadioCfg[0])) * sizeof(mrfiRadioCfg[0])));

