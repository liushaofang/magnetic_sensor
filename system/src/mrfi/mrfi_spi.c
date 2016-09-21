#include "mrfi_spi.h"

#define DUMMY_BYTE                  0xDB

#define PARSE_REGADDR_READ_MRFISPI(x) (x | BV(7))
#define PARSE_REGADDR_WRITE_MRFISPI(x) (x & ~BV(7))
#define PARSE_REGADDR_SINGLE_MRFISPI(x) (x & ~BV(6))
#define PARSE_REGADDR_BURST_MRFISPI(x) (x | BV(6))

__monitor uint8_t strobeCmd_MRFISPI(uint8_t cmdMRFI) {
	uint8_t statusByte = 0;

	assert_LED((cmdMRFI >= 0x30) && (cmdMRFI <= 0x3D), ERR_ILLEGAL_CMD_MRFI);

	SELECT_ON_CC1101_TILL_XOSC_STABLE_MRFI();

	SPI_TX_BUF = cmdMRFI;
	while(!IS_RXFLAG_SET_MRFISPI())
			;
	statusByte = SPI_RX_BUF;

	CLEAR_RXFLAG_MRFISPI();
	CLEAR_TXFLAG_MRFISPI();
	SELECT_OFF_CC1101_MRFI();

	statusByte &= 0x70;
	statusByte >>= 4;
	return statusByte;
}

__monitor uint8_t readReg_MRFISPI(uint8_t regAddr) {
	assert_LED(regAddr <= 0x3B, ERR_ILLEGAL_CMD_MRFI);

	uint8_t regPreviousValue = 0;
        /*      为了防止出现Pe550警告   */
        regPreviousValue += 1;

	regAddr = PARSE_REGADDR_SINGLE_MRFISPI(regAddr);
	regAddr = PARSE_REGADDR_READ_MRFISPI(regAddr);

	SELECT_ON_CC1101_TILL_XOSC_STABLE_MRFI();
	CLEAR_RXFLAG_MRFISPI();
	CLEAR_TXFLAG_MRFISPI();

	SPI_TX_BUF = regAddr;

	TRANSFER_BYTE_MRFISPI(regPreviousValue, DUMMY_BYTE);

	while(!IS_RXFLAG_SET_MRFISPI())
		;
	regPreviousValue = SPI_RX_BUF;

	CLEAR_RXFLAG_MRFISPI();
	CLEAR_TXFLAG_MRFISPI();
	SELECT_OFF_CC1101_MRFI();

	return regPreviousValue;
}

__monitor void writeReg_MRFISPI(const uint8_t addrAndValue[][2], uint8_t size) {
	assert_LED(size > 0 , ERR_ILLEGAL_ARGUMENT);

	SELECT_ON_CC1101_TILL_XOSC_STABLE_MRFI();
	CLEAR_RXFLAG_MRFISPI();
	CLEAR_TXFLAG_MRFISPI();

	for (uint8_t i = 0; i < size; i++) {
		uint8_t regAddr = addrAndValue[i][0];
		uint8_t regValue = addrAndValue[i][1];

		assert_LED((regAddr <= 0x2E) || (regAddr == 0x3E), ERR_ILLEGAL_CMD_MRFI);

		uint8_t regPreviousValue = 0;
                /*      为了防止出现Pe550警告   */
                regPreviousValue += 1;

		regAddr = PARSE_REGADDR_SINGLE_MRFISPI(regAddr);
		regAddr = PARSE_REGADDR_WRITE_MRFISPI(regAddr);

		SPI_TX_BUF = regAddr;
		TRANSFER_BYTE_MRFISPI(regPreviousValue, regValue);
	}

	CLEAR_RXFLAG_MRFISPI();
	CLEAR_TXFLAG_MRFISPI();
	SELECT_OFF_CC1101_MRFI();
}

__monitor void writeTxFifo_MRFISPI(uint8_t * pData, uint8_t len) {
	assert_LED(len != 0, ERR_ILLEGAL_LEN_MRFI);

	uint8_t valueRead = 0;
        /*      为了防止出现Pe550的警告  */
        valueRead += 1;

	uint8_t regAddr = TXFIFO;
	regAddr = PARSE_REGADDR_BURST_MRFISPI(regAddr);
	regAddr = PARSE_REGADDR_WRITE_MRFISPI(regAddr);

	SELECT_ON_CC1101_TILL_XOSC_STABLE_MRFI();
	CLEAR_RXFLAG_MRFISPI();
	CLEAR_TXFLAG_MRFISPI();

	SPI_TX_BUF = regAddr;
	TRANSFER_BYTE_MRFISPI(valueRead, *pData);

	len--;

	while(len) {
		pData++;
		len--;
		TRANSFER_BYTE_MRFISPI(valueRead, *pData);
	}

	while(!IS_RXFLAG_SET_MRFISPI())
		;
	valueRead = SPI_RX_BUF;

	CLEAR_RXFLAG_MRFISPI();
	CLEAR_TXFLAG_MRFISPI();
	SELECT_OFF_CC1101_MRFI();
}

__monitor void readRxFifo_MRFISPI(uint8_t * pData, uint8_t len) {
	if(len == 0){
		assert_LED(len != 0, ERR_ILLEGAL_LEN_MRFI);
	}

	uint8_t regAddr = RXFIFO;
	regAddr = PARSE_REGADDR_BURST_MRFISPI(regAddr);
	regAddr = PARSE_REGADDR_READ_MRFISPI(regAddr);

	SELECT_ON_CC1101_TILL_XOSC_STABLE_MRFI();
	CLEAR_RXFLAG_MRFISPI();
	CLEAR_TXFLAG_MRFISPI();

	SPI_TX_BUF = regAddr;

	TRANSFER_BYTE_MRFISPI(*pData, DUMMY_BYTE);

	len--;

	while(len) {
		TRANSFER_BYTE_MRFISPI(*pData, DUMMY_BYTE);
		pData++;
		len--;
	}

	while(!IS_RXFLAG_SET_MRFISPI())
			;
	*pData = SPI_RX_BUF;

	CLEAR_RXFLAG_MRFISPI();
	CLEAR_TXFLAG_MRFISPI();
	SELECT_OFF_CC1101_MRFI();
}


