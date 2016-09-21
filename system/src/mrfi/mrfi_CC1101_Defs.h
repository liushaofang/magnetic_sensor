#ifndef __MRFI_CC1101_DEFS_H__
#define __MRFI_CC1101_DEFS_H__

#include "mrfi_spi.h"

static const uint8_t mrfiRadioCfg[][2] = {
#ifndef MAG_TEST
	{IOCFG0, 0x06},
	{IOCFG2, 0x06},
	{MCSM2, BV(4) | BV(3)},
	{MCSM1, BV(4)},
	{MCSM0, BV(4) | BV(3) | BV(2)},
	{WORCTRL, BV(5) | BV(4) | BV(3)},
	{WOREVT1, 0x02},
	{WOREVT0, 0x4D},
	{PKTLEN, PACKET_LEN},
	{PKTCTRL0, BV(6) | BV(2) | BV(0)},
        {PKTCTRL1, BV(6) | BV(3) | BV(2) | BV(1) | BV(0)},
	{FIFOTHR, BV(2) | BV(1) | BV(0)},
	{FSCTRL1, 0x05},
	{FREQ2, 0x10},
	{FREQ1, 0xA7},
	{FREQ0, 0x62},
        //2014.07.20:liusf modified to change rf communication speed from 10KHZ to 250KHZ
	//10KBaud
        //{MDMCFG4, 0xC8},        //10KHZ
	//{MDMCFG3, 0x93},        //10KHZ
        //15KBaud
        {MDMCFG4, 0xC9},        //15KHZ
	{MDMCFG3, 0x2E},        //15KHZ
        //250KBaud
        //{MDMCFG4, 0xCD},
        //{MDMCFG3, 0x3B},
        //125KBaud
        //{MDMCFG4, 0xCC},
        //{MDMCFG3, 0x3B},
        //62.5KBaud
        //{MDMCFG4, 0xCB},        //2014.07.20:liusf debug find out this communication speed can complete RF communication
        //{MDMCFG3, 0x3B},
        //014.07.20:liusf modified end
	{MDMCFG2,  BV(4) | BV(0)},
	{MDMCFG1,  BV(6) | BV(5) | BV(4) | BV(1)},
	{DEVIATN, 0x34},
        
        //2014.01.13:   liusf modified for mrfi optimization，调试发现通过修改后降低了DVGA的最高门限，能够显著提高通过射频接收对CCA的判断，从而提高发射效率
        {AGCCTRL2, 0xC3},
	{AGCCTRL1, 0x00},
	{AGCCTRL0, 0xB0},
        //2014.01.13:   liusf modified end
        
	{FOCCFG, 0x16},
	{FSCAL3, 0xE9},
	{FSCAL2, 0x2A},
	{FSCAL1, 0x00},
	{FSCAL0, 0x1F},
	{TEST0, 0x09},
#else
	{IOCFG0, 0x06},
	{IOCFG2, 0x06},
	{MCSM2, 0x07},
	{MCSM1, BV(4) | BV(3) | BV(2)},
	{MCSM0, BV(4) | BV(3) | BV(2)},
//	{WORCTRL, 0xFB},
//	{WOREVT1, 0x87},
//	{WOREVT0, 0x6B},
	{PKTLEN, PACKET_LEN},
	{PKTCTRL0, BV(6) | BV(2) | BV(0)},
	{PKTCTRL1, BV(7) | BV(6) | BV(3) | BV(2) | BV(1) | BV(0)},
	{FIFOTHR, BV(2) | BV(1) | BV(0)},
	{FSCTRL1, 0x0C},
	{FSCTRL0, 0x00},
	{FREQ2, 0x10},
	{FREQ1, 0xB1},
	{FREQ0, 0x3B},
	{MDMCFG4, 0x2D},
	{MDMCFG3, 0x3B},
	{MDMCFG2,  0x13},
	{MDMCFG1,  0x72},
	{MDMCFG0,  0xF8},
	{DEVIATN, 0x62},
	{FOCCFG, 0x1D},
	{BSCFG, 0x1C},
	{AGCCTRL2, 0xC7},
	{AGCCTRL1, 0x00},
	{AGCCTRL0, 0xB0},
	{FSCAL3, 0xEA},
	{FSCAL2, 0x2A},
	{FSCAL1, 0x00},
	{FSCAL0, 0x1F},
	{TEST0, 0x09},
#endif
};

#endif
