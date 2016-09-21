/*
 * SPI Driver for controlling the Radio chip
 *
 * 主要代码继承自SimpliciTI
 *
 * @author CaoHua
 *
 */

#ifndef __MRFI_SPI_H__
#define __MRFI_SPI_H__

#include "Led.h"
#include "mrfi_CC1101_Concept.h"

#ifdef HW_I2C_DEV

#define P3BIT_SPI_SI_MRFI 4
#define P3BIT_SPI_SO_MRFI 5
#define P3BIT_SPI_CLK_MRFI 0

#define SPI_CTL0 	UCA0CTL0
#define SPI_CTL1 	UCA0CTL1
#define SPI_BR0			UCA0BR0
#define SPI_BR1			UCA0BR1
#define SPI_RXIFG		UCA0RXIFG
#define SPI_TXIFG		UCA0TXIFG
#define SPI_TX_BUF		UCA0TXBUF
#define SPI_RX_BUF		UCA0RXBUF

#else

#define P3BIT_SPI_SI_MRFI 1
#define P3BIT_SPI_SO_MRFI 2
#define P3BIT_SPI_CLK_MRFI 3

#define SPI_CTL0 	UCB0CTL0
#define SPI_CTL1 	UCB0CTL1
#define SPI_BR0			UCB0BR0
#define SPI_BR1			UCB0BR1
#define SPI_RXIFG		UCB0RXIFG
#define SPI_TXIFG		UCB0TXIFG
#define SPI_TX_BUF		UCB0TXBUF
#define SPI_RX_BUF		UCB0RXBUF

#endif

#define P1BIT_GDO0_MRFI 2

#define P1BIT_GDO2_MRFI 3

#define P1BIT_MAGNET_INT 4

#define INIT_GDO0_PIN_MRFI() st(					\
		P1SEL &= ~BV(P1BIT_GDO0_MRFI);   	\
		P1DIR &= ~BV(P1BIT_GDO0_MRFI);		\
		P1IES |= BV(P1BIT_GDO0_MRFI);		\
		P1IFG &= ~BV(P1BIT_GDO0_MRFI); 		\
		P1IE |= BV(P1BIT_GDO0_MRFI);		\
)

#define INIT_GDO2_PIN_MRFI() st(					\
		P1SEL &= ~BV(P1BIT_GDO2_MRFI);   	\
		P1DIR &= ~BV(P1BIT_GDO2_MRFI);		\
		P1IES &= ~BV(P1BIT_GDO2_MRFI);		\
		P1IFG &= ~BV(P1BIT_GDO2_MRFI); 		\
		P1IE |= BV(P1BIT_GDO2_MRFI);		\
)

#define DISABLE_MRFI_INTR() st(					\
                P1IE &= ~BV(P1BIT_GDO0_MRFI);		\
		P1IE &= ~BV(P1BIT_GDO2_MRFI);		\
)

#define ENABLE_MRFI_INTR() st(					\
                P1IE |= BV(P1BIT_GDO0_MRFI);		\
		P1IE |= BV(P1BIT_GDO2_MRFI);		\
)

#define CLEAR_GDO0_FLAG_MRFI() st(P1IFG &= ~BV(P1BIT_GDO0_MRFI);)
#define IS_GDO0_FLAG_SET_MRFI()	(P1IFG & BV(P1BIT_GDO0_MRFI))

#define CLEAR_GDO2_FLAG_MRFI() st( P1IFG &= ~BV(P1BIT_GDO2_MRFI); )
#define IS_GDO2_FLAG_SET_MRFI() (P1IFG & BV(P1BIT_GDO2_MRFI) )

//#define CLEAR_MAGNET_INT_FLAG() st( P1IFG &= ~BV(P1BIT_MAGNET_INT); )
//#define IS_MAGNET_INT_FLAG_SET() (P1IFG & BV(P1BIT_MAGNET_INT) )

#define P2BIT_CC1101_CSN_MRFI 5

#define INIT_MRFISPI() st(						\
		P3SEL |= BV(P3BIT_SPI_CLK_MRFI) 		\
					| BV(P3BIT_SPI_SI_MRFI)		\
					| BV(P3BIT_SPI_SO_MRFI);	\
												\
		P2SEL &= ~BV(P2BIT_CC1101_CSN_MRFI);	\
		P2DIR |=  BV(P2BIT_CC1101_CSN_MRFI);	\
		P2OUT |=  BV(P2BIT_CC1101_CSN_MRFI);	\
		P2IE  &= ~BV(P2BIT_CC1101_CSN_MRFI);	\
												\
		SPI_CTL1 = UCSWRST;						\
		SPI_CTL1 = UCSWRST | UCSSEL_2;			\
		SPI_CTL0 = UCCKPH | UCMSB | UCMST | UCSYNC;	\
		SPI_BR0 = 2;							\
		SPI_BR1 = 0;							\
		SPI_CTL1 &= ~UCSWRST;					\
)

#define SELECT_ON_CC1101_MRFI()  st(P2OUT &= ~BV(P2BIT_CC1101_CSN_MRFI);)
#define SELECT_OFF_CC1101_MRFI() st(P2OUT |=  BV(P2BIT_CC1101_CSN_MRFI);)
#define IS_CC1101_NOT_SELECTED_MRFI() (P2OUT & BV(P2BIT_CC1101_CSN_MRFI))

#define IS_SPI_SO_HIGH_MRFI() (P3IN & BV(P3BIT_SPI_SO_MRFI))

#define SELECT_ON_CC1101_TILL_XOSC_STABLE_MRFI() st(SELECT_ON_CC1101_MRFI(); while(IS_SPI_SO_HIGH_MRFI());)

#define CLEAR_RXFLAG_MRFISPI() st(IFG2 &= ~SPI_RXIFG;)
#define CLEAR_TXFLAG_MRFISPI() st(IFG2 &= ~SPI_TXIFG;)
#define IS_RXFLAG_SET_MRFISPI() (IFG2 & SPI_RXIFG)
#define IS_TXFLAG_SET_MRFISPI() (IFG2 & SPI_TXIFG)

#define TRANSFER_BYTE_MRFISPI(byteRead, nextByteSent) st( \
		while(!IS_TXFLAG_SET_MRFISPI())	\
			;							\
		SPI_TX_BUF = (nextByteSent);		\
		while(!IS_RXFLAG_SET_MRFISPI())	\
			;							\
		(byteRead) = SPI_RX_BUF;			\
)

__monitor uint8_t strobeCmd_MRFISPI(uint8_t addr);
__monitor uint8_t readReg_MRFISPI(uint8_t addr);
__monitor void writeReg_MRFISPI(const uint8_t addrAndValue[][2], uint8_t size);
__monitor void writeTxFifo_MRFISPI(uint8_t * pWriteData, uint8_t len);
__monitor void readRxFifo_MRFISPI(uint8_t * pReadData, uint8_t len);

#endif
