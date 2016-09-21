#include "i2c_driver.h"

#ifdef HW_I2C_DEV

void init_I2CM(){
	P3SEL |= BV(1) | BV(2);                          // Assign I2C pins to USCI_B0
	UCB0CTL1 |= UCSWRST;                    // Enable SW reset
	UCB0CTL0 = UCMST | UCMODE_3 | UCSYNC;   // I2C Master, synchronous mode
	UCB0CTL1 = UCSSEL_2 | UCSWRST;          // Use SMCLK, keep SW reset
	UCB0BR0 = 18;                           // fSCL = SMCLK/18 = ~400kHz
	UCB0BR1 = 0;
	UCB0I2CSA = 0x1E;                       // Slave Address is 048h
	UCB0CTL1 &= ~UCSWRST;                   // Clear SW reset, resume operation

	IE2 &= ~UCB0RXIE;                       // disable Receive ready interrupt
	IE2 &= ~UCB0TXIE;                       // disable Transmit ready interrupt
}

/**
 * Write data to specified slave on the I2C bus.
 *
 * @param addrOfTargetSlave ���豸��ַ
 * @param buffer ���д���ݵ�ָ��
 * @param bufferLength д���ĸ���
 * @return��SUCCESS/FAILURE
 */
uint8_t write_I2CM(uint8_t addrOfTargetSlave, uint8_t* buffer, uint16_t bufferLength) {
//	UCB0I2CSA = addrOfTargetSlave;
	while (UCB0CTL1 & UCTXSTP);

	UCB0CTL1 |= UCTR;                   // UCTR=1 => Transmit Mode (R/W bit = 0)
	IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
	UCB0CTL1 |= UCTXSTT;   //����I2CΪ����ģʽ������START�ź�
	DELAY_US_BSP(150);
	/* д��һ��Ҫ���͵��� */
	UCB0TXBUF = *buffer;

	while(UCB0CTL1 & UCTXSTT);

	while (!(IFG2 & UCB0TXIFG)){
		if(UCB0STAT & UCNACKIFG) {
			UCB0CTL1 |=UCTXSTP;
			while (UCB0CTL1 & UCTXSTP);
			return FALSE;
		}
	}

	bufferLength--;
	while (bufferLength--) {
		/* д��һ��Ҫ���͵��� */
		UCA0TXBUF = *buffer;

		while (!(IFG2 & UCB0TXIFG)){
			if(UCB0STAT & UCNACKIFG) {
				UCB0CTL1 |=UCTXSTP;
				while (UCB0CTL1 & UCTXSTP);
				return FALSE;
			}
		}

		/* ָ���¸�Ҫ���͵��� */
		buffer++;
	}

	UCB0CTL1 |= UCTXSTP;                // start condition generation
	while (UCB0CTL1 & UCTXSTP);         // Ensure stop condition got sent
	return SUCCESS;
}

/**
 * Read data from specified slave on the I2C bus.
 *
 * @param addrOfTargetSlave ���豸��ַ
 * @param buffer ��Ŷ�����ָ��
 * @param bufferLength �����ĸ���
 * @return SUCCESS/FAILURE
 */
uint8_t read_I2CM(uint8_t addrOfTargetSlave, uint8_t* buffer, uint16_t bufferLength) {
    UCB0CTL1 &= ~UCTR;                      // UCTR=0 => Receive Mode (R/W bit = 1)
    IFG2 &= ~UCB0RXIFG;                     // Clear RXIFG flag

    UCB0CTL1 |= UCTXSTT;
	DELAY_US_BSP(150);
	while(UCB0CTL1 & UCTXSTT);

	if(UCB0STAT & UCNACKIFG) {
		UCB0CTL1 |=UCTXSTP;
		while (UCB0CTL1 & UCTXSTP);
		return FALSE;
	}

	bufferLength--;
	while (bufferLength--) {
		while(!(IFG2 & UCB0RXIFG));
		*buffer = UCB0RXIFG;
		/* ָ���¸�Ҫ���͵��� */
		buffer++;
	}

	UCB0CTL1 |= UCTXSTP;                // start condition generation
	while(!(IFG2 & UCB0RXIFG));
	*buffer = UCB0RXIFG;
	while (UCB0CTL1 & UCTXSTP);         // Ensure stop condition got sent

    return SUCCESS;
}

#else

#endif



