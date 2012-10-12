/**
 * Written by Tim Johns.
 *
 * The function bodies in this file are specific to MSP430F5310.
 */

#ifndef _SPILIB_C
#define _SPILIB_C

#include <msp430f5310.h>
#include <stdint.h>
#include "spi.h"
#include "circuit.h"

/*----------------------------------------------------------------------------*/
/* Set up SPI for master (MCU) and slaves									  */
/*----------------------------------------------------------------------------*/
void spi_config(void) {
	mcu_spi_ports();	// Set up ports for SPI

/* Set up USCI_A1 SPI */
// Clock polarity high, MSB first, master, 3-pin SPI, synchronous
	UCA1CTL0 = UCCKPL | UCMSB | UCMST | UCMODE_0 | UCSYNC;
// Clock source SMCLK, set reset bit high
	UCA1CTL1 = UCSSEL__SMCLK | UCSWRST;
    UCA1BR1 = 0;					// Upper byte of divider word
	UCA1BR0 = 2;					// Clock = SMCLK / 2
    UCA1CTL1 &= ~UCSWRST;			// Release from reset
}

/*----------------------------------------------------------------------------*/
/* Transmit byte to USCI_A1 SPI slave and return received byte				  */
/*----------------------------------------------------------------------------*/
uint8_t spia_send(const uint8_t b) {
	while ((UCA1IFG & UCTXIFG) == 0);	// Wait while not ready
	UCA1TXBUF = b;						// Transmit
	while ((UCA1IFG & UCRXIFG) == 0);	// Wait for RX buffer (full)
	return (UCA1RXBUF);
}

/*----------------------------------------------------------------------------*/
/* Receive and return byte from USCI_A1 SPI slave							  */
/*----------------------------------------------------------------------------*/
uint8_t spia_rec(void) {
	while ((UCA1IFG & UCTXIFG) == 0);	// Wait while not ready
	UCA1TXBUF = 0xFF;					// Dummy byte to start SPI
	while ((UCA1IFG & UCRXIFG) == 0);	// Wait for RX buffer (full)
	return (UCA1RXBUF);
}

#endif