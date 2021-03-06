/**
 * Written by Tim Johns.
 *
 * The function bodies in this file are specific to the MSP430F5310 MCU.
 */

#ifndef _MSPLIB_C
#define _MSPLIB_C

#include <msp430f5310.h>
#include <stdint.h>
#include "msp430f5310_extra.h"

/*----------------------------------------------------------------------------*/
/* Configure watchdog timer (also used to feed watchdog)					  */
/*----------------------------------------------------------------------------*/
void wdt_config(void) {
// Select ACLK source, clear timer, timer interval = clock / 32k (1 s at 32 kHz)
	WDTCTL = WDTPW | WDTSSEL__ACLK | WDTCNTCL | WDTIS__32K;
}

/*----------------------------------------------------------------------------*/
/* Stop watchdog timer														  */
/*----------------------------------------------------------------------------*/
void wdt_stop(void) {
	WDTCTL = WDTPW | WDTHOLD;		// Stop watchdog timer
}

/*----------------------------------------------------------------------------*/
/* Set up and configure ADC (10 bit)										  */
/*----------------------------------------------------------------------------*/
void adc_config(void) {
// REF Master Control, Voltage Level Select 2.5 V (VREF+), Temp.Sensor off,
// Reference On
	REFCTL0 = REFMSTR | REFVSEL_3 | REFTCOFF | REFON;
	ADC10CTL0 &= ~ADC10ENC;						// Disable ADC
	ADC10CTL0 = ADC10SHT_15 | ADC10ON;			// 1024 clock cycles, ADC on
	ADC10MCTL0 = ADC10SREF_1 | ADC10INCH_3;		// VR+ = VREF+ and VR- = AVSS
// CLK/1, MODCLK source, single-channel
	ADC10CTL1 = ADC10SHP | ADC10DIV_0 | ADC10SSEL_0 | ADC10CONSEQ_0;
	ADC10CTL2 = ADC10RES;						// 10 bit resolution
	ADC10IFG = 0x0000;							// Clear interrupt flags
	ADC10CTL0 |= ADC10ENC | ADC10SC;			// Enable and read once
	while (!(ADC10IFG & ADC10IFG0));			// Wait for ready flag
}

/*----------------------------------------------------------------------------*/
/* Read and return voltage with ADC (10 bit)								  */
/*----------------------------------------------------------------------------*/
uint16_t adc_read(void) {
// Actual voltage = (ADC10MEM0 / 1024) * 2.5 * 2
// Half voltage due to resistor divider
	ADC10IFG = 0x0000;							// Clear interrupt flags
	ADC10CTL0 &= ~ADC10ENC;						// Disable ADC
	ADC10CTL0 |= ADC10ENC | ADC10SC;			// Enable and read once
	while (!(ADC10IFG & ADC10IFG0));			// Wait for ready flag
	return ADC10MEM0;
}

/*----------------------------------------------------------------------------*/
/* Set up and configure clock												  */
/*----------------------------------------------------------------------------*/
void clock_config(void) {
// Set DCO range to 4.6 - 39.0 MHz
	UCSCTL1 = DCORSEL1 | DCORSEL2;
// Set DCOCLKDIV to (32768 kHz / 1) * 366 =~ 12 MHz
// (Set DCOCLK to 24 MHz)
	UCSCTL2 = FLLN1 | FLLN2 | FLLN3 | FLLN5 | FLLN6 | FLLN8 | FLLD0;
// Set ACLK source: REFOCLK = 32 kHz,
// SMCLK source: DCOCLKDIV = 12 MHz,
// MCLK source: DCOCLKDIV = 12 MHz
	UCSCTL4 = SELA__REFOCLK | SELS__DCOCLKDIV | SELM__DCOCLKDIV;
// Set SMCLKREQEN = 0, MCLKREQEN = 0 so that SMCLK and MCLK are off during LPM3
	UCSCTL8 &= (~BIT1 & ~BIT2);
}

/*----------------------------------------------------------------------------*/
/* Enable interrupts														  */
/*----------------------------------------------------------------------------*/
void enable_interrupts(void) {
	__enable_interrupt();
}

/*----------------------------------------------------------------------------*/
/* Trigger brownout reset													  */
/*----------------------------------------------------------------------------*/
void brownout_reset(void) {
	PMMCTL0 = PMMPW | PMMSWBOR;
}

#endif