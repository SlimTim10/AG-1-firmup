/**
 * Written by Tim Johns.
 * 
 * MSP430F5310 flash functions.
 *
 */

#ifndef _FLASHLIB_C
#define _FLASHLIB_C

#include <msp430f5310.h>
#include <stdint.h>
#include "flash.h"

/*----------------------------------------------------------------------------*/
/* Erase flash segment (512 bytes) at address addr							  */
/*----------------------------------------------------------------------------*/
void erase_segment(uint16_t *addr) {
	while (BUSY & FCTL3);	// Test BUSY until ready
	FCTL3 = FWPW;			// Clear LOCK
	FCTL1 = FWPW | ERASE;	// Enable segment erase
	*addr = 0;				// Dummy write
}

/*----------------------------------------------------------------------------*/
/* Write word value w to flash at address addr								  */
/* NOTE: This function assumes that flash writing has been enabled.			  */
/*----------------------------------------------------------------------------*/
void fwrite_word(uint16_t *addr, uint16_t w) {
	while (BUSY & FCTL3);	// Test BUSY until ready
	FCTL1 = FWPW | WRT;		// Enable write
// Reverse endian of word value for writing
	w = ((uint16_t)(w << 8)) | (w >> 8);
	*addr = w;				// Write word to flash and increment pointer
}

#endif