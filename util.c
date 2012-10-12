/**
 * Written by Tim Johns.
 * 
 * Utility functions.
 *
 */

#ifndef _UTILLIB_C
#define _UTILLIB_C

#include <stdint.h>
#include "util.h"

/*----------------------------------------------------------------------------*/
/* Parses the character c interpreting its content as a hexadecimal number	  */
/* and returns its value as a byte											  */
/*----------------------------------------------------------------------------*/
uint8_t htoi(uint8_t c) {
	if (c >= 0x30 && c <= 0x39) {
		return (c - 0x30);
	} else if (c >= 0x41 && c <= 0x46) {
		return (c - 0x37);
	} else if (c >= 0x61 && c <= 0x66) {
		return (c - 0x57);
	}

	return 0;
}

#endif