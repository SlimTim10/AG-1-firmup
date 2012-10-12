/**
 * Written by Tim Johns.
 * 
 * MSP430F5310 flash functions library.
 *
 */

#ifndef _FLASHLIB_H
#define _FLASHLIB_H

void erase_segment(uint16_t *);
void fwrite_word(uint16_t *, uint16_t);

#endif