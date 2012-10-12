/**
 * Written by Tim Johns.
 *
 * Firmware updater using file (msp430-txt/TI-txt format) located on SD card.
 *
 * MCU: MSP430F5310
 *
 * The stack size should be set to 300 bytes for this project.
 */

#include <msp430f5310.h>
#include <stdint.h>
#include "spi.h"
#include "sdfat.h"
#include "msp430f5310_extra.h"
#include "circuit.h"
#include "util.h"
#include "flash.h"

uint16_t prog_start = 0x9000;	// Starting address of next program

// Location of (this) firmup program in flash memory
#define FIRMUP_ADDR_BEGIN	0x8000
#define FIRMUP_ADDR_END		0x8FFF

#define BUFF_SIZE		512		// Size of data buffer

#define CLOCK_SPEED		12		// DCO speed (MHz)

// Feed the watchdog
#define FEED_WATCHDOG	wdt_config()

// CTRL button tap or hold
#define CTRL_TAP	1
#define CTRL_HOLD	2

// Infinite loop
#define HANG()			for (;;);

uint32_t firm_file_offset(uint8_t *, struct fatstruct *);
uint8_t valid_addr(uint16_t);
void LED1_PANIC(void);
void LED1_LOW_VOLTAGE(void);

/*----------------------------------------------------------------------------*/
/* Global variables															  */
/*----------------------------------------------------------------------------*/
// (do not refer to this variable directly--use pointers)
uint8_t data_sd_buff[BUFF_SIZE];	// Data buffer for R/W to SD card

/*----------------------------------------------------------------------------*/
/* Main routine																  */
/*----------------------------------------------------------------------------*/
void main(void) {
	uint8_t version[] = "FIRMUP VERSION 20120907";	// Firmware version

	uint8_t *data_sd;		// Pointer to SD card data buffer

	uint8_t sec;			// Used for timing with RTC
	uint16_t debounce;		// Used for debouncing

	uint32_t fdataoffset;	// Offset of firmware file data
	uint8_t user_input;		// User input with UI

	struct fatstruct fatinfo;

/* Used in firmware file parsing and update */
	uint16_t *flashptr;		// Pointer to flash address
	uint16_t wval;			// Word value in firmware file parsing
	uint16_t addr;			// Address value in firmware file parsing
	uint32_t nextseg;		// Next flash segment address in file parsing
	uint8_t datacount;		// Byte data counter in firmware file parsing
	uint8_t tmp8;
	uint16_t i;				// Counter
	uint8_t data_or_addr;	// Used in firmware file parsing
	uint8_t eof;			// End of file flag
	uint16_t reset_vector;

	data_sd = data_sd_buff;	// Set pointer to data buffer address

	__disable_interrupt();	// Disable interrupts
	wdt_stop();				// Stop watchdog timer
	clock_config();			// Set up and configure the clock
	mcu_pin_config();		// Configure MCU pins
	adc_config();			// Set up ADC

	LED1_ON();
	while (ctrl_high());	// Wait for button release (from bender program)

/*******************************************************************************
*	Look for firmware file on SD card
*******************************************************************************/
	if ((fdataoffset = firm_file_offset(data_sd, &fatinfo)) == -1) {
		asm("BR prog_start");	// Go to main program
	}

///TODO change to double flash (off.....flash-flash.....flash-flash...)
/*******************************************************************************
*	Turn on LED to ask "Do you want to perform a firmware update?"
*******************************************************************************/
	TA0CCR0 = 0x7FFF;			// Count up to 0x7FFF 
// ACLK source (32768 Hz), f/1, count up to CCR0, Timer_A clear
	TA0CTL = TASSEL_1 | ID_0 | MC_1 | TACLR;

	LED1_OFF();
	while (!ctrl_high()) {	// Wait for button press
		LED1_ON();
		while (TA0R < 0x0CCC)
			if (ctrl_high())
				break;
		LED1_OFF();
		while (TA0R < 0x1998)
			if (ctrl_high())
				break;
		LED1_ON();
		while (TA0R < 0x2664)
			if (ctrl_high())
				break;
		LED1_OFF();
		while (TA0R != 0x0000)
			if (ctrl_high())
				break;
	}

/*******************************************************************************
*	Interpret user input
*	Button hold:	Confirm
*	Button tap:		Decline
*******************************************************************************/
	debounce = 0x1000;
	while (debounce--);		// Wait for debouncing

// Wait until button is released or hold time (2 sec) is met
	rtc_restart();		// Restart RTC
	sec = RTCSEC;
	while (ctrl_high() && sec < 2) {
// Get new RTCSEC value when RTC is ready
		if (rtc_rdy()) {
			sec = RTCSEC;
		}
	}

	if (sec >= 2) {		// Wake up on button hold
		user_input = CTRL_HOLD;
	} else {
		user_input = CTRL_TAP;
	}

	LED1_ON();

	while (ctrl_high());	// Wait for button release

/*******************************************************************************
*	DECLINE
*******************************************************************************/
	if (user_input == CTRL_TAP) {	// User declined
		asm("BR prog_start");		// Go to main program
	}

/*******************************************************************************
*	CONFIRM - BEGIN FIRMWARE UPDATE
*******************************************************************************/
	wdt_stop();		// Stop watchdog timer
	FCTL3 = FWPW;	// Clear LOCK

// Get current value of reset vector (word at address 0xFFFE)
	reset_vector = *((uint16_t *)0xFFFE);

	data_or_addr = 0;	// 0: parse data, 1: parse address
	wval = 0;			// Parsed word value
	addr = 0;			// Parsed address value
	nextseg = 0;		// Next flash segment address
	datacount = 0;		// Count the parsed bytes of data
	eof = 0;			// End of file flag

	while (!eof) {
		read_block(data_sd, fdataoffset);
		LED1_TOGGLE();		// Flash to show progress

		for (i = 0; i < 512 && !eof; i++) {
			tmp8 = data_sd[i];

// msp430-txt/TI-txt file format parsing
			switch (tmp8) {

			case 0x00:		// Null byte should not be in file
				eof = 1;
				LED1_PANIC();	// Flash LED to show "panic"
// Trigger brownout reset upon failure
				PMMCTL0 = PMMPW | PMMSWBOR;

			case 'q':		// 'q' marks the termination of the file
				eof = 1;
			case '@':		// @ADDR is the starting address of a section
// Write trailing bytes to flash before parsing next address
				if (datacount > 0) {
					fwrite_word(flashptr, wval);	// Write word value to flash
					*flashptr++;					// Increment flash address
					wval = 0;
					datacount = 0;
				}
				data_or_addr = 1;
				addr = 0;							// Reset for future parsing
				break;

			case 0x0D:		// New line (carriage return)
			case 0x0A:		// New line (line feed)
				if (data_or_addr == 1) {			// Finished parsing address
					if (!valid_addr(addr)) {		// Invalid address
						eof = 1;
						break;
					}
					flashptr = (uint16_t *)addr;	// Flash address to write
					data_or_addr = 0;
				}
			case ' ':		// Space separates data bytes
				if (datacount == 4) {				// Word value
					if ((uint16_t)flashptr >= nextseg) {
						erase_segment(flashptr);
						nextseg =	(uint32_t)flashptr -
									((uint32_t)flashptr % 512)
									+ 512;
					}
					fwrite_word(flashptr, wval);	// Write word value to flash
					*flashptr++;					// Increment flash address
					wval = 0;
					datacount = 0;
				}
				break;

			default:
				if (data_or_addr == 0) {	// Parse data
					wval = (wval << 4) | htoi(tmp8);
					datacount++;
				} else {					// Parse address
					addr = (addr << 4) | htoi(tmp8);
				}
				break;
			}
		}

		fdataoffset += 512;		// Next block
	}

	LED1_ON();

// Rewrite reset vector (always reset to firmware updater)
	while (BUSY & FCTL3);	// Test BUSY until ready
	FCTL1 = FWPW | WRT;		// Enable write
	*((uint16_t *)0xFFFE) = reset_vector;

	FCTL1 = FWPW;			// Clear WRT and ERASE
	FCTL3 = FWPW | LOCK;	// Set LOCK

// Delete firmware file
	write_block(data_sd, fatinfo.dtoffset, 0);	// Clear directory table
	read_block(data_sd, fatinfo.fatoffset);		// Read FAT 
	write_block(data_sd, fatinfo.fatoffset, 4);	// Write FAT as empty

	LED1_OFF();

	asm("BR prog_start");	// Go to main program
}

/*----------------------------------------------------------------------------*/
/* Look for firmware file on SD card 										  */
/* Return offset of file data on success.									  */
/* Return -1 on error.														  */
/* Firmware file must be first file in directory table and has a name in the  */
/* format FIRMW***.TXT (where * is one wild character).						  */
/*----------------------------------------------------------------------------*/
uint32_t firm_file_offset(uint8_t *data_sd, struct fatstruct *fatinfo) {
	uint8_t avail;			// Availability of slave devices
	uint16_t clust;			// Starting cluster
	uint32_t filesize;		// Size of firmware file

	wdt_config();			// Start watchdog timer

	spi_config();			// Set up SPI for MCU
	power_on(SD_PWR);		// Turn on power to SD Card

	avail = init_sd();		// Get availability of SD Card
	if (avail != 0) {		// If any slaves are not available
		return -1;
	}

	FEED_WATCHDOG;

// Find and read the FAT16 boot sector
	if (read_boot_sector(data_sd, fatinfo)) {
		return -1;
	}

	FEED_WATCHDOG;

// Parse the FAT16 boot sector
	if (parse_boot_sector(data_sd, fatinfo)) {
		return -1;
	}

	FEED_WATCHDOG;

// Read first entry in directory table
	read_block(data_sd, fatinfo->dtoffset);

	FEED_WATCHDOG;

// Check if first file name matches FIRMW***.TXT (where * is one wild character)
// If no firmware file is found, run main program
	if (!(data_sd[0] == 'F' &&
		data_sd[1] == 'I' &&
		data_sd[2] == 'R' &&
		data_sd[3] == 'M' &&
		data_sd[4] == 'W' &&
		data_sd[8] == 'T' &&
		data_sd[9] == 'X' &&
		data_sd[10] == 'T'))
	{
		return -1;
	}

// Starting cluster number of firmware file data
	clust = (data_sd[27] << 8) | data_sd[26];

// Firmware file size
	filesize =	(((uint32_t)data_sd[31]) << 24) |
				(((uint32_t)data_sd[30]) << 16) |
				(data_sd[29] << 8) |
				data_sd[28];

	wdt_stop();		// Stop watchdog timer

	return get_cluster_offset(clust, fatinfo);
}

/*----------------------------------------------------------------------------*/
/* Check validity of flash address	 										  */
/* Return 1 for valid address.												  */
/* Return 0 for address in this program's memory range or reset vector.		  */
/*----------------------------------------------------------------------------*/
uint8_t valid_addr(uint16_t a) {
	if (a >= FIRMUP_ADDR_BEGIN && a <= FIRMUP_ADDR_END)
		return 0;
	else if (a == 0xFFFE)
		return 0;
	else
		return 1;
}

/*----------------------------------------------------------------------------*/
/* Flash LED multiple times quickly to show "panic"							  */
/*----------------------------------------------------------------------------*/
void LED1_PANIC(void) {
	uint16_t i;
	uint8_t j, k;
	LED1_OFF();
	for (k = 0; k < 20; k++) {
		LED1_TOGGLE();
		for (j = 0; j < CLOCK_SPEED; j++) {
			for (i = 0; i < 8000; i++);
		}
	}
}