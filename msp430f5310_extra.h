/**
 * Written by Tim Johns.
 */

#ifndef _MSPLIB_H
#define _MSPLIB_H

/* Threshold voltage for device operation = 3.0 V */
#define VOLTAGE_THRSHLD		0x0267

// ACLK (32768 Hz) timings with 16-bit timer
#define ACLK_1SEC	0x7FFF
#define ACLK_2SEC	0xFFFF

void enter_LPM(void);
void exit_LPM(void);
void wdt_config(void);
void wdt_stop(void);
void adc_config(void);
uint16_t adc_read(void);
void clock_config(void);
void enable_interrupts(void);
void brownout_reset(void);
void timer_config(void);
void timer_disable(void);
//void timer_int_en(void);
//void timer_int_dis(void);

#endif