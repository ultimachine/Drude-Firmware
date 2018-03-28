/*
             LUFA Library
     Copyright (C) Dean Camera, 2013.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2013  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the VirtualSerialDigitizer demo. This file contains the main tasks of
 *  the demo and is responsible for the initial application hardware configuration.
 */

#include <stdio.h>      /* printf, scanf, NULL */
#include <stdlib.h>     /* malloc, free, rand */

#include "VirtualSerialDigitizer.h"
#include "myserial.h"
#include "i2c.h"

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber         = 0,
				.DataINEndpoint                 =
					{
						.Address                = CDC_TX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.DataOUTEndpoint                =
					{
						.Address                = CDC_RX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.NotificationEndpoint           =
					{
						.Address                = CDC_NOTIFICATION_EPADDR,
						.Size                   = CDC_NOTIFICATION_EPSIZE,
						.Banks                  = 1,
					},
			},
	};

/** Buffer to hold the previously generated Digitizer HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevDigitizerHIDReportBuffer[sizeof(USB_DigitizerReport_Data_t)];

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Digitizer_HID_Interface =
	{
		.Config =
			{
				.InterfaceNumber                = 2,
				.ReportINEndpoint               =
					{
						.Address                = DIGITIZER_EPADDR,
						.Size                   = DIGITIZER_EPSIZE,
						.Banks                  = 1,
					},
				.PrevReportINBuffer             = PrevDigitizerHIDReportBuffer,
				.PrevReportINBufferSize         = sizeof(PrevDigitizerHIDReportBuffer),
			},
	};


static RingBuffer_t FromHost_Buffer;
static uint8_t  FromHost_Buffer_Data[128];

void HandleSerial();
char cmd[30];
int cmd_cnt;

void putchar_printf(char var, FILE *stream)
{
	CDC_Device_SendByte(&VirtualSerial_CDC_Interface, var);
}

char my_getchar(char var, FILE *stream)
{
	return CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
}

static FILE mystdout = FDEV_SETUP_STREAM(putchar_printf, NULL, _FDEV_SETUP_WRITE);
static FILE  mystdin = FDEV_SETUP_STREAM(NULL, my_getchar, _FDEV_SETUP_READ);

#define RESET_PIN (_BV(PB3))
#define RESETN_PIN (_BV(PB6))
void init_screen(uint8_t);

uint8_t debug_output=0;

uint16_t xpos;
uint16_t ypos;
uint8_t multitouch_id=0;
uint8_t touch_state;
uint8_t touch_pressure=0;

uint8_t report_available=0;
uint8_t msgs_available=0;

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	stdout=&mystdout;
	stdin=&mystdin;

	SetupHardware();

	//LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	GlobalInterruptEnable();

	//Eval Prototype Sharp Reset Pin
	//DDRB &= ~RESET_PIN; //Set Input
	//PORTB |= RESET_PIN; //Set High
	//DDRB |= RESET_PIN; //Set Output

	//Drude Sharp LCD Reset PIN
	PORTF |= RESETN_PIN; //Set High
	DDRF |= RESETN_PIN; //Set Output
	
	//Digitizer Interrupt
	//DDRB &= ~(_BV(7)); //PB7 input
	//PORTB &= ~(1<<PB7); //PB7 low
	PORTB |= (_BV(7)); //Set high (input pullup)
	
	//IP4787CZ32Y HDMI ESD interface chip (HDMI_ACT PIN) Active-High (Test-Point 12)
	PORTF |= (_BV(PF1)); //Set high (input pullup)
	
	//Toshiba Interrupt Pin
	PORTE |= (_BV(PE6)); //Set high (input pullup)
	//Toshiba Standby Pin
	PORTF |= (_BV(PF4)); //Set high (input pullup)
	//Toshiba Reset Pin - Active Low
	PORTC |= (_BV(PC7)); //Set high (input pullup)

	//LCD-CABC
	//PORTF |= (_BV(PF7)); //Set high (defaults to input pullup)
	//DDRF &= ~(_BV(PF7)); //Set output
	//PORTF |= (_BV(PF7)); //Set low

	//LED-PWM
	PORTD |= (_BV(PD6)); //Set high (defaults to input pullup)
	//PORTD &= ~(_BV(PD6)); //Set low
	//DDRD &= ~(_BV(PD6)); //Set output


	RingBuffer_InitBuffer(&FromHost_Buffer, FromHost_Buffer_Data, sizeof(FromHost_Buffer_Data));

	init_screen(0x1F); //magic number!
	
	mxt_list_types();

	for (;;)
	{
		HandleSerial();
		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		HID_Device_USBTask(&Digitizer_HID_Interface);
		USB_USBTask();
		HandleDigitizer();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Hardware Initialization */
	//LEDs_Init();
	USB_Init();

	TWI_Init();
/*
    PCICR |= (1 << PCIE0);     // enable PCMSK0 scan on PCIE0
    PCMSK0 |= (1 << PCINT7);   // PCINT7 (PB7) to trigger interrupt on pin change
	sei();
*/

}
/*
ISR (PCINT0_vect) {
	if ( (PINB &(1<<PB7)) == 0 ) {
		msgs_available = 1;
		if(debug_output) printf("PB7low\n");
	} else {
		if(debug_output) printf("PB7high\n");
	}
	
}
*/

void HandleDigitizer()
{
	if ( (PINB &(1<<PB7)) == 0 ) msgs_available = 1;
	//if( (1<<PB7)&PINB == 0 ) proc_msgs();
	if( msgs_available ) proc_msgs();
}

void HandleSerial(void)
{
	// Only try to read in bytes from the CDC interface if the transmit buffer is not full 
	if (!(RingBuffer_IsFull(&FromHost_Buffer)))
	{
		int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
		// Read bytes from the USB OUT endpoint into the USART transmit buffer 
		if (!(ReceivedByte < 0)){
		  	RingBuffer_Insert(&FromHost_Buffer, ReceivedByte);
			CDC_Device_SendByte(&VirtualSerial_CDC_Interface, ReceivedByte);
		}
	}

	while (RingBuffer_GetCount(&FromHost_Buffer) > 0)
	{
		int16_t c = RingBuffer_Remove(&FromHost_Buffer);

		if(c == '\n' || c == '\r'){
			if(cmd_cnt > 0 && (cmd[cmd_cnt-1] == '\n' || cmd[cmd_cnt-1] == '\r'))
				cmd_cnt--;
			cmd[cmd_cnt] = 0;			
			execute_command();
			cmd_cnt = 0;			
		}
		else{
			cmd[cmd_cnt++] = c;			
		}
	}
}

uint16_t value;

uint8_t crc8(unsigned char, unsigned char);
uint8_t crc81(unsigned char, unsigned char);
void mxt_write_checksum(uint16_t, uint8_t);
void mxt_read_checksum(uint16_t);
void proc_msgs();


//treat certain addresses as special commands for init sequence
#define SEQ_SIZE 0xF0
#define SEQ_DELAY 0xF1
#define SEQ_RESET 0xF2
#define SEQ_READ 0xF3
#define SEQ_END 0xFF


const uint8_t PROGMEM init_sequence[] = { 
SEQ_RESET,
SEQ_DELAY,
SEQ_SIZE, 2,
0x00,0x04,0x04,0x00,
0x00,0x02,0x00,0x0F,
0x00,0x02,0x00,0x00,
0x00,0x06,0xA0,0x00,
0x00,0x0A,0xA8,0x0C,
0x00,0x14,0xFF,0x07,
0x00,0x16,0xFF,0x07,

0x00,0x20,0xFA,0x60,
0x00,0x22,0x13,0x02,
0x00,0x60,0x01,0x00,
0x70,0x80,0x80,0x00,

0x00,0x24,0x24,0x14,
0x00,0x26,0x08,0x02,


SEQ_SIZE, 4,
0x01,0x40,0x00,0x00,0x00,0x00,
0x01,0x44,0x00,0x00,0x00,0x00,
0x01,0x48,0x00,0x00,0x00,0x00,
0x01,0x4C,0x00,0x00,0x00,0x00,
0x01,0x50,0x00,0x00,0x00,0x00,

0x02,0x10,0x00,0x31,0x00,0x00,
0x02,0x14,0x06,0x00,0x00,0x00,
0x02,0x18,0x04,0x24,0x00,0x00,
0x02,0x1C,0x03,0x00,0x00,0x00,
0x02,0x20,0x06,0x07,0x00,0x00,
0x02,0x24,0x00,0x4A,0x00,0x00,
0x02,0x28,0x0B,0x00,0x00,0x00,
0x02,0x2C,0x05,0x00,0x00,0x00,
0x02,0x34,0x1F,0x00,0x00,0x00,
0x02,0x38,0x01,0x00,0x00,0x00,
0x02,0x04,0x01,0x00,0x00,0x00,
0x05,0x18,0x01,0x00,0x00,0x00,
0x05,0x00,0xA7,0x00,0x00,0xA3,
0x05,0x00,0x01,0x80,0x00,0xC3,

SEQ_SIZE, 2,
0x07,0x0C,0x15,0x10,
0x07,0x0E,0x00,0x00,
0x07,0x10,0xFF,0x10,
0x07,0x00,0x05,0x00,

0x07,0x0C,0x15,0x10,
0x07,0x0E,0x00,0x00,
0x07,0x10,0xBB,0x10,
0x07,0x00,0x05,0x00,

0x07,0x0C,0x15,0x10,
0x07,0x0E,0x00,0x00,
0x07,0x10,0xB0,0x15,
0x07,0x00,0x05,0x00,

0x07,0x0C,0x39,0x40,
0x07,0x0E,0x06,0x00,
0x07,0x10,0x3B,0x03,
0x07,0x12,0x06,0x04,
0x07,0x14,0x3C,0x66,
0x07,0x00,0x05,0x00,

0x07,0x0C,0x15,0x10,
0x07,0x0E,0x00,0x00,
0x07,0x10,0xFF,0x10,
0x07,0x00,0x05,0x00,

0x07,0x0C,0x15,0x10,
0x07,0x0E,0x00,0x00,
0x07,0x10,0x35,0x00,
0x07,0x00,0x05,0x00,

0x07,0x0C,0x05,0x10,
0x07,0x0E,0x00,0x00,
0x07,0x10,0x11,0x00,
0x07,0x00,0x05,0x00,

SEQ_DELAY,
0x07,0x0C,0x15,0x10,
0x07,0x0E,0x00,0x00,
0x07,0x10,0x51,0xFF,
0x07,0x00,0x05,0x00,

0x07,0x0C,0x15,0x10,
0x07,0x0E,0x00,0x00,
0x07,0x10,0x53,0x24,
0x07,0x00,0x05,0x00,

0x07,0x0C,0x15,0x10,
0x07,0x0E,0x00,0x00,
0x07,0x10,0x55,0x00,
0x07,0x00,0x05,0x00,

0x07,0x0C,0x15,0x10,
0x07,0x0E,0x00,0x00,
0x07,0x10,0xFF,0xF0,
0x07,0x00,0x05,0x00,

0x07,0x0C,0x15,0x10,
0x07,0x0E,0x00,0x00,
0x07,0x10,0x92,0x01,
0x07,0x00,0x05,0x00,

0x07,0x0C,0x15,0x10,
0x07,0x0E,0x00,0x00,
0x07,0x10,0xFF,0x10,
0x07,0x00,0x05,0x00,

0x07,0x0C,0x05,0x10,
0x07,0x0E,0x00,0x00,
0x07,0x10,0x29,0x00,
0x07,0x00,0x05,0x00,

0x07,0x00,0x00,0x00,

SEQ_SIZE, 1,
0x85,0x02,0x01,
0x85,0x12,0xFE,
0x85,0x31,0x00,
0x85,0x34,0x3E,
0x85,0x33,0x07,
SEQ_SIZE, 2, 
0x85,0x40,0x8C,0x0A,
SEQ_SIZE, 1,
0x85,0x52,0xD1,
0x86,0x30,0xB0,
SEQ_SIZE, 2, 
0x86,0x31,0x1E,0x04,
SEQ_SIZE, 1,
0x86,0x70,0x01,
0x85,0x32,0x80,
0x85,0x36,0x40,
0x85,0x3D,0x03,
0x85,0x3F,0x0A,
0x85,0x43,0x32,
0x85,0x44,0x10,
0x85,0x45,0x31,
0x85,0x46,0x2D,
0x85,0x71,0x02,
0x85,0xAA,0x50,
0x85,0xAB,0x00,
0x85,0xAF,0xF6,
0x85,0xC7,0x01,
0x85,0xCB,0x01,

SEQ_SIZE, 8,
0x8C,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,
0x8C,0x08,0x52,0x62,0x88,0x88,0x00,0x88,0x88,0x88,
0x8C,0x10,0x1C,0x15,0x01,0x03,0x80,0x00,0x00,0x78,
0x8C,0x18,0x0A,0xDA,0xFF,0xA3,0x58,0x4A,0xA2,0x29,
0x8C,0x20,0x17,0x49,0x4B,0x00,0x00,0x00,0x01,0x01,
0x8C,0x28,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x8C,0x30,0x01,0x01,0x01,0x01,0x01,0x01,0x53,0x39,
0x8C,0x38,0x38,0xA0,0x40,0x80,0x37,0x70,0x30,0x20,
0x8C,0x40,0x3A,0x00,0x38,0x80,0x47,0x00,0x00,0x1E,
0x8C,0x48,0x8C,0x0A,0xD0,0x8A,0x20,0xE0,0x2D,0x10,
0x8C,0x50,0x10,0x3E,0x96,0x00,0x13,0x8E,0x21,0x00,
0x8C,0x58,0x00,0x1E,0x00,0x00,0x00,0xFC,0x00,0x54,
0x8C,0x60,0x6F,0x73,0x68,0x69,0x62,0x61,0x2D,0x48,
0x8C,0x68,0x32,0x43,0x0A,0x20,0x00,0x00,0x00,0xFD,
0x8C,0x70,0x00,0x17,0x3D,0x0F,0x8C,0x17,0x00,0x0A,
0x8C,0x78,0x20,0x20,0x20,0x20,0x20,0x20,0x01,0xC4,
0x8C,0x80,0x02,0x03,0x17,0x74,0x47,0x84,0x13,0x03,
0x8C,0x88,0x02,0x07,0x06,0x01,0x23,0x09,0x07,0x01,
0x8C,0x90,0x66,0x03,0x0C,0x00,0x30,0x00,0x80,0x8C,
0x8C,0x98,0x0A,0xD0,0x8A,0x20,0xE0,0x2D,0x10,0x10,
0x8C,0xA0,0x3E,0x96,0x00,0xC4,0x8E,0x21,0x00,0x00,
0x8C,0xA8,0x18,0x8C,0x0A,0xD0,0x8A,0x20,0xE0,0x2D,
0x8C,0xB0,0x10,0x10,0x3E,0x96,0x00,0x13,0x8E,0x21,
0x8C,0xB8,0x00,0x00,0x18,0x8C,0x0A,0xA0,0x14,0x51,
0x8C,0xC0,0xF0,0x16,0x00,0x26,0x7C,0x43,0x00,0xC4,
0x8C,0xC8,0x8E,0x21,0x00,0x00,0x98,0x8C,0x0A,0xA0,
0x8C,0xD0,0x14,0x51,0xF0,0x16,0x00,0x26,0x7C,0x43,
0x8C,0xD8,0x00,0x13,0x8E,0x21,0x00,0x00,0x98,0x00,
0x8C,0xE0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x8C,0xE8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x8C,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x8C,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2E,

SEQ_SIZE, 1,
0x85,0xD1,0x01,
0x85,0x60,0x24,
0x85,0x63,0x11,
0x85,0x64,0x0F,

0x85,0x74,0x08,
0x85,0x73,0x00,
0x85,0x76,0xA0,

0x86,0x00,0x00,
0x86,0x02,0xF3,
0x86,0x03,0x02,
0x86,0x04,0x0C,
0x86,0x06,0x05,
0x86,0x07,0x00,
0x86,0x20,0x22,
0x86,0x40,0x01,
0x86,0x41,0x65,
0x86,0x42,0x07,
0x86,0x51,0x02,
0x86,0x52,0x02,
0x86,0x65,0x10,

0x87,0x0B,0x2C,
0x87,0x0C,0x53,
0x87,0x0D,0xFF,
0x87,0x0E,0x30,
0x90,0x07,0x10,
0x85,0x31,0x01,
0x85,0x34,0x3F,
0x85,0x4A,0x01,

SEQ_READ,0x85,0x20,
SEQ_DELAY,

SEQ_SIZE,4,
0x60,0x00,0x59,0x00,0x0F,0x80,
0x40,0x00,0x00,0x00,0x10,0x00,

0x40,0x80,0x04,0x00,0x00,0x00,
0x40,0x84,0x00,0x01,0x03,0x00,
0x40,0x88,0x38,0x04,0x00,0x00,
0x40,0x8C,0x80,0x07,0x00,0x00,
0x41,0x70,0x0C,0x00,0x00,0x00,
0x41,0x74,0x00,0x01,0x00,0x12,
0x41,0x78,0x04,0x80,0x01,0x00,
0x43,0x80,0x00,0x0A,0x00,0x00,
0x43,0x84,0x80,0x07,0x38,0x04,
0x44,0x84,0x20,0x03,0x00,0x05,

0x60,0x04,0x10,0x00,0x00,0x00,
0x60,0x08,0x3C,0xC7,0x00,0x00,
0x60,0x0c,0x09,0x00,0x00,0x00,
0x60,0x10,0x29,0x00,0x00,0x00,
0x60,0x14,0x7F,0x07,0x00,0x00,
0x60,0x18,0x02,0x00,0x00,0x00,
0x60,0x1C,0x1F,0x00,0x00,0x00,
0x60,0x20,0x4F,0x00,0x00,0x00,
0x60,0x24,0x37,0x04,0x00,0x00,
0x60,0x28,0x2F,0x00,0x00,0x00,

0x52,0x04,0x01,0x00,0x00,0x00,
0x52,0x08,0x00,0x00,0x00,0x00,
0x52,0x0C,0xA1,0x12,0x00,0x00,
0x52,0x10,0xFF,0xF2,0xBE,0xF9,
0x52,0x14,0x00,0x00,0x00,0x00,
0x52,0x18,0x00,0x00,0x00,0x00,
0x52,0x1C,0xA1,0x12,0x00,0x00,
0x52,0x20,0x05,0x00,0x07,0x24,
0x52,0x24,0x00,0x00,0x00,0x00,
0x52,0x28,0x00,0x00,0x00,0x00,
0x52,0x2C,0xA1,0x12,0x00,0x00,
0x52,0x30,0x88,0x19,0xFD,0xFF,
0x52,0x34,0x00,0x00,0x00,0x00,

0x50,0x44,0x80,0x07,0x38,0x04,
0x50,0x48,0x80,0x07,0x38,0x04,
0x50,0x4C,0x20,0x03,0x00,0x05,
0x50,0x50,0x01,0x00,0x00,0x05,
0x50,0x54,0x38,0x04,0x00,0x00,
0x50,0x5C,0xF8,0xD7,0x00,0x00,
0x50,0x70,0x01,0x00,0x00,0x05,
0x50,0x74,0x38,0x04,0x00,0x00,
0x50,0x7C,0xF8,0xD7,0x00,0x00,
0x50,0x90,0x00,0x00,0x00,0x00,
0x50,0x94,0x80,0x07,0x00,0x00,
0x50,0x9C,0xD8,0x66,0x02,0x00,
0x50,0xA0,0x00,0x00,0x00,0x00,
0x50,0xA4,0x80,0x07,0x00,0x00,
0x50,0xAC,0xD8,0x66,0x02,0x00,

SEQ_SIZE,2,
0x07,0x02,0x34,0x00,
0x07,0x06,0x80,0x07,
0x07,0x08,0x71,0x01,
0x07,0x00,0x03,0x00,
0x00,0x04,0x17,0x0C,
SEQ_END
};



#define HDMICONV_ADDR0 0x0F //0x0F on eval, 0x1F on drude
#define HDMICONV_ADDR1 0x1F //0x0F on eval, 0x1F on drude

/*
// THIS FUNCTION UNDER CONSTRUCTION
int hdmiconv_send(const uint8_t *buf, int count){
	int stat;
	//uint8_t sendbyte;

	for(int i=0; i<count; i++){				// write the data
		TWI_Write(buf[i]);
		stat = TWI_GetStatus();
		if (stat != 0x28)return stat;
	}
	TWI_Stop();							// Send stop condition 

	return 0;
}
* */

void printDescriptor(const uint8_t *desc,uint16_t size)
{
	uint8_t val;
	
	for(uint16_t i=0; i<size;i++)
	{
		val = pgm_read_byte(desc++);
		printf_P(PSTR("0x%02X "),val);
	}
	printf("\n");
	
}


#if !defined(ARRAY_SIZE)
    #define ARRAY_SIZE(x) (sizeof((x)) / sizeof((x)[0]))
#endif

uint16_t init_size = ARRAY_SIZE(init_sequence);

uint8_t print_sequence(const uint8_t *buf, uint8_t i2c_addr)
{
	uint8_t value;
	uint8_t size;
	uint8_t addr_msb = SEQ_END;
	uint8_t addr_lsb;
	uint8_t stat;
	
	size = 2;
	
	//printf(PSTR("INIT SIZE: %u\n"), init_size);
	
	while(1)
	{
		addr_msb = pgm_read_byte(buf++); //read address or command, then set buf to point to the data
		if(debug_output) printf("%02X ",addr_msb);
		
		if(addr_msb == SEQ_SIZE) {
			size = pgm_read_byte(buf++); //read the size from buf, then set buf to next register address or command
			if(debug_output) printf_P(PSTR("SEQ_SIZE %u\n"), size);
			continue;
		}
		else if(addr_msb == SEQ_END) {
			break;
		}
		else if(addr_msb == SEQ_DELAY) {
			//value = pgm_read_byte(buf++);
			_delay_ms(100);
			if(debug_output) printf_P(PSTR("SEQ_DELAY\n"));
			continue;
		}
		else if(addr_msb == SEQ_RESET) {
			if(debug_output) printf_P(PSTR("SEQ_RESET\n"), value);

			//Eval SharpLCD Reset Pin
			PORTB |= RESET_PIN; //Set High
			DDRB |= RESET_PIN; //Set Output
			//Drude SharpLCD Reset Pin
			PORTB |= RESETN_PIN; //Set High
			DDRB |= RESETN_PIN; //Set Output
			_delay_us(10);	//Wait 10 microseconds
			
			PORTB &= ~(RESETN_PIN); //Drude Reset Pin Set Low
			PORTB &= ~(RESET_PIN); //Eval Reset Pin Set Low
			_delay_us(10);	//Wait 10 microseconds
			
			PORTB |= RESET_PIN; //Eval Reset Pin Set High
			PORTB |= RESETN_PIN; //Drude Toshina Reset Pin Set High
			_delay_ms(10);
			//DDRB &= ~(RESET_PIN); //Set Input

			continue;
		}
		else if(addr_msb == SEQ_READ) {
			value = pgm_read_byte(buf++);
			addr_lsb = pgm_read_byte(buf++);
			if(debug_output) printf_P(PSTR("SEQ_READ %02X %02X\n"), value,addr_lsb);
			continue;
		};
		

		TWI_Start();						// First start condition 
		stat = TWI_GetStatus();
		if (stat != 0x08) { if(debug_output) printf_P(PSTR("\nTWI_Start()\n")); return stat; }

		TWI_Write((i2c_addr<<1));		// Chip address + write
		stat = TWI_GetStatus();
		if (stat != 0x18) { if(debug_output) printf_P(PSTR("\nTWI_Write(HDMICONV_ADDR)\n")); return stat; }

		/*
		if( ! i2c_start(HDMICONV_ADDR) ) {
			printf_P(PSTR("i2c_start failed.\n"));
		} */

		TWI_Write(addr_msb);			// Address high byte
		stat = TWI_GetStatus();
		if (stat != 0x28) { if(debug_output) printf_P(PSTR("\nTWI_Write(addr_msb)\n")); return stat; }
		
		for(uint8_t i=0;  i<=size && i<10  ; i++)
		{
			value = pgm_read_byte(buf++);
			if(debug_output) printf("%02X ",value);

			TWI_Write(value);
			stat = TWI_GetStatus();
			if (stat != 0x28) return stat;
			
		}

		TWI_Stop();							// Send stop condition
		_delay_us(5); 		
		if(debug_output) printf("\n");
	}

}

void init_screen(uint8_t i2c_address)
{
	//_delay_ms(110); //TOO FAST, WILL NOT TURN ON.
	//_delay_ms(115); //WORKS
	//_delay_ms(112); //SOMETIMES WORKED THEN STOPPED WORKING
	_delay_ms(150);
	print_sequence(init_sequence,i2c_address);
}

void i2c_scan()
{
	uint8_t ret;
	for(uint8_t i=1;i<128;i++)
	{
		TWI_Stop();
		ret = i2c_start(i);
		if( ret == 0 ) {
			printf_P(PSTR("Found i2c device: %02X\n"),i);
		} 
		else {
			printf_P(PSTR("NA: %02X  Ret: %02X\n"),i, ret);
		}
	}
}



/*
//void print_sequence(const uint8_t *buf, uint8_t linecount, uint8_t length)
void print_sequence(const uint8_t *buf)
{
	uint8_t value;
	uint8_t size=2;
	
	//uint8_t linecount = pgm_read_byte(buf++);
	//uint8_t linelength = pgm_read_byte(buf++);
	
	for(uint8_t i=0; i < linecount; i++)
	{
		for(uint8_t idx=0; idx < linelength; idx++)
		{
			value = pgm_read_byte(buf++);
			printf("%02x ",value);
		}
		
		printf("\n");
	}
}
} */

/*
void hdmi_send_sequence(const uint8_t &sequence)
{
	
}

void hdmi_init()
{

}
*/

void execute_command(){
	//if(strcmp_PF(cmd, PSTR("hi")) == 0)
	if(strncmp(cmd, "hi", 2) == 0)
	{
		unsigned long val=4000000;
		val = atol(cmd+3);
		//MyPrintpgmln("hello");
		printf_P(PSTR("hello val: %lu\r\n"),val);
		printf_P(PSTR("hello again %lx\r\n"),0xff00aa22);
	}

	else if(strncmp(cmd, "read ", 5) == 0)
	{
		int val;
		uint8_t grrvalue;
		val = atoi(cmd+5);
		//MyPrintpairln("read: ",val);
		printf_P(PSTR("read %u \n"),val);

		i2c_recv_checksum(val,&grrvalue,1);
		printf_P(PSTR("grrvalue %u \n ") ,grrvalue);
	}

	else if(strncmp(cmd, "write ", 6) == 0)
	{
		int val;
		val = atoi(cmd+6);
		//MyPrintpairln("read: ",val);

		mxt_write(val, value);
	}

	else if(strncmp(cmd, "value ", 6) == 0)
	{
		value = atoi(cmd+6);
		MyPrintpairln("value: ",value);
	}
	else if(strncmp(cmd, "dbg ", 4) == 0)
	{
		debug_output = atoi(cmd+4);
		printf_P(PSTR("debug_output: %u"),debug_output);
	}
	else if(strncmp(cmd, "crc ", 4) == 0)
	{
		//value = atoi(cmd+4);
		//MyPrintpairln("value: ",value);
		printf_P(PSTR("value: %u 0x%x crc8: %u 0x%x \n"),value,value,crc8(0,value),crc81(0,value) );
	}
	else if(strncmp(cmd, "shift", 4) == 0)
	{
		printf_P(PSTR("91: 0x%02X  shifted: 0x%02X \n"),91,91>>7 );
	}

	else if(strncmp(cmd, "word ", 5) == 0)
	{
		int val;
		val = atoi(cmd+5);
		//MyPrintpairln("read: ",val);

		mxt_word(val);
	}

	else if(strcmp_P(cmd,PSTR( "init")) == 0)
	{
		printFreeMem();
		mxt_init();
		//MyPrintlnpgm("no init yet");
		//printFreeMem();
	}
	else if(strcmp_P(cmd,PSTR( "header")) == 0)
	{
		mxt_header();
	}
	else if(strcmp_P(cmd,PSTR( "types")) == 0)
	{
		mxt_list_types();
	}
	else if(strcmp_P(cmd,PSTR( "mem")) == 0)
	{
		printFreeMem();
	}
	else if(strcmp_P(cmd,PSTR( "rep")) == 0)
	{
		mxt_read_num_messages();
	}
	else if(strcmp_P(cmd,PSTR( "screen")) == 0)
	{
		init_screen(HDMICONV_ADDR0);
	}
	else if(strcmp_P(cmd,PSTR( "msg")) == 0)
	{
		mxt_read_T5_messages();
	}
	else if(strcmp_P(cmd,PSTR( "desc")) == 0)
	{
		printDescriptor(&DigitizerReport,DigitizerReport_Size);
	}
	else if(strcmp_P(cmd,PSTR( "descsize")) == 0)
	{
		printf_P(PSTR("DigitizerReport_Size: %u\n"),DigitizerReport_Size);
	}
	else if(strcmp_P(cmd,PSTR( "toshint")) == 0) //Command for Toshiba Interrupt Pin Status
	{
		printf_P(PSTR("pe6: %u \n"),  (_BV(PE6)&PINE) );
	}
	else if(strcmp_P(cmd,PSTR( "proc")) == 0)
	{
		proc_msgs();
	}
	else if(strcmp_P(cmd,PSTR( "rall")) == 0)
	{
		mxt_write_T6_report_all();
	}
	else if(strcmp_P(cmd,PSTR( "seq0")) == 0)
	{
		uint8_t retval;
		debug_output=1;
		retval = print_sequence(init_sequence,HDMICONV_ADDR0);
		TWI_Stop();
		printf_P(PSTR("\ni2c RET: %02X\n"),retval);
		debug_output=0;
	}
	else if(strcmp_P(cmd,PSTR( "seq1")) == 0)
	{
		uint8_t retval;
		debug_output=1;
		retval = print_sequence(init_sequence,HDMICONV_ADDR1);
		TWI_Stop();
		printf_P(PSTR("\ni2c RET: %02X\n"),retval);
		debug_output=0;
	}
	else if(strcmp_P(cmd,PSTR( "blink")) == 0)
	{
		DDRE &= ~(1<<PE6); //Set Input
		PORTE |= _BV(PE6); //Set High
		_delay_ms(500);
		PORTE &= ~(_BV(PE6)); //Set Low
		DDRE |= (1<<PE6); //Set Output
		_delay_ms(500);
	}
	else if(strcmp_P(cmd,PSTR( "blinkp")) == 0)
	{
		DDRE &= ~(1<<PE6); //Set Input
		PORTE |= _BV(PE6); //Set High
		_delay_ms(500);
		//DDRE |= (1<<PE6); //Set Output
		PORTE &= ~(_BV(PE6)); //Set Low
		_delay_ms(500);
	}
	else if(strcmp_P(cmd,PSTR( "digint")) == 0) //Digitizer Interrupt Pin Status
	{
		printf_P(PSTR("pb7: %u \n"),  (_BV(PB7)&PINB) );
	}
	else if(strcmp_P(cmd,PSTR( "scan")) == 0)
	{
		printf_P(PSTR("Starting i2c scan\n"));
		i2c_scan();
	}
	else
	{
		printf_P(PSTR("Unknown! command list: hi, init"));
	}

	printf_P(PSTR("> "));

}

int __mxt_read_reg(uint16_t reg, uint16_t len, void *val)
{
	int ret;

	ret = i2c_recv(reg, val, len);
	if (ret)
		if(debug_output) printf_P(PSTR("__mxt_read_reg(%04x, %04x, %04x) errorcode: %04x  "), reg, len, (unsigned int) val, ret);

	return ret;
}

int __mxt_write_reg(uint16_t reg, uint16_t len, const void *val)
{
	int ret;

	ret = i2c_send(reg, val, len);
	if (ret)
		if(debug_output) printf_P(PSTR("__mxt_write_reg() failed: %04x\n"), ret);

	return ret;
}



uint8_t mxt_get_num_objects()
{
	int ret;
	uint8_t val;

	//ret = __mxt_read_reg(6,1,&val);  //Address 6
	ret = i2c_recv(6,&val,1);  //Address 6
	if(debug_output) printf_P(PSTR("num objects: "));
	if(ret){
		if(debug_output) printf_P(PSTR("error\n"));
		return 0;
	}
	else
	{
		if(debug_output) printf_P(PSTR("%u\n"),val);
		return val;
	}	
}

uint32_t mxt_get_info_checksum(uint16_t csum_addr)
{
	int ret;
	uint8_t csum_array[3];
	uint32_t csum = 0;

	if( (ret = i2c_recv(csum_addr, &csum_array, 3)) != 0 ) //Address 6
	{
		if(debug_output) printf_P(PSTR("mxt_get_info_checksum(): i2c_recv error: %u"),ret);
		return 0xffffffff;
	}
	else
	{
		//printf_P(PSTR("csum_array: 0x%2x%2x%2x\n"),csum_array[2],csum_array[1],csum_array[0] );
		csum = csum_array[0] | csum_array[1] << 8 | (uint32_t)csum_array[2] << 16;
		//printf_P(PSTR("csum: %lx \n"), csum);
		return csum;
	}	
}

void mxt_header()
{
	int ret;
	uint8_t val[7];

	//uint8_t *val;
	//if( (val = malloc(7)) == NULL ) MyPrintpgmln("malloc(7) error");

	if( (ret = i2c_recv(0,&val,7)) != 0 )
	{
		printf_P(PSTR("mxt_header(): i2c_recv error: 0x%X"),ret);
		return;
	}

	printf_P(PSTR("header: %u %u %u %u %u %u %u\n"),val[0],val[1],val[2],val[3],val[4],val[5],val[6]);
}


/* Update 24-bit CRC with two new bytes of data */
uint32_t crc24_step(uint32_t crc, uint8_t byte1, uint8_t byte2)
{
	const uint32_t crcpoly = 0x80001b;
	uint16_t data = byte1 | (byte2 << 8);
	uint32_t result = data ^ (crc << 1);

	/* XOR result with crcpoly if bit 25 is set (overflow occurred) */
	if (result & 0x01000000)
		result ^= crcpoly;

	return result & 0x00ffffff;
}

uint32_t crc24_block(uint32_t crc, const uint8_t *data, uint8_t len)
{
	size_t i;

	for (i = 0; i < len - 1; i += 2)
	{
		//printf_P(PSTR("crc%i:0x%lx %u %u \n"), i, crc, data[i], data[i + 1] );
		crc = crc24_step(crc, data[i], data[i + 1]);
	}

	/* If there were an odd number of bytes pad with 0 */
	if (i < len)
	{
		//printf_P(PSTR("crc%i:0x%lx %u pad:0 \n"), i, crc, data[i] );
		crc = crc24_step(crc, data[i], 0);
	}

	return crc;
}


uint8_t info_blk[253];
uint8_t info_blk_size;
uint8_t num_objects;

uint8_t msg_buf[100];

uint16_t T5_addr;
uint8_t T5_size;
uint8_t T5_reports;

uint16_t T6_addr;

uint16_t T44_addr;
uint8_t T44_msg_count;

uint16_t T100_addr;
uint8_t T100_size;
uint8_t T100_reports; //possibly pointless restructured WIP code.
uint8_t T100_reports_address; //possibly pointless restructured WIP code.
uint8_t T100_report_id_min=0;
//uint8_t T100_report_id_max;


uint8_t mxt_get_num_messages()
{
	return __mxt_read_reg(T44_addr, 1, &T44_msg_count);	
}

void mxt_read_num_messages()
{
	uint8_t ret;

	ret = __mxt_read_reg(T44_addr, 1, &T44_msg_count);
	//if(debug_output) printf_P(PSTR("number_messages: "));
	if(ret)
	{
		if(debug_output) printf_P(PSTR("  mxt_read_num_messages() error\n"));
		T44_msg_count = 0;
	}
	else
	{
		//if(debug_output) printf("%u\n",T44_msg_count);
	}
	
	if(T44_msg_count == 0) msgs_available = 0;
}

void mxt_T46_check_config()
{
	uint8_t ret;

	ret = __mxt_read_reg(T44_addr, 1, &T44_msg_count);
	printf_P(PSTR("num_messages: "));
	if(ret)
	{
		printf_P(PSTR("error\n"));
		T44_msg_count = 0;
	}
	else
	{
		printf("%u\n",T44_msg_count);
	}
}


void mxt_write_T6_report_all()
{
	uint8_t ret;
	uint8_t val = 1;
	if( ret = i2c_send(T6_addr+3, &val, 1) != 0)
	{
		printf_P(PSTR("i2c_send error: %u\n"), ret);
		return ret;
	}
}

void mxt_read_T5_messages()
{
	uint8_t ret;
	uint8_t crc;

	mxt_read_num_messages();

	if(T44_msg_count == 0) 
	{
		printf_P(PSTR("No Messages.\n"));
		return;
	}


	printf_P(PSTR("T5_address: %u\n"), T5_addr);

	if( ret = i2c_recv(T5_addr, &msg_buf, (T5_size+1) * T44_msg_count ) != 0)  //+0x8000 
	{
		printf_P(PSTR("i2c_recv error: %u\n"), ret);
		return;
	}

	for(uint8_t msg_count=0; msg_count < T44_msg_count; msg_count++)
	{
		uint8_t msg_index = msg_count * (T5_size+1);
		printf_P(PSTR("msg%u: "), msg_count);
		for(uint8_t i = 0; i <= T5_size; i++)
		{
			printf_P(PSTR(" %u"),msg_buf[msg_index+i]);
		}
		printf("\n");
	}

	printf_P(PSTR("T5_address_checksum: %x\n"), T5_addr | 0x8000);

	if( ret = i2c_recv_checksum(T5_addr, &msg_buf, (T5_size+1) * T44_msg_count ) != 0)  //+0x8000 
	{
		printf_P(PSTR("i2c_recv error: %u\n"), ret);
		return;
	}

	for(uint8_t msg_count=0; msg_count < T44_msg_count; msg_count++)
	{
		uint8_t msg_index = msg_count * (T5_size+1);
		printf_P(PSTR("msg%u: "), msg_count);
		for(uint8_t i = 0; i <= T5_size; i++)
		{
			printf_P(PSTR(" %u"),msg_buf[msg_index+i]);
		}
		printf("\n");
	}

		crc=0;
		printf_P(PSTR("msg:"));
		for(uint8_t i = 0; i <= T5_size; i++)
		{
			if(i<T5_size) crc=crc8(crc,msg_buf[i]);
			printf_P(PSTR(" %u"),msg_buf[i]);
		}
		printf_P(PSTR("   crc8: %u \n"),crc);

}


uint8_t mxg_begin(uint16_t addr)
{
       int stat;
 
        TWI_Start();                                            // First start condition 
        stat = TWI_GetStatus();
     if (stat != 0x08) return stat;
 
       TWI_Write((MXT_APP_LOW<<1));            // Chip address + write
        stat = TWI_GetStatus();
    if (stat != 0x18) return stat;

       TWI_Write((addr & 0x00FF));                     // Address low byte
       stat = TWI_GetStatus();
    if (stat != 0x28) return stat;

       TWI_Write(addr>>8 & 0x00FF);            // Address high byte
       stat = TWI_GetStatus();
    if (stat != 0x28) return stat;
    
    TWI_Stop();
     
     return 0;
 }


// T44_msg_count byte + report_id byte + 9 byte msg_content
#define MSG_SIZE 11

void proc_msgs()
{
	uint8_t ret;
	uint8_t crc;
	uint8_t msgdata[MSG_SIZE];
	
	if(T100_report_id_min == 0) return; //return if not initialized;
	
	//mxt_read_num_messages();
	
	/*
	ret = mxg_begin(T44_addr);
	if(ret) { 
		if(debug_output) printf_P(PSTR("mxg_begin error: 0x%02X\n"), ret); 
		return; 
	} 
	*/

	do
	{			
			ret = i2c_read(&msgdata,MSG_SIZE);
			if(ret) { 
				if(debug_output) printf_P(PSTR("i2c_read error: 0x%02X\n"), ret); 
				return; 
			}

			T44_msg_count = msgdata[0];
			
			crc=0;
			if(debug_output) printf_P(PSTR("msg:"));
			for(uint8_t i = 0; i < MSG_SIZE; i++)
			{
					if(i<T5_size) crc=crc8(crc,msgdata[i]);
					if(debug_output) printf_P(PSTR(" %u"),msgdata[i]);
			}
			if(debug_output) printf_P(PSTR("   crc8: %u \n"),crc);
			
			if( (msgdata[1] >= T100_report_id_min) && (msgdata[1] <= (T100_report_id_min+10)) )
					proc_T100_msg(msgdata+1); // +1 to skip count only, +2 to skip T44_msg_count and report_id
	} while(T44_msg_count > 1);
	
	msgs_available = 0;
	
	//TWI_Stop();	

}

//From atmel_mxt_ts.c
enum t100_type {
	MXT_T100_TYPE_FINGER		= 1,
	MXT_T100_TYPE_PASSIVE_STYLUS	= 2,
	MXT_T100_TYPE_ACTIVE_STYLUS	= 3,
	MXT_T100_TYPE_HOVERING_FINGER	= 4,
	MXT_T100_TYPE_GLOVE		= 5,
	MXT_T100_TYPE_LARGE_TOUCH	= 6,
};

// 9byte msg = [1] touch_status, [2] xpos_lsb, [3] xpos_msb, [4] ypos_lsb, [5] ypos_msb ,  [6:9] aux_data

enum t100_event {
	MXT_T100_EVENT_NOEVENT = 0,
	MXT_T100_EVENT_MOVE = 1,
	MXT_T100_EVENT_UNSUP = 2,
	MXT_T100_EVENT_SUP = 3,
	MXT_T100_EVENT_DOWN = 4,
	MXT_T100_EVENT_UP = 5,
	MXT_T100_EVENT_UNSUPSUP = 6,
	MXT_T100_EVENT_UNSUPUP = 7,
	MXT_T100_EVENT_DOWNSUP = 8,
	MXT_T100_EVENT_DOWNUP = 9,
	
};


void proc_T100_msg(uint8_t *msg)
{
		uint8_t touch_status;
		uint8_t detect;
		uint8_t type;
		uint8_t event;

		touch_status = msg[0+1]; // bits =  [7] detect, [6:4] type, [3:0] event
		detect = (touch_status >> 7); //first/ most significant bit is the detect bit.
		type = (touch_status >> 4) & 0b0111; // 3 bit type (bits 6-4)
		event = touch_status & 0x0F; // event is last 4 bits.
		multitouch_id = msg[0] - T100_report_id_min;  	//global var for usb report.

		if(type != MXT_T100_TYPE_FINGER) return; //Everything is a finger.

		if(event == MXT_T100_EVENT_UP) //Check UP events first because UP events don't set the detect bit.
		{
			//Don't update X/Y pos?
			touch_state = 0b1000; //[3]UnTouch [2]Touch
			touch_pressure = 0;
			report_available=1;
			return;
		}
		
		if(!detect) return;

		if(event == MXT_T100_EVENT_DOWN) {
			touch_state = 0b0110; // Bits: [0]InRange [1]TipSwitch
			touch_pressure = 1;
		}
		else if(event == MXT_T100_EVENT_MOVE) {
			xpos = 1079 -  (msg[1+1] | ((uint16_t)msg[2+1]<<8));  //X global var for usb report.
			ypos = 1919 -  (msg[3+1] | ((uint16_t)msg[4+1]<<8));  //Y global var for usb report.
			touch_state = 0b0111; // Bits: [0]InRange [1]TipSwitch
			touch_pressure = 1;
		} 
		else {
			if(debug_output) printf_P(PSTR("\n\nUNKNOWN TOUCH STATE!!!!!!!!!!!!!!!!!!!!!!!!!\n\n"));
			return;
		}
		
		//Only (Detected) && (Finger) &&  (MOVE)||(Down) events allowed this far.

        if(debug_output) printf_P(PSTR("  touch_status: 0x%02X type: %u  detect: %u  event: %u  xpos: %u  ypos: %u  id:%d\n"), touch_status, type, detect, event, xpos, ypos, multitouch_id);
		report_available = 1;
}

void mxt_list_types()
{
	uint16_t addr_lsbf; //least significant byte first.

	uint8_t xrange[2];
	int error;
	
	uint8_t reportid_start;
	uint8_t reportid_number_per_instance;
	uint8_t reportid_total;
	uint8_t instances;
	uint8_t reportid_running_total=0;
	
	_delay_ms(1000);

	mxt_init();

    // [0] = T#-object-type, [1] = config address LSB, [2] = config address MSB, [3] = Config size - 1, [4] =  Object instances - 1, [5] = number of report_ids per instance
	for(int i=7;i< (info_blk_size-7-6); i+=6)
	{
		instances = info_blk[i+4] + 1; // instances start counting from zero, so add one
		reportid_number_per_instance = info_blk[i+5];
		reportid_total = instances * reportid_number_per_instance;
		
		if(reportid_total) {
			reportid_start = reportid_running_total + 1;
			reportid_running_total += reportid_total;
		}
		
		addr_lsbf = info_blk[i+1] | ((uint16_t)info_blk[i+2] << 8);
		//addr_msbf = ( (uint16_t)info_blk[i+1]<<8) | info_blk[i+2];
		if(debug_output) printf_P(PSTR("Type: %u, addr: %u size: %u instances: %u reports: %u"), info_blk[i], addr_lsbf, info_blk[i+3], info_blk[i+4], info_blk[i+5] );
		if(debug_output) printf_P(PSTR(" reportid_start: %u  reportid_total: %u  reportid_running_total: %u\n"), reportid_start, reportid_total, reportid_running_total);

		if(info_blk[i] == 5) 
		{
			T5_addr = addr_lsbf;
			T5_size = info_blk[i+3];
			//T5_reports = info_blk[i+5];
			//T5_reports_address=i+5;
		}

		if(info_blk[i] == 6) T6_addr = addr_lsbf;

		if(info_blk[i] == 44) T44_addr = addr_lsbf;

		if(info_blk[i] == 100) 
		{
			T100_addr = addr_lsbf;
			T100_size = info_blk[i+3];
			T100_reports = info_blk[i+5];
			T100_reports_address=i+5;
			T100_report_id_min=reportid_start+2;
		}
	}

	if(debug_output) printf_P(PSTR("T100_addr: %u\n"),T100_addr);
	if(debug_output) printf_P(PSTR("T100_report_id_min: %u\n"),T100_report_id_min);

	#define MXT_T100_XRANGE		13
	if( (error = i2c_recv(T100_addr + MXT_T100_XRANGE,&xrange,2)) != 0 )
	{
		if(debug_output) MyPrintpair("i2c reg: ",xrange);
		if(debug_output) MyPrintpairln(" i2c_recv() error: ",error);
	}
	else
	{
		if(debug_output) printf_P(PSTR("xrange: %u\r\n"), (uint16_t) xrange[1]<<8 | xrange[0] );
	}
	
	mxt_read_num_messages();

}

void mxt_init()
{
	//uint8_t *info_blk;

	int status;
	uint32_t crc;

	num_objects = mxt_get_num_objects();

	info_blk_size = 7 + 6 * num_objects; // info_header_size + obj_table_entry_size * num_table_entry_objects

/*
	if((info_blk = malloc(info_blk_size)) == NULL)
	{
		MyPrintpairln("malloc(info_blk_size) error! info_blk_size=", info_blk_size);
		return;
	}
*/

	if( (status = i2c_recv(0,&info_blk,info_blk_size)) != 0 ) // Start from Register 0, Store array in info_blk, Retrieve (bytes * blk_size)
	{
		if(debug_output) MyPrintpairln("i2c_recv(info_blk) error: ",status);
		return;
	}

	//printf("Header: %u %u %u %u %u %u %u", info_blk[0], info_blk[1], info_blk[2], info_blk[3], info_blk[4], info_blk[5], info_blk[6] );
	//printf("Header: %u %u %u %u %u %u %u", 1, 2, 3, 4, 5, 6, 7);

	crc = crc24_block(0,info_blk, info_blk_size); // Calc CRC of info block, start with CRC 0
	if(debug_output) printf_P(PSTR("calculated info block crc: %lx \n"), crc);

	if (crc != mxt_get_info_checksum(info_blk_size) ) // checksum is the 3 bytes after info block
	{
		if(debug_output) printf_P(PSTR("Invalid info block checksum."));
		return;
	}
}

/*
uint32_t crc24(uint32_t crc, uint8_t firstbyte, uint8_t secondbyte)
{
	static const uint32_t crcpoly = 0x80001b;
	uint32_t result;
	uint16_t data_word;

	data_word = (uint16_t)((uint16_t)(secondbyte << 8u) | firstbyte);
	result = ((crc<<1u) ^ (uint32_t)data_word);

	if(result & 0x1000000)  // If bit 25 is set
	{
		result ^= crcpoly;// XOR result with crcpoly
	}

	return result;
}
*/

/*
void mxt_check_crc(uint8_t *mem, uint8_t crc_area_size)
{
	uint32_t crc = 0;                // Calculated CRC 
	//uint16_t crc_area_size;          // Size of data for CRC calculation 
	//uint8_t *mem;                    // Data buffer 
	uint8_t i;
	uint8_t status;


	// Call the CRC function crc24() iteratively to calculate the CRC,
	// passing it two bytes at a time. 

	i = 0;
	while (i < (crc_area_size - 1))
	{
		printf("crc%i:0x%lx %u %u \n", i, crc, *(mem + i), *(mem + i + 1) );
		crc = crc24(crc, *(mem + i), *(mem + i + 1));
		i += 2;
	}

	// Call the crc24() with the final byte, 
	// plus an extra 0 value byte to make the sequence even. 
	crc = crc24(crc, *(mem + i), 0);
	// Final result 
	crc = (crc & 0x00FFFFFF);	// <- mask the 32-bit result to 24-bit

	printf_P(PSTR("crc: %lx\r\n"), crc);
}

*/




void mxt_word(uint16_t reg)
{
	int error;
	uint8_t val[2];
	uint16_t word;

	if( (error = i2c_recv_checksum(reg,&val,2)) != 0 )
	{
		MyPrintpair("i2c reg: ",reg);
		MyPrintpairln(" i2c_recv() error: ",error);
	}
	else
	{
		word = val[0] | (uint16_t) val[1] << 8;
		printf_P(PSTR("i2c reg: %u val[1]: 0x%x val[0]: 0x%x wordhex: 0x%x word-dec: %u \r\n"), reg, val[1], val[0], word, word);
	}
}

void mxt_read(uint16_t reg)
{
	int error;
	uint8_t val;
	if( (error = i2c_recv(reg,&val,1)) != 0 )
	{
		MyPrintpair("i2c reg: ",reg);
		MyPrintpairln(" i2c_recv() error: ",error);
	}
	else
	{
		//MyPrintpair("i2c reg: ",reg);
		//MyPrintpairln(" val: ",val);
		printf_P(PSTR("i2c reg: %u val: %u 0x%x \r\n"), reg, val, val);

	}
}

uint8_t crc81(unsigned char crc, unsigned char data)
{
	static const uint8_t crcpoly = 0x8c;
	uint8_t fb;

	for(uint8_t i=0;i<8;i++) {
		fb = (crc ^ data) & 0x01;
		data >>= 1;
		crc >>= 1;
		if (fb) crc ^= crcpoly;
	}

	return crc;
}

uint8_t crc8(unsigned char crc, unsigned char data)
{
static const uint8_t crcpoly = 0x8c;
uint8_t index;
uint8_t fb;
index = 8;
do
{
fb = (crc ^ data) & 0x01;
data >>= 1;
crc >>= 1;
if (fb)
crc ^= crcpoly;
} while (--index);
return crc;
}

void mxt_write(uint16_t reg,uint8_t val)
{
	int error;
	uint8_t vals[2];
	uint8_t crc;
	
	crc=crc8(0, (uint8_t)(reg & 0x00FF) );
	crc=crc8(crc, (uint8_t)((reg>>8)|0x80) );
	crc=crc8(crc,val);

        vals[0]=val;
	vals[1]=crc;
	

	if( (error = i2c_send(reg | 0x8000 ,vals,2)) != 0 )
	{
		MyPrintpair("i2c reg: ",reg | 0x8000);
		MyPrintpairln(" i2c_recv() error: ",error);
	}
	else
	{
		//MyPrintpair("i2c reg: ",reg | 0x8000);
		//MyPrintpair(" write: ",val);
		//MyPrintpairln(" crc8: ",vals[1]);
		printf_P(PSTR("i2c reg: %u val: %u crc8: %u \n"), reg, vals[0], vals[1] );
		//printf_P(PSTR("i2c reg: %u val: %u 0x%x \r\n"), reg, val, val);

	}
}

void mxt_write_checksum(uint16_t reg,uint8_t val)
{
	int error;

	if( (error = i2c_send(reg,val,1)) != 0 )
	{
		MyPrintpair("i2c reg: ",reg | 0x8000);
		MyPrintpairln(" i2c_recv() error: ",error);
	}
	else
	{
		printf_P(PSTR("i2c reg: 0x%04x  val: 0x%02x \n"), reg, val );
	}
}

void mxt_write_checksumOLD(uint16_t reg,uint8_t val)
{
	int error;
	uint8_t vals[2];
	uint8_t crc;

	reg = reg | 0x8000; //checksum mode - the address most significant bit set to '1'
	
	crc=crc8(0, (uint8_t)(reg & 0x00FF) );
	crc=crc8(crc, (uint8_t)(reg>>8));
	crc=crc8(crc,val);

        vals[0]=val;
	vals[1]=crc;

	printf_P(PSTR("i2c reg_lsb: 0x%02x reg_msb: 0x%02x val: 0x%02x  crc8: 0x%02x \n"), (uint8_t)(reg & 0x00FF),(uint8_t)(reg>>8), vals[0], vals[1] );

	if( (error = i2c_send(reg,vals,2)) != 0 )
	{
		MyPrintpair("i2c reg: ",reg);
		MyPrintpairln(" i2c_recv() error: ",error);
	}
}


void mxt_read_checksum(uint16_t reg)
{
	int error;
	uint8_t vals[2];
	uint8_t crc;
	
	uint8_t readdata[2];
	uint8_t grr;

	reg = reg | 0x8000; //checksum mode - the address most significant bit set to '1'
	
	crc=crc8(0, (uint8_t)(reg & 0x00FF) );
	crc=crc8(crc, (uint8_t)(reg>>8));

	//printf_P(PSTR("i2c reg_lsb: 0x%02x reg_msb: 0x%02x crc8: 0x%02x \n"), (uint8_t)(reg & 0x00FF),(uint8_t)(reg>>8), crc );

	if( (error = i2c_recv(reg,&grr,1)) != 0 )
	{
		MyPrintpair("mxt_read_checksum: i2c reg: ",reg);
		MyPrintpairln("mxt_read_checksum: i2c_recv() error: ",error);
	} 
	else
	{
		//printf_P(PSTR("readdata: %u\n"), (uint16_t)(readdata[1]<<8) | (readdata[0] & 0x00FF) );
		printf_P(PSTR("grr: %u\n"), grr );		
	}
}


/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Digitizer_HID_Interface);
	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

	USB_Device_EnableSOFEvents();

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
	HID_Device_ProcessControlRequest(&Digitizer_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Digitizer_HID_Interface);
}

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean true to force the sending of the report, false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
	USB_DigitizerReport_Data_t* DigitizerReport = (USB_DigitizerReport_Data_t*)ReportData;

    if (report_available) 
    {
		//if(debug_output) printf("usb hid digizer reported!\n");
		report_available=0;
		
        DigitizerReport->X = xpos; 
        DigitizerReport->Y = ypos; 

		//DigitizerReport->Tip_and_InRange       = (status & MXT_RELEASE) ? 0x00 : 0x03; 
		DigitizerReport->Tip_and_InRange       = touch_state; //0x03; //0x00 down 0x03 up?
		DigitizerReport->Contact_identifier    = multitouch_id;
		DigitizerReport->Contact_count_max     = 2;	
		//DigitizerReport->Pressure              = touch_pressure;	

        *ReportSize = sizeof(USB_DigitizerReport_Data_t);
    } else {
		*ReportSize = 0;
	}

    return true;
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
	// Unused (but mandatory for the HID class driver) in this demo, since there are no Host->Device reports
}



