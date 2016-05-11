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

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	stdout=&mystdout;
	stdin=&mystdin;

	SetupHardware();

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	GlobalInterruptEnable();

	RingBuffer_InitBuffer(&FromHost_Buffer, FromHost_Buffer_Data, sizeof(FromHost_Buffer_Data));


	for (;;)
	{
		HandleSerial();
		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		HID_Device_USBTask(&Digitizer_HID_Interface);
		USB_USBTask();
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
	LEDs_Init();
	USB_Init();

	TWI_Init();
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


	else if(strncmp(cmd, "crc ", 4) == 0)
	{
		//value = atoi(cmd+4);
		//MyPrintpairln("value: ",value);
		printf_P(PSTR("value: %u 0x%x crc8: %u 0x%x \n"),value,value,crc8(0,value),crc81(0,value) );
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
		mxt_get_T5_reports();
		mxt_get_T100_reports();
		mxt_read_num_messages();
	}
	else if(strcmp_P(cmd,PSTR( "msg")) == 0)
	{
		mxt_read_T5_messages();
	}
	else if(strcmp_P(cmd,PSTR( "proc")) == 0)
	{
		proc_msgs();
	}
	else if(strcmp_P(cmd,PSTR( "rall")) == 0)
	{
		mxt_write_T6_report_all();
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
		printf_P(PSTR("__mxt_read_reg(%04x, %04x, %04x) errorcode: %04x\n"), reg, len, (unsigned int) val, ret);

	return ret;
}

int __mxt_write_reg(uint16_t reg, uint16_t len, const void *val)
{
	int ret;

	ret = i2c_send(reg, val, len);
	if (ret)
		printf_P(PSTR("__mxt_write_reg() failed: %04x\n"), ret);

	return ret;
}



uint8_t mxt_get_num_objects()
{
	int ret;
	uint8_t val;

	ret = __mxt_read_reg(6,1,&val);  //Address 6
	printf_P(PSTR("num objects: "));
	if(ret){
		printf_P(PSTR("error\n"));
		return 0;
	}
	else
	{
		printf_P(PSTR("%u\n"),val);
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
		printf_P(PSTR("mxt_get_info_checksum(): i2c_recv error: %u"),ret);
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
		printf_P(PSTR("mxt_header(): i2c_recv error: %u"),ret);
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
uint8_t T5_reports_address;

uint16_t T6_addr;

uint16_t T44_addr;
uint8_t T44_msg_count;

uint16_t T100_addr;
uint8_t T100_size;
uint8_t T100_reports;
uint8_t T100_reports_address;


void mxt_read_num_messages()
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

#define MSG_SIZE 11
void proc_msgs()
{
	uint8_t ret;
	uint8_t crc;
	uint8_t msgdata[MSG_SIZE];
	
	mxt_read_num_messages();

	while(T44_msg_count)
	{
			ret = i2c_start(T44_addr);
			if(ret) { printf_P(PSTR("i2c_recv error: %u\n"), ret); return; }
			ret = i2c_read(&msgdata,MSG_SIZE);
			if(ret) { printf_P(PSTR("i2c_recv error: %u\n"), ret); return; }

			T44_msg_count = msgdata[0];
			
			crc=0;
			printf_P(PSTR("msg:"));
			for(uint8_t i = 0; i < MSG_SIZE; i++)
			{
					if(i<T5_size) crc=crc8(crc,msgdata[i]);
					printf_P(PSTR(" %u"),msgdata[i]);
			}
			printf_P(PSTR("   crc8: %u \n"),crc);
	}
	
	TWI_Stop();	

}

void mxt_get_T5_reports()
{
	uint8_t ret;
	if( ret = i2c_recv(T5_reports_address, &T5_reports, 1) != 0)
	{
		printf_P(PSTR("i2c_recv error: %u\n"), ret);
	}
	printf_P(PSTR("T5 Reports: %u\n"), T5_reports);
}

void mxt_get_T100_reports()
{
	uint8_t ret;
	if( ret = i2c_recv(T100_reports_address, &T5_reports, 1) != 0)
	{
		printf_P(PSTR("i2c_recv error: %u\n"), ret);
	}
	printf_P(PSTR("T100 Reports: %u\n"), T5_reports);
}


void mxt_list_types()
{
	uint16_t addr_lsbf; //least significant byte first.

	uint8_t xrange[2];
	int error;

	mxt_init();

	for(int i=7;i< (info_blk_size-7-6); i+=6)
	{
		addr_lsbf = info_blk[i+1] | ((uint16_t)info_blk[i+2] << 8);
		//addr_msbf = ( (uint16_t)info_blk[i+1]<<8) | info_blk[i+2];
		printf_P(PSTR("Type: %u, addr: %u size: %u instances: %u reports: %u\n"), info_blk[i], addr_lsbf, info_blk[i+3], info_blk[i+4], info_blk[i+5] );
		if(info_blk[i] == 5) 
		{
			T5_addr = addr_lsbf;
			T5_size = info_blk[i+3];
			T5_reports = info_blk[i+5];
			T5_reports_address=i+5;
		}

		if(info_blk[i] == 6) T6_addr = addr_lsbf;

		if(info_blk[i] == 44) T44_addr = addr_lsbf;

		if(info_blk[i] == 100) 
		{
			T100_addr = addr_lsbf;
			T100_size = info_blk[i+3];
			T100_reports = info_blk[i+5];
			T100_reports_address=i+5;
		}
	}

	printf_P(PSTR("T100_addr: %u\n"),T100_addr);

	#define MXT_T100_XRANGE		13
	if( (error = i2c_recv(T100_addr + MXT_T100_XRANGE,&xrange,2)) != 0 )
	{
		MyPrintpair("i2c reg: ",xrange);
		MyPrintpairln(" i2c_recv() error: ",error);
	}
	else
	{
		printf_P(PSTR("xrange: %u\r\n"), (uint16_t) xrange[1]<<8 | xrange[0] );
	}

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
		MyPrintpairln("i2c_recv(info_blk) error: ",status);
		return;
	}

	//printf("Header: %u %u %u %u %u %u %u", info_blk[0], info_blk[1], info_blk[2], info_blk[3], info_blk[4], info_blk[5], info_blk[6] );
	//printf("Header: %u %u %u %u %u %u %u", 1, 2, 3, 4, 5, 6, 7);

	crc = crc24_block(0,info_blk, info_blk_size); // Calc CRC of info block, start with CRC 0
	printf_P(PSTR("calculated info block crc: %lx \n"), crc);

	if (crc != mxt_get_info_checksum(info_blk_size) ) // checksum is the 3 bytes after info block
	{
		printf_P(PSTR("Invalid info block checksum."));
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

    if (1) 
    {
        DigitizerReport->X = 100; 
        DigitizerReport->Y = 150; 
        //DigitizerReport->Finger =0x03; 
        //DigitizerReport->Temp= 0x00; 
    } 

    *ReportSize = sizeof(USB_DigitizerReport_Data_t);
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



