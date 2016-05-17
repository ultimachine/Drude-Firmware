#include <avr/io.h>
#include "i2c.h"
#include <util/twi.h>

uint8_t _crc8(unsigned char crc, unsigned char data)
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

/**************************************
 * A bit higher level stuff 
 **************************************/

// Receive count bytes from addr
int i2c_recv_checksum(uint16_t addr, uint8_t *buf, int count){
	int stat;

	uint8_t crc;

	addr |= 0x8000; //checksum mode - set the address most significant bit set to '1'
	
	crc=_crc8(0, (uint8_t)(addr & 0x00FF) );
	crc=_crc8(crc, (uint8_t)(addr>>8));

	TWI_Start();						// First start condition 
	stat = TWI_GetStatus();
    	if (stat != 0x08) return stat;

	TWI_Write((MXT_APP_LOW<<1));		// Chip address + write
	stat = TWI_GetStatus();
    	if (stat != 0x18) return stat;

	TWI_Write((addr & 0x00FF));			// Address low byte
	stat = TWI_GetStatus();
        if (stat != 0x28) return stat;

	TWI_Write(addr>>8 & 0x00FF);		// Address high byte
	stat = TWI_GetStatus();
        if (stat != 0x28) return stat;

	TWI_Write(crc);		// CRC
	stat = TWI_GetStatus();
        if (stat != 0x28) return stat;

	TWI_Stop();


	TWI_Start();
	stat = TWI_GetStatus();
    	if (stat != 0x08 && stat != 0x10)
        return stat;

	TWI_Write((MXT_APP_LOW<<1) | 0x01);	// Chip address + read
	stat = TWI_GetStatus();
    	if (stat != 0x40)
        return stat;

	for(int i=0; i<count-1; i++){			
		buf[i] = TWI_Read(1);			// Read the data and ACK
	}
	buf[count-1] = TWI_Read(0);			// No ack on last byte
	TWI_Stop();							// Send stop condition 
	
	return 0;
}

// Send a message via I2C
int i2c_send_checksum(uint16_t addr, const uint8_t *buf, int count){
	int stat;
	uint8_t crc;

	addr = addr | 0x8000; //checksum mode - the address most significant bit set to '1'
	
	crc=_crc8(0, (uint8_t)(addr & 0x00FF) );
	crc=_crc8(crc, (uint8_t)(addr>>8));

	TWI_Start();						// First start condition 
	stat = TWI_GetStatus();
    	if (stat != 0x08) return stat;

	TWI_Write((MXT_APP_LOW<<1));		// Chip address + write
	stat = TWI_GetStatus();
    	if (stat != 0x18) return stat;

	TWI_Write((addr & 0x00FF));			// Address low byte
	stat = TWI_GetStatus();
   	if (stat != 0x28) return stat;

	TWI_Write(addr>>8 & 0x00FF);		// Address high byte
	stat = TWI_GetStatus();
    	if (stat != 0x28) return stat;

	for(int i=0; i<count; i++){				// write the data
		crc=crc8(crc,buf[i]);
		TWI_Write(buf[i]);
		stat = TWI_GetStatus();
		if (stat != 0x28)
		    return stat;
	}

	TWI_Write(crc);		// SEND CHECKSUM
	stat = TWI_GetStatus();
    	if (stat != 0x28) return stat;

	TWI_Stop();							// Send stop condition 

	return 0;
}

uint8_t i2c_start(uint8_t addr)
{
	uint8_t stat;

	TWI_Start();						// First start condition 
	stat = TWI_GetStatus();
    if (stat != 0x08) return stat;

	TWI_Write((addr<<1));		// Chip address + write
	stat = TWI_GetStatus();
    if (stat != 0x18)
        return stat;

    return 0;
}


int i2c_read(uint8_t *buf, int count)
{
	int stat;
	
	TWI_Start();						// First or Second start condition 	
	stat = TWI_GetStatus();
    if ((stat != 0x10) && stat != 0x08) return stat;

	TWI_Write((MXT_APP_LOW<<1) | 0x01);	// Chip address + read
	stat = TWI_GetStatus();
    if (stat != 0x40) return stat;

	for(int i=0; i<count-1; i++){			
		buf[i] = TWI_Read(1);			// Read the data and ACK
	}
	buf[count-1] = TWI_Read(0);			// No ack on last byte
	TWI_Stop();							// Send stop condition 
	
	return 0;
}

// Receive count bytes from addr
int i2c_recv(uint16_t addr, uint8_t *buf, int count){
	int stat;

	TWI_Start();						// First start condition 
	stat = TWI_GetStatus();
    if (stat != 0x08)
        return stat;

	TWI_Write((MXT_APP_LOW<<1));		// Chip address + write
	stat = TWI_GetStatus();
    if (stat != 0x18)
        return stat;

	TWI_Write((addr & 0x00FF));			// Address low byte
	stat = TWI_GetStatus();
    if (stat != 0x28)
        return stat;

	TWI_Write(addr>>8 & 0x00FF);		// Address high byte
	stat = TWI_GetStatus();
    if (stat != 0x28)
        return stat;

	TWI_Start();						// Second start condition 	
	stat = TWI_GetStatus();
    if (stat != 0x10)
        return stat;

	TWI_Write((MXT_APP_LOW<<1) | 0x01);	// Chip address + read
	stat = TWI_GetStatus();
    if (stat != 0x40)
        return stat;

	for(int i=0; i<count-1; i++){		
		buf[i] = TWI_Read(1);			// Read the data and ACK
	}
	buf[count-1] = TWI_Read(0);			// No ack on last byte
	TWI_Stop();							// Send stop condition 
	
	return 0;
}


// Send a message via I2C
int i2c_send(uint16_t addr, const uint8_t *buf, int count){
	int stat;

	TWI_Start();						// First start condition 
	stat = TWI_GetStatus();
    if (stat != 0x08)
        return stat;

	TWI_Write((MXT_APP_LOW<<1));		// Chip address + write
	stat = TWI_GetStatus();
    if (stat != 0x18)
        return stat;

	TWI_Write((addr & 0x00FF));			// Address low byte
	stat = TWI_GetStatus();
    if (stat != 0x28)
        return stat;

	TWI_Write(addr>>8 & 0x00FF);		// Address high byte
	stat = TWI_GetStatus();
    if (stat != 0x28)
        return stat;

	for(int i=0; i<count; i++){				// write the data
		TWI_Write(buf[i]);
		stat = TWI_GetStatus();
		if (stat != 0x28)
		    return stat;
	}
	TWI_Stop();							// Send stop condition 

	return 0;
}


/**************************************
 * Low level I2C stuff
 **************************************/

void TWI_Init(void){
    //set SCL to 400kHz
    TWSR = 0x00;
    //TWBR = 0x0C;
    //TWBR = ((F_CPU / frequency) - 16) / 2;
    TWBR = ((F_CPU / 100000 ) - 16) / 2;
    //enable TWI
    TWCR = (1<<TWEN);
}

// Send start signal 
void TWI_Start(void){
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

//send stop signal
void TWI_Stop(void){
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

// Write a byte
void TWI_Write(uint8_t u8data){
    TWDR = u8data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

//read byte with NACK
uint8_t TWI_Read(char ack){
    TWCR = (1<<TWINT)|(1<<TWEN)|((ack?1:0)<<TWEA);
    while ((TWCR & (1<<TWINT)) == 0);
    return TWDR;
}

// Get TWI status or use TWI_STATUS
uint8_t TWI_GetStatus(void){
    return TWSR & 0xF8;
}


