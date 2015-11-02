#include <avr/io.h>
#include "i2c.h"

/**************************************
 * A bit higher level stuff 
 **************************************/

#define MXT_APP_LOW	0x4a

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

	TWI_Write((addr<<8 & 0xFF00));		// Address high byte
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

	TWI_Write((addr<<8 & 0xFF00));		// Address high byte
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
    TWBR = 0x0C;
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

// Get TWI status
uint8_t TWI_GetStatus(void){
    return TWSR & 0xF8;
}


