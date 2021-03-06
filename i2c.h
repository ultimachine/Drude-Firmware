
#ifndef _digi_i2c_h_
#define _digi_i2c_h_

#define MXT_APP_LOW	0x4a

uint8_t i2c_start(uint8_t addr);

int i2c_recv(uint16_t addr, uint8_t *buf, int count);
int i2c_send(uint16_t addr, const uint8_t *buf, int count);
int i2c_recv_checksum(uint16_t addr, uint8_t *buf, int count);
int i2c_send_checksum(uint16_t addr, const uint8_t *buf, int count);

void TWI_Init(void);
void TWI_Start(void);
void TWI_Stop(void);
void TWI_Write(uint8_t u8data);
uint8_t TWI_Read(char ack);
uint8_t TWI_GetStatus(void);

#endif
