#ifndef MYI2C_H
#define MYI2C_H

#include <avr/io.h>
#include <stdint.h>

// Public function prototypes
void i2c_init(void);
void i2c_start(void);
void i2c_write(uint8_t data);
void i2c_stop(void);
uint8_t i2c_read_ack(void);
uint8_t i2c_read_nack(void);

#endif
