#include "MyI2C.h"

void i2c_init(void){
    TWSR = 0x00;       // Prescaler = 1
    TWBR = 72;         // ~100kHz for 16MHz clock
    TWCR = (1<<TWEN);  // Enable TWI
}
uint8_t i2c_read_ack(void) {
    
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT))); // Wait for receive to finish
    return TWDR;
}


uint8_t i2c_read_nack(void) {
    
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))); // Wait for receive to finish
    return TWDR;
}
void i2c_start(void){
    TWCR = (1<<TWSTA) | (1<<TWINT) | (1<<TWEN);
    while(!(TWCR & (1<<TWINT))); // Wait for start to finish
}

void i2c_write(uint8_t data){
    TWDR = data;
    TWCR = (1<<TWINT) | (1<<TWEN);
    while(!(TWCR & (1<<TWINT))); // Wait for transmission
}

void i2c_stop(void){
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}
