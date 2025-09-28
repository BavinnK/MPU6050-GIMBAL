#include "MyI2C.h"
#include <Arduino.h>
#define mpu_addL 0x68 //when ADO pulled low 
#define mpu_addH 0x69 //when ADO pulled high
#define command_flag 0b00000000 //if RS is zero it means command 
#define read_flag    0b00000001 //else if its 1 it means we want to display data
void mpu_init(){
  i2c_start();
  i2c_write(mpu_addL|command_flag);
}





void setup() {
  i2c_init();

}

void loop() {
  // put your main code here, to run repeatedly:

}
