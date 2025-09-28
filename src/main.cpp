#include "MyI2C.h"
#define mpu_addL 0x68 //when ADO pulled low 
#define mpu_addH 0x69 //when ADO pulled high
#define command_flag 0b00000000 //if RS is zero it means command 
#define read_flag    0b00000001 //else if its 1 it means we want to display data
#define PWR_MGMT_1 0x6B
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Bit:  7              6     5      4          3      2    1    0
//Name: DEVICE_RESET SLEEP CYCLE TEMP_DIS  Dontcare   CLKSEL[2:0]

void mpu_init(){
  // first we have to wake the mpu from sleep and then we have to mess with it later
  i2c_start();
  i2c_write(mpu_addL|command_flag);
  i2c_write(PWR_MGMT_1);
  i2c_write(0b00000001); //as u can see the bit names of the PWR_MGMT_1 register we want to put everything zero but not the first bit why ? bc we use the gyro1 bc the datasheet says its recomended
  //Datasheet recommends using PLL with one of the gyros (usually X gyro → value 1) because it’s more stable than the internal 8mHz osc
  i2c_stop();
}





void setup() {
  i2c_init();

}

void loop() {
  // put your main code here, to run repeatedly:

}
