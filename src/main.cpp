enum MPU_6050_registers{
  PWR_MGMT_1=0x6B,
  SMPLRT_DIV=0x19,
  CONFIG=0x1A,
  GYRO_CONFIG=0x1B,
  ACCEL_CONFIG=0x1C,
  INT_ENABLE=0x38,
  WHO_AM_I=0x75 //lol this is not a joke its acually one of the register names in MPU6050 "its a readonly registerrrrrrrrrrrrrrrrrr"
};
#include "MyI2C.h"
#include "MyUSART.h"
#define mpu_addL 0x68 //when ADO pulled low 
#define mpu_addH 0x69 //when ADO pulled high
#define command_flag 0b00000000 //if RS is zero it means command 
#define read_flag    0b00000001 //else if its 1 it means we want to display data
#define clk_speed 16000000
#define baud 9600
#define my_ubrr (clk_speed/16/baud-1)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PWR_MGMT_1 register mapping
//Bit:  7              6     5      4          3      2    1    0
//Name: DEVICE_RESET SLEEP CYCLE TEMP_DIS  Dontcare   CLKSEL[2:0]
////////////////////////////
//SMPLRT_DIV: 
//where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7), and 1kHz when the DLPF is enabled and by this formula you will get the smaple rate 
//Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
//and also we want the DLPF to be enabled all the time

void mpu_init(){
  // first we have to wake the mpu from sleep and then we have to mess with it later
  i2c_start();
  i2c_write((mpu_addL<<1)|0);
  i2c_write(PWR_MGMT_1);
  i2c_write(0b00000001); //as u can see the bit names of the PWR_MGMT_1 register we want to put everything zero but not the first bit why ? bc we use the gyro1 bc the datasheet says its recomended
  //Datasheet recommends using PLL with one of the gyros (usually X gyro → value 1) because it’s more stable than the internal 8mHz osc
  i2c_stop();

  i2c_start();
  i2c_write((mpu_addL<<1)|0);
  i2c_write(CONFIG);
  i2c_write(0b00000010);//as we said in above we have to enable the digital low pass filter and it depends on u but i wanted 100hz so closet option was third one in the table from the datasheet
  i2c_stop();

  i2c_start();
  i2c_write((mpu_addL<<1)|0);
  i2c_write(SMPLRT_DIV);
  i2c_write(0b00001001);//we want 100 hz why i explained it down in a comments
  i2c_stop();
  /*
      What sample rate should you want?

    It depends on your project:

    Camera gimbal / robotics (your case) → 100 Hz to 200 Hz is perfect.

    Fast enough to track motion smoothly

    Slow enough that your microcontroller can handle the math

    Common choice: 100 Hz (DIV=9)

    Quadcopters with sensor fusion (MPU + filter/IMU) → Often 200–500 Hz. (e.g. DIV=1 → 500 Hz).

    Simple tilt sensor / human motion → 50 Hz or even lower. (DIV=19 → 50 Hz).

    High-speed experiments → Up to the max (1 kHz with DLPF, or 8 kHz without).
  */
  i2c_start();
  i2c_write((mpu_addL<<1)|0);
  i2c_write(GYRO_CONFIG);
  i2c_write(0b00000000);//we dont want any selftest so we dont care about the last 3 bits and the first three too all we care is the sensitivity of the gyro and we wanna set it to ± 250 °/s
  //so we just sent a zero to that register and it should be fine
  i2c_stop();


  i2c_start();
  i2c_write((mpu_addL<<1)|0);
  i2c_write(ACCEL_CONFIG);
  i2c_write(0b00000000);//same as the gyro config register all we want is ± 2g then we send a zero to that register too
  i2c_stop();

  i2c_start();
  i2c_write((mpu_addL<<1)|0);
  i2c_write(INT_ENABLE);
  i2c_write(0b00000001);//we need only the data ready bit to be set to 1 for example the mot en bit when it senses motion it sends an int so the device wakes up but its used in power down mode so we dont want this
  //and the other ints too we dont want them we want continues data int
  i2c_stop();


  

}//done
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t mpu_read_register(uint8_t reg_addr) {
    uint8_t data;

    
    i2c_start();
    i2c_write((mpu_addL<<1)|0); 
    i2c_write(reg_addr);
    i2c_start();//repeated start 
    i2c_write((mpu_addL<<1)|1);
    data = i2c_read_nack(); 
    i2c_stop();

    return data;
}


void setup() {
  i2c_init();
  mpu_init();
  USART_init(my_ubrr);
  //now we test if we tamed the beast or not lol
  uint8_t who_am_i=mpu_read_register(WHO_AM_I);
  if(who_am_i==mpu_addL){
    USART_strTransmit("the beast has been tamed!!!!! :)");
  }
  else 
    //Serial.println("the beast ran away!!!!! :( ");
    USART_strTransmit("the beast ran away!!!!! :( ");

}

void loop() {
  // put your main code here, to run repeatedly:

}
