enum MPU_6050_registers{
  PWR_MGMT_1=0x6B,
  SMPLRT_DIV=0x19,
  CONFIG=0x1A,
  GYRO_CONFIG=0x1B,
  ACCEL_CONFIG=0x1C,
  INT_ENABLE=0x38,
  WHO_AM_I=0x75, //lol this is not a joke its acually one of the register names in MPU6050 "its a readonly registerrrrrrrrrrrrrrrrrr"
  GYRO_XH=0x43, //x axis high val
  GYRO_XL=0x44, //x axis low val
  GYRO_YH=0x45, //y axis high val
  GYRO_YL=0x46, //y axis low val
  GYRO_ZH=0x47, //z axis high val
  GYRO_ZL=0x48,  //z axis low val
  ACCEL_XH=0x3B, //x high
  ACCEL_XL=0x3C,  //x low
  ACCEL_YH=0x3D, //y high
  ACCEL_YL=0x3E, //y low
  ACCEL_ZH=0x3F, //z high
  ACCEL_ZL=0x40  //z low
};
/////////////////////////////////////////////////////////////////////////////////////////////////struct
struct MPU_datas{
  int16_t ax,ay,az;//accel
  int16_t gx,gy,gz;//gyro
};
//////////////////////////////////////////////////////////////////////////////////////////////////
#include "MyI2C.h"
//#include "MyUSART.h"
#include <avr/io.h>
#include <util/delay.h>
#include <math.h> 
#define mpu_addL 0x68 //when ADO pulled low 
#define mpu_addH 0x69 //when ADO pulled high
#define command_flag 0b00000000 //if RS is zero it means command 
#define read_flag    0b00000001 //else if its 1 it means we want to display data
#define clk_speed 16000000
#define baud 9600
#define my_ubrr (clk_speed/16/baud-1)
#define accel_sen 16384.0
#define gyro_sen  131.0

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

//and i looked at the datasheet the output data are 16 bit values 8 high and 8 low and now we need to get the data and cast it to a 16 bit register 
int16_t read_data(uint8_t regH,uint8_t regL){
  uint8_t high_data=mpu_read_register(regH);
  uint8_t low_data=mpu_read_register(regL);
  int16_t full_data=(int16_t)(high_data<<8)|low_data;
  return full_data;


}//this function is not that efficent in the datasheet it says u can do a burst read means in one i2c transmit u can read all 12 bytes from the mpu
///////////////////////////////////////////////////////////////////////// here is burst func
void burst_read(MPU_datas* data){
  int8_t buffer[14];
  i2c_start();
  i2c_write((mpu_addL<<1)|0);
  i2c_write(ACCEL_XH);//we gonna start from this byte
  i2c_start();//reapeted start so we can read data
  i2c_write((mpu_addL<<1)|1);
  for (int i =0; i<13; i++) {
  buffer[i]=i2c_read_ack();//so it continues sending data by burst
  }
  buffer[13]=i2c_read_nack();//not ack so it stops sending data
  i2c_stop();

  data->ax=(buffer[0]<<8)|buffer[1];//we have accel x high byte and low byte thats why we send it that way
  data->ay=(buffer[2]<<8)|buffer[3];
  data->az=(buffer[4]<<8)|buffer[5];
  data->gx=(buffer[8]<<8)|buffer[9];
  data->gy=(buffer[10]<<8)|buffer[11];
  data->gz=(buffer[12]<<8)|buffer[13];
//becarful of one thing when doing burst reading the datas of the accel, gyro and temp sensor are stored in countinues memory so even if u dont use the temp data like me 
// bc i trying to make a gimbal still u have to consider for the temp data and make the buffer big enough so u can put temp data there too

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//this is an old driver i wrote a month ago to controlling two servos by a joy stick now i have to medify it 
void set_timer(void){
  //the timer setup
  TCCR1A=(1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);//we gonna use a non-inverting PWM with 8 as the prescaler why? 
  // why the OCR1 is 39999 we want the top value as 20 ms with clk speed of 16Mhz of the 328p and 50hz pwm by this formula (TOP=(clk_SPEED)/(8*50)-1) we will get 39999
  //and u may think why 39999 is bc first we used 16bit timer of the 328p and the time for the counter to count from 0 till 39999 it takes 20ms or 20000us
  //bc the servo expects this time with 50hz 
  //1ms=0 degree so the counter needs to count till 2000 this means 1ms till 3000 is 1.5 and 4000 is 2ms

  //1.5=90
  //2=180
  //and the rest is counted as low
  TCCR1B=( 1<<WGM12 )|(1<<WGM13)|(1<<CS11);
  ICR1=39999;//the top value that we want to count till this number
  //and also the pin that sends the   PWM should be a specific pin of the MCU look at the pinoutt of the arduino u see ~ symbol beside some pins means this pin can send a PWM
  //and the pin should be PB1 why when looking at the pinout of the uno or nano u see a arrow pointing to PB1 and it says TIMER OC1A
  DDRB|=(1<<PB1)|(1<<PB2);
}
//now to an important part which is the mapping like we wanna set the mapping from 0->1023 to 2000->4000 we just said why 
uint16_t servo_map(uint16_t val){
  //from 0-1023 to 2000-4000 ticks
  //we need to cast the val why ? bc without it it becomes 2mil something and thats wayy beyond 16bit so we need to cast and add an offsett of 2000
  return 2000 + ((uint32_t)val * 2000) / 1023;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {
  i2c_init();
  mpu_init();
  //USART_init(my_ubrr);
  Serial.begin(115200);
  //now we test if we tamed the beast or not lol
  uint8_t who_am_i=mpu_read_register(WHO_AM_I);
 // if(who_am_i==mpu_addL){
   // USART_strTransmit("the beast has been tamed!!!!! :)");
  //}
  //else 
    //Serial.println("the beast ran away!!!!! :( ");
    //USART_strTransmit("the beast ran away!!!!! :( ");

}
char buffer[50];
void loop() {
  struct MPU_datas pmuData;
  burst_read(&pmuData);
  int16_t accel_x=pmuData.ax/accel_sen;
  int16_t accel_y=pmuData.ay/accel_sen;
  int16_t accel_z=pmuData.az/accel_sen;
  ////////////////////////////////////
  int16_t gyro_x=pmuData.gx/gyro_sen;
  int16_t gyro_y=pmuData.gy/gyro_sen;
  int16_t gyro_z=pmuData.gz/gyro_sen;
  sprintf(buffer,"accel_data\n\r X:%d Y:%d Z:%d",accel_x,accel_y,accel_z);
  Serial.println(buffer);
  delay(100);
  sprintf(buffer,"gyro_data\n\r X:%d Y:%d Z:%d",gyro_x,gyro_y,gyro_z);
  Serial.println(buffer);

  delay(100);
  



}
