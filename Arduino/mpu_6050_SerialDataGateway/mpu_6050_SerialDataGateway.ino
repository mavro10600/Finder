#include <Messenger.h>
#include <limits.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <i2cmaster.h>
#include "MLX90620_registers.h"


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13

#define USING_THERMAL_SENSOR false
#define USING_IMU true 
//#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_QUATERNION

#if !USING_IMU
  #undef OUTPUT_READABLE_YAWPITCHROLL
  #undef OUTPUT_READABLE_QUATERNION
#endif


unsigned long LastUpdateMicrosecs=0;
unsigned long LastUpdateMillisecs=0;
unsigned long CurrentMicrosecs=0;
unsigned long MicrosecsSinceLastUpdate=0;
float SecondsSinceLastUpdate=0;
unsigned long elapsedTimeMicros = 0;

Messenger messengerHandler = Messenger();

#if USING_THERMAL_SENSOR
int refreshRate = 16; //Set this value to your desired refresh frequency

int conta=0;
//Global variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
int irData[64]; //Contains the raw IR data from the sensor
float temperatures[64]; //Contains the calculated temperatures of each pixel in the array
float Tambient; //Tracks the changing ambient temperature of the sensor
byte eepromData[256]; //Contains the full EEPROM reading from the MLX (Slave 0x50)

//These are constants calculated from the calibration data stored in EEPROM
//See varInitialize and section 7.3 for more information
int v_th, a_cp, b_cp, tgc, b_i_scale;
float k_t1, k_t2, emissivity;
int a_ij[64], b_ij[64];

//These values are calculated using equation 7.3.3.2
//They are constants and can be calculated using the MLX90620_alphaCalculator sketch
float alpha_ij[64] = {
  1.75315E-8, 1.85210E-8, 1.86956E-8, 1.69494E-8, 1.95105E-8, 2.14314E-8, 2.12568E-8, 1.91031E-8, 
  2.14314E-8, 2.37597E-8, 2.37597E-8, 2.16060E-8, 2.33522E-8, 2.55059E-8, 2.59134E-8, 2.39343E-8, 
  2.51567E-8, 2.72521E-8, 2.74850E-8, 2.49238E-8, 2.60880E-8, 2.86491E-8, 2.82417E-8, 2.63208E-8, 
  2.66701E-8, 2.90566E-8, 2.92312E-8, 2.70775E-8, 2.64954E-8, 2.98133E-8, 2.98133E-8, 2.72521E-8, 
  2.72521E-8, 3.02207E-8, 2.98133E-8, 2.74850E-8, 2.69029E-8, 2.98133E-8, 2.99879E-8, 2.72521E-8, 
  2.60880E-8, 2.94058E-8, 2.94058E-8, 2.78342E-8, 2.63208E-8, 2.86491E-8, 2.86491E-8, 2.72521E-8, 
  2.51567E-8, 2.80671E-8, 2.78342E-8, 2.63208E-8, 2.33522E-8, 2.69029E-8, 2.66701E-8, 2.57387E-8, 
  2.24209E-8, 2.53313E-8, 2.55059E-8, 2.35851E-8, 2.04418E-8, 2.33522E-8, 2.33522E-8, 2.21881E-8,
};

byte loopCount = 0; //Used in main loop
void setupThermalSensor(){
  i2c_init(); //Init the I2C pins
  //PORTC = (1 << PORTC4) | (1 << PORTC5); //Enable pull-ups
  delay(5); //Init procedure calls for a 5ms delay after power-on
  read_EEPROM_MLX90620(); //Read the entire EEPROM
  setConfiguration(refreshRate); //Configure the MLX sensor with the user's choice of refresh rate
  calculate_TA(); //Calculate the current Tambient
}
void readThermalSensor(){
  if(loopCount++ == 16) //Tambient changes more slowly than the pixel readings. Update TA only every 16 loops.
  { 
    calculate_TA(); //Calculate the new Tambient
    if(checkConfig_MLX90620()){ //Every 16 readings check that the POR flag is not set
      setConfiguration(refreshRate); //Re-write the configuration bytes to the MLX
    }
    loopCount = 0; //Reset count
  }

  readIR_MLX90620(); //Get the 64 bytes of raw pixel data into the irData array
  calculate_TO(); //Run all the large calculations to get the temperature data for each pixel
  conta++;
  
  if(conta>20){
    //prettyPrintTemperatures(); //Print the array in a 4 x 16 pattern
    conta=0;
  }
  
  }



#endif



#if USING_IMU
MPU6050 mpu(0x68);
// MPU control/status vars
bool blinkState = false, dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
// ================================================================
// ===           IMU INTERRUPT DETECTION ROUTINE                ===
// ================================================================
void dmpDataReady() {
    mpuInterrupt = true;
}
void setupIMU(){
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();
    delay(1000);
    /*
    Your offsets:  -3301 -385  693 148 -33 -19
    */
    mpu.setXAccelOffset(-3301);
    mpu.setYAccelOffset(-385);
    mpu.setZAccelOffset(693);
    mpu.setXGyroOffset(148);
    mpu.setYGyroOffset(-33);
    mpu.setZGyroOffset(-19);
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);
      // enable Arduino interrupt detection
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      //attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      dmpReady = true;
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
      
    }
}

void readIMU(){
  // reset interrupt flag and get INT_STATUS bytes
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else 
  if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize){ 
      fifoCount = mpu.getFIFOCount();

    }
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // update Euler angles in degrees
      
      mpu.dmpGetQuaternion(&q, fifoBuffer);

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        // update real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      #endif
      
      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState); 
    } 
}
#endif

// ================================================================
// ===               Serial Reading Function                    ===
// ================================================================
void readFromSerial(){
   while(Serial.available() > 0){
       int data = Serial.read();
       messengerHandler.process(data);
       messengerHandler.readLong();

    }  
}

void Reset(){
  /*
  digitalWrite(GREEN_LED,HIGH);
  delay(1000);
  digitalWrite(RESET_PIN,LOW);
  digitalWrite(GREEN_LED,LOW);
  */
  ;
}

// ================================================================
// ===               TIME UPDATE FUNCTION                       ===
// ================================================================
void updateTime(){
  CurrentMicrosecs = micros();
  LastUpdateMillisecs = millis();
  MicrosecsSinceLastUpdate = CurrentMicrosecs - LastUpdateMicrosecs;
  if (MicrosecsSinceLastUpdate < 0)
    {
    MicrosecsSinceLastUpdate = INT_MIN - LastUpdateMicrosecs + CurrentMicrosecs;
    }
  LastUpdateMicrosecs = CurrentMicrosecs;
  SecondsSinceLastUpdate = MicrosecsSinceLastUpdate / 1000000.0;

  Serial.print("t");
  Serial.print("\t");
  Serial.print(LastUpdateMicrosecs);
  Serial.print("\t");
  Serial.print(SecondsSinceLastUpdate);
  Serial.print("\n");
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//OnMssg Complete function definition
void onMessageCompleted(){ 
  char reset[] = "r";
  char set_speed[] = "s";
  if(messengerHandler.checkString(reset)){
     Serial.println("Reset Done"); 
     Reset();
  }
  if(messengerHandler.checkString(set_speed)){
     //This will set the servos angle
  }
}
void updateData(){
    Serial.print('e');
    Serial.print("\t");
  #ifdef OUTPUT_READABLE_YAWPITCHROLL
    Serial.print(ypr[2]);//roll
    Serial.print("\t");
    Serial.print(ypr[1]);//pitch
    Serial.print("\t");
    Serial.print(ypr[0]);//yaw

  #endif
  #ifdef OUTPUT_READABLE_QUATERNION
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.print(q.z);
    Serial.print("\t");
    Serial.print(q.w);

  #endif
  #if !USING_IMU
    Serial.print(0.0);//x
    Serial.print("\t");
    Serial.print(0.0);//y
    Serial.print("\t");//
    Serial.print(0.0);//z
    Serial.print("\t");
    Serial.print(1.0);//w
  #endif
  
  Serial.print("\n");
}


void setup() {
  Serial.begin(115200);//cuando se ve en el ide de arduino
  pinMode(LED_PIN,OUTPUT);
  messengerHandler.attach(onMessageCompleted);
 
  #if USING_IMU
    setupIMU();
  #endif
  #if USING_THERMAL_SENSOR
    setupThermalSensor();
  #endif
  
}
void loop() {
  readFromSerial();
  updateTime(); 
  #if USING_IMU
    if( !(!mpuInterrupt && fifoCount < packetSize) )//imu ready-to-read condition
      readIMU();
  #endif
  #if USING_THERMAL_SENSOR
    readThermalSensor();
  #endif
  updateData();//send data to computer  

}

#if USING_THERMAL_SENSOR
//From the 256 bytes of EEPROM data, initialize 
void varInitialization(byte calibration_data[])
{
  v_th = 256 * calibration_data[VTH_H] + calibration_data[VTH_L];
  k_t1 = (256 * calibration_data[KT1_H] + calibration_data[KT1_L]) / 1024.0; //2^10 = 1024
  k_t2 = (256 * calibration_data[KT2_H] + calibration_data[KT2_L]) / 1048576.0; //2^20 = 1,048,576
  emissivity = ((unsigned int)256 * calibration_data[CAL_EMIS_H] + calibration_data[CAL_EMIS_L]) / 32768.0;
  
  a_cp = calibration_data[CAL_ACP];
  if(a_cp > 127) a_cp -= 256; //These values are stored as 2's compliment. This coverts it if necessary.

  b_cp = calibration_data[CAL_BCP];
  if(b_cp > 127) b_cp -= 256;

  tgc = calibration_data[CAL_TGC];
  if(tgc > 127) tgc -= 256;

  b_i_scale = calibration_data[CAL_BI_SCALE];

  for(int i = 0 ; i < 64 ; i++)
  {
    //Read the individual pixel offsets
    a_ij[i] = calibration_data[i]; 
    if(a_ij[i] > 127) a_ij[i] -= 256; //These values are stored as 2's compliment. This coverts it if necessary.

    //Read the individual pixel offset slope coefficients
    b_ij[i] = calibration_data[0x40 + i]; //Bi(i,j) begins 64 bytes into EEPROM at 0x40
    if(b_ij[i] > 127) b_ij[i] -= 256;
  }
  
}

//Receives the refresh rate for sensor scanning
//Sets the two byte configuration registers
//This function overwrites what is currently in the configuration registers
//The MLX doesn't seem to mind this (flags are read only)
void setConfiguration(int irRefreshRateHZ)
{
  byte Hz_LSB;

  switch(irRefreshRateHZ)
  {
  case 0:
    Hz_LSB = 0b00001111;
    break;
  case 1:
    Hz_LSB = 0b00001110;
    break;
  case 2:
    Hz_LSB = 0b00001101;
    break;
  case 4:
    Hz_LSB = 0b00001100;
    break;
  case 8:
    Hz_LSB = 0b00001011;
    break;
  case 16:
    Hz_LSB = 0b00001010;
    break;
  case 32:
    Hz_LSB = 0b00001001;
    break;
  default:
    Hz_LSB = 0b00001110;
  }

  byte defaultConfig_H = 0b01110100; // x111.01xx, Assumes NA = 0, ADC low reference enabled, Ta Refresh rate of 2Hz

  i2c_start_wait(MLX90620_WRITE);
  i2c_write(0x03); //Command = configuration value
  i2c_write((byte)Hz_LSB - 0x55);
  i2c_write(Hz_LSB);
  i2c_write(defaultConfig_H - 0x55); //Assumes NA = 0, ADC low reference enabled, Ta Refresh rate of 2Hz
  i2c_write(defaultConfig_H);
  i2c_stop();
}

//Read the 256 bytes from the MLX EEPROM and setup the various constants (*lots* of math)
//Note: The EEPROM on the MLX has a different I2C address from the MLX. I've never seen this before.
void read_EEPROM_MLX90620()
{
  i2c_start_wait(MLX90620_EEPROM_WRITE);
  i2c_write(0x00); //EEPROM info starts at location 0x00
  i2c_rep_start(MLX90620_EEPROM_READ);
  //Read all 256 bytes from the sensor's EEPROM
  for(int i = 0 ; i <= 255 ; i++){
    eepromData[i] = i2c_readAck();
  }
  i2c_stop(); //We're done talking

  varInitialization(eepromData); //Calculate a bunch of constants from the EEPROM data
  writeTrimmingValue(eepromData[OSC_TRIM_VALUE]);
}

//Given a 8-bit number from EEPROM (Slave address 0x50), write value to MLX sensor (Slave address 0x60)
void writeTrimmingValue(byte val)
{
  i2c_start_wait(MLX90620_WRITE); //Write to the sensor
  i2c_write(0x04); //Command = write oscillator trimming value
  i2c_write((byte)val - 0xAA);
  i2c_write(val);
  i2c_write(0x56); //Always 0x56
  i2c_write(0x00); //Always 0x00
  i2c_stop();
}

//Gets the latest PTAT (package temperature ambient) reading from the MLX
//Then calculates a new Tambient
//Many of these values (k_t1, v_th, etc) come from varInitialization and EEPROM reading
//This has been tested to match example 7.3.2
void calculate_TA(void)
{
  unsigned int ptat = readPTAT_MLX90620();

  Tambient = (-k_t1 + sqrt(square(k_t1) - (4 * k_t2 * (v_th - (float)ptat)))) / (2*k_t2) + 25; //it's much more simple now, isn't it? :)
}

//Reads the PTAT data from the MLX
//Returns an unsigned int containing the PTAT
unsigned int readPTAT_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE);
  i2c_write(CMD_READ_REGISTER); //Command = read PTAT
  i2c_write(0x90); //Start address is 0x90
  i2c_write(0x00); //Address step is 0
  i2c_write(0x01); //Number of reads is 1
  i2c_rep_start(MLX90620_READ);

  byte ptatLow = i2c_readAck(); //Grab the lower and higher PTAT bytes
  byte ptatHigh = i2c_readAck();

  i2c_stop();
  
  return( (unsigned int)(ptatHigh << 8) | ptatLow); //Combine bytes and return
}

//Calculate the temperatures seen for each pixel
//Relies on the raw irData array
//Returns an 64-int array called temperatures
void calculate_TO()
{
  float v_ir_off_comp;
  float v_ir_tgc_comp;
  float v_ir_comp;

  //Calculate the offset compensation for the one compensation pixel
  //This is a constant in the TO calculation, so calculate it here.
  int cpix = readCPIX_MLX90620(); //Go get the raw data of the compensation pixel
  float v_cp_off_comp = (float)cpix - (a_cp + (b_cp/pow(2, b_i_scale)) * (Tambient - 25)); 

  for (int i = 0 ; i < 64 ; i++)
  {
    v_ir_off_comp = irData[i] - (a_ij[i] + (float)(b_ij[i]/pow(2, b_i_scale)) * (Tambient - 25)); //#1: Calculate Offset Compensation 

    v_ir_tgc_comp = v_ir_off_comp - ( ((float)tgc/32) * v_cp_off_comp); //#2: Calculate Thermal Gradien Compensation (TGC)

    v_ir_comp = v_ir_tgc_comp / emissivity; //#3: Calculate Emissivity Compensation

    temperatures[i] = sqrt( sqrt( (v_ir_comp/alpha_ij[i]) + pow(Tambient + 273.15, 4) )) - 273.15;
  }
}

//Reads 64 bytes of pixel data from the MLX
//Loads the data into the irData array
void readIR_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE);
  i2c_write(CMD_READ_REGISTER); //Command = read a register
  i2c_write(0x00); //Start address = 0x00
  i2c_write(0x01); //Address step = 1
  i2c_write(0x40); //Number of reads is 64
  i2c_rep_start(MLX90620_READ);

  for(int i = 0 ; i < 64 ; i++)
  {
    byte pixelDataLow = i2c_readAck();
    byte pixelDataHigh = i2c_readAck();
    irData[i] = (int)(pixelDataHigh << 8) | pixelDataLow;
  }

  i2c_stop();
}

//Read the compensation pixel 16 bit data
int readCPIX_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE);
  i2c_write(CMD_READ_REGISTER); //Command = read register
  i2c_write(0x91);
  i2c_write(0x00);
  i2c_write(0x01);
  i2c_rep_start(MLX90620_READ);

  byte cpixLow = i2c_readAck(); //Grab the two bytes
  byte cpixHigh = i2c_readAck();
  i2c_stop();

  return ( (int)(cpixHigh << 8) | cpixLow);
}

//Reads the current configuration register (2 bytes) from the MLX
//Returns two bytes
unsigned int readConfig_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE); //The MLX configuration is in the MLX, not EEPROM
  i2c_write(CMD_READ_REGISTER); //Command = read configuration register
  i2c_write(0x92); //Start address
  i2c_write(0x00); //Address step of zero
  i2c_write(0x01); //Number of reads is 1

    i2c_rep_start(MLX90620_READ);

  byte configLow = i2c_readAck(); //Grab the two bytes
  byte configHigh = i2c_readAck();

  i2c_stop();

  return( (unsigned int)(configHigh << 8) | configLow); //Combine the configuration bytes and return as one unsigned int
}

//Poll the MLX for its current status
//Returns true if the POR/Brown out bit is set
boolean checkConfig_MLX90620()
{
  if ( (readConfig_MLX90620() & (unsigned int)1<<POR_TEST) == 0)
    return true;
  else
    return false;
}

//Prints the temperatures in a way that's more easily viewable in the terminal window
void prettyPrintTemperatures()
{
  Serial.println();
  for(int i = 0 ; i < 64 ; i++)
  {
    if(i % 16 == 0) Serial.println();
    Serial.print(temperatures[i]);
    //Serial.print(irData[i]);
    Serial.print(", ");
  }
}

//Prints the temperatures in a way that's more easily parsed by a Processing app
//Each line starts with '$' and ends with '*'
void rawPrintTemperatures()
{
  Serial.print("$");
  for(int i = 0 ; i < 64 ; i++)
  {
    Serial.print(temperatures[i]);
    if(i!=63){
      Serial.print(","); //Don't print comma on last temperature
    }
  }
  Serial.println("*");
}

//Given a Celsius float, converts to Fahrenheit
float convertToFahrenheit (float Tc)
{
  float Tf = (9/5) * Tc + 32;

  return(Tf);
}
#endif
  
