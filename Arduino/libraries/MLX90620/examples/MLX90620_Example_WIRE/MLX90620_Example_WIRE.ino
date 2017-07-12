/*
7-9-2017
Open Robotics Laboratory(Taller de Robótica Abierta)
National Autonomous University of México
César Pineda
This code is an Nathan Seidle's code adapation from  Spark Fun Electronics. Here I exchanged i2cmaster library to Wire library. 
The reason of this is because I had compilation issues trying to use Nathan's code in a Tiva C launchpad on Energia IDE.
This example shows how to read and calculate the 64 temperatures for the 64 pixels of the MLX90620 thermopile sensor.
 */

#include <Wire.h>
#include "MLX90620_registers.h"
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
  1.67684E-8, 1.85146E-8, 1.87474E-8, 1.67684E-8, 1.87474E-8, 2.04936E-8, 2.04936E-8, 1.79325E-8, 
  2.00862E-8, 2.20653E-8, 2.16578E-8, 1.93295E-8, 2.10757E-8, 2.32294E-8, 2.28220E-8, 2.04936E-8, 
  2.18324E-8, 2.43936E-8, 2.41607E-8, 2.16578E-8, 2.28220E-8, 2.49756E-8, 2.49756E-8, 2.26473E-8, 
  2.32294E-8, 2.53249E-8, 2.57323E-8, 2.34040E-8, 2.32294E-8, 2.61398E-8, 2.59070E-8, 2.38115E-8, 
  2.32294E-8, 2.59070E-8, 2.61398E-8, 2.39861E-8, 2.29966E-8, 2.57323E-8, 2.61398E-8, 2.38115E-8, 
  2.28220E-8, 2.57323E-8, 2.57323E-8, 2.38115E-8, 2.26473E-8, 2.53249E-8, 2.53249E-8, 2.34040E-8, 
  2.12503E-8, 2.43936E-8, 2.51503E-8, 2.29966E-8, 2.00862E-8, 2.28220E-8, 2.32294E-8, 2.20070E-8, 
  1.81653E-8, 2.08429E-8, 2.22399E-8, 2.04936E-8, 1.61863E-8, 1.95041E-8, 1.99116E-8, 1.85146E-8, 
};

byte loopCount = 0; //Used in main loop
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


//Begin Program code

void setup(){
	Serial.begin(115200);
	Serial.println("MLX90620 example");
	Wire.begin();
	//Wire.setModule(1);
	delay(5); //Init procedure calls for a 5ms delay after power-on
	read_EEPROM_MLX90620(); //Read the entire EEPROM
	setConfiguration(refreshRate); //Configure the MLX sensor with the user's choice of refresh rate
	calculate_TA(); //Calculate the current Tambient
}
void loop(){/*

  if(loopCount++ == 16) //Tambient changes more slowly than the pixel readings. Update TA only every 16 loops.
  { 
    calculate_TA(); //Calculate the new Tambient

    if(checkConfig_MLX90620()) //Every 16 readings check that the POR flag is not set
    {
      Serial.println("POR Detected!");
      setConfiguration(refreshRate); //Re-write the configuration bytes to the MLX
    }

    loopCount = 0; //Reset count
  }

  readIR_MLX90620(); //Get the 64 bytes of raw pixel data into the irData array

  calculate_TO(); //Run all the large calculations to get the temperature data for each pixel
  
  conta++;
  if(conta>20){
    prettyPrintTemperatures(); //Print the array in a 4 x 16 pattern
    conta=0;
  }
  //rawPrintTemperatures(); //Print the entire array so it can more easily be read by Processing app
  */
}
//From the 256 bytes of EEPROM data, initialize 
void varInitialization(byte calibration_data[]){
	v_th = 256 * calibration_data[VTH_H] + calibration_data[VTH_L];
  	k_t1 = (256 * calibration_data[KT1_H] + calibration_data[KT1_L]) / 1024.0; //2^10 = 1024
  	k_t2 = (256 * calibration_data[KT2_H] + calibration_data[KT2_L]) / 1048576.0; //2^20 = 1,048,576
  	emissivity = ((unsigned int)256 * calibration_data[CAL_EMIS_H] + calibration_data[CAL_EMIS_L]) / 32768.0;
  
  	a_cp = calibration_data[CAL_ACP];
  	if(a_cp > 127)
  		a_cp -= 256; //These values are stored as 2's compliment. This coverts it if necessary.

  	b_cp = calibration_data[CAL_BCP];
  	if(b_cp > 127) 
  		b_cp -= 256;

  	tgc = calibration_data[CAL_TGC];
  	if(tgc > 127) 
  		tgc -= 256;

  	b_i_scale = calibration_data[CAL_BI_SCALE];

  	for(int i = 0 ; i < 64 ; i++){
    	//Read the individual pixel offsets
    	a_ij[i] = calibration_data[i]; 
    	if(a_ij[i] > 127) 
    		a_ij[i] -= 256; //These values are stored as 2's compliment. This coverts it if necessary.

    	//Read the individual pixel offset slope coefficients
    	b_ij[i] = calibration_data[0x40 + i]; //Bi(i,j) begins 64 bytes into EEPROM at 0x40
    	if(b_ij[i] > 127) 
    		b_ij[i] -= 256;
  	}
  
}
void read_EEPROM_MLX90620(){
	Wire.beginTransmission(0xA0);
	Wire.write(0x00);//EEPROM info starts at location 0x00
	Wire.endTransmission();
	//Wire.beginTransmission(MLX90620_EEPROM_READ);*/
  //Wire.requestFrom(MLX90620_EEPROM_READ, 256);    // request 256 bytes from slave device #2
	Wire.requestFrom(0xA1, 256);
  //Read all 256 bytes from the sensor's EEPROM	
	/*
	for(int i = 0 ; i <= 255 ; i++){
    	eepromData[i] = Wire.read();
      Serial.print("-");
      Serial.print(eepromData[i]);
    }*/
	
	int i = 0;
  Serial.println("OK");
	while(Wire.available()){// slave may send less than requested
    eepromData[i]= Wire.read();    // receive a byte as character
    i++;
    Serial.print(eepromData[i]);         // print the character
  }

  Serial.print("\n");
  Serial.println("OK");
  //Serial.print(i); //Should be = 255
  Wire.endTransmission();
	varInitialization(eepromData); //Calculate a bunch of constants from the EEPROM data
	writeTrimmingValue(eepromData[OSC_TRIM_VALUE]);
}

//Given a 8-bit number from EEPROM (Slave address 0x50), write value to MLX sensor (Slave address 0x60)
void writeTrimmingValue(byte val){
	Wire.beginTransmission(MLX90620_WRITE);//Write to the sensor
	Wire.write(0x04);//Command = write oscillator trimming value
  	Wire.write((byte)val - 0xAA);
  	Wire.write(val);
	Wire.write(0x56); //Always 0x56
	Wire.write(0x00); //Always 0x00
  	Wire.endTransmission();
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

  Wire.beginTransmission(MLX90620_WRITE);
  Wire.write(0x03); //Command = configuration value
  Wire.write((byte)Hz_LSB - 0x55);
  Wire.write(Hz_LSB);
  Wire.write(defaultConfig_H - 0x55); //Assumes NA = 0, ADC low reference enabled, Ta Refresh rate of 2Hz
  Wire.write(defaultConfig_H);
  Wire.endTransmission();
}

//Gets the latest PTAT (package temperature ambient) reading from the MLX
//Then calculates a new Tambient
//Many of these values (k_t1, v_th, etc) come from varInitialization and EEPROM reading
//This has been tested to match example 7.3.2
void calculate_TA(void)
{
  unsigned int ptat = readPTAT_MLX90620();
  //Tambient = (-k_t1 + sqrt(square(k_t1) - (4 * k_t2 * (v_th - (float)ptat)))) / (2*k_t2) + 25; //it's much more simple now, isn't it? :)
  Tambient = (-k_t1 + sqrt(pow(k_t1,2) - (4 * k_t2 * (v_th - (float)ptat)))) / (2*k_t2) + 25; //it's much more simple now, isn't it? :)
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

//Reads the PTAT data from the MLX
//Returns an unsigned int containing the PTAT
unsigned int readPTAT_MLX90620()
{
	Wire.beginTransmission(MLX90620_WRITE);
	Wire.write(CMD_READ_REGISTER); //Command = read PTAT
  	Wire.write(0x90); //Start address is 0x90
  	Wire.write(0x00); //Address step is 0
  	Wire.write(0x01); //Number of reads is 1
  	Wire.endTransmission();
  	Wire.beginTransmission(MLX90620_READ);
  	Wire.requestFrom(MLX90620_READ,2);
  	byte ptatLow = Wire.read(); //Grab the lower and higher PTAT bytes
  	byte ptatHigh =  Wire.read();
  	//agregar while si no funciona la condición de arriba
  	Wire.endTransmission();
  	return( (unsigned int)(ptatHigh << 8) | ptatLow); //Combine bytes and return
}



//Reads 64 bytes of pixel data from the MLX
//Loads the data into the irData array
void readIR_MLX90620(){
	Wire.beginTransmission(MLX90620_WRITE);
  	Wire.write(CMD_READ_REGISTER); //Command = read a register
  	Wire.write(0x00); //Start address = 0x00
  	Wire.write(0x01); //Address step = 1
  	Wire.write(0x40); //Number of reads is 64
  	Wire.endTransmission();

  	Wire.beginTransmission(MLX90620_READ);
  	Wire.requestFrom(MLX90620_READ,64);

  	/*
  	for(int i = 0 ; i < 64 ; i++){
    	byte pixelDataLow = i2c_readAck();
    	byte pixelDataHigh = i2c_readAck();
    	irData[i] = (int)(pixelDataHigh << 8) | pixelDataLow;
  	}
  	*/

  	int i = 0;
	while(Wire.available()){// slave may send less than requested
    	byte pixelDataLow = Wire.read();
    	byte pixelDataHigh = Wire.read();
    	irData[i] = (int)(pixelDataHigh << 8) | pixelDataLow;
    	Serial.print(irData[i]);// print the character
    	i++;
  	}
  	Serial.print(i); //Should be = 63
  	Wire.endTransmission();
}

//Read the compensation pixel 16 bit data
int readCPIX_MLX90620()
{
  Wire.beginTransmission(MLX90620_WRITE);
  Wire.write(CMD_READ_REGISTER); //Command = read register
  Wire.write(0x91);
  Wire.write(0x00);
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.beginTransmission(MLX90620_READ);
  Wire.requestFrom(MLX90620_READ,2);
  byte cpixLow = Wire.read(); //Grab the two bytes
  byte cpixHigh = Wire.read();
  Wire.endTransmission();

  return ( (int)(cpixHigh << 8) | cpixLow);
}

//Reads the current configuration register (2 bytes) from the MLX
//Returns two bytes
unsigned int readConfig_MLX90620()
{
  Wire.beginTransmission(MLX90620_WRITE);//The MLX configuration is in the MLX, not EEPROM
  Wire.write(CMD_READ_REGISTER); //Command = read configuration register
  Wire.write(0x92); //Start address
  Wire.write(0x00); //Address step of zero
  Wire.write(0x01); //Number of reads is 1
  Wire.endTransmission();

  Wire.beginTransmission(MLX90620_READ);
  Wire.requestFrom(MLX90620_READ,2);

  byte configLow = Wire.read(); //Grab the two bytes
  byte configHigh = Wire.read();

  Wire.endTransmission();

  return( (unsigned int)(configHigh << 8) | configLow); //Combine the configuration bytes and return as one unsigned int
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

  for (int i = 0 ; i < 64 ; i++){
    v_ir_off_comp = irData[i] - (a_ij[i] + (float)(b_ij[i]/pow(2, b_i_scale)) * (Tambient - 25)); //#1: Calculate Offset Compensation 

    v_ir_tgc_comp = v_ir_off_comp - ( ((float)tgc/32) * v_cp_off_comp); //#2: Calculate Thermal Gradien Compensation (TGC)

    v_ir_comp = v_ir_tgc_comp / emissivity; //#3: Calculate Emissivity Compensation

    temperatures[i] = sqrt( sqrt( (v_ir_comp/alpha_ij[i]) + pow(Tambient + 273.15, 4) )) - 273.15;
  }
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