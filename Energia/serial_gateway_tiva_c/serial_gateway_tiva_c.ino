#include <Messenger.h>
#include <limits.h>
#include <DynamixelSerial.h>
#include <SimpleDynamixel.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <Servo.h> 

#define RESET_PIN PB_2
#define INTERRUPT_PIN PB_5  // use pin 2 on Arduino Uno & most boards
#define DATA_CONTROL_PIN PA_3 //Data control pin for dynamixel
 
//#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_QUATERNION

#define USING_IMU false
#define USING_CO2_SENSOR false
#define USING_DYNAMIXEL true

Messenger messengerHandler = Messenger();

//Time update variables
bool led = HIGH;
unsigned long LastUpdateMicrosecs=0;
unsigned long LastUpdateMillisecs=0;
unsigned long CurrentMicrosecs=0;
unsigned long MicrosecsSinceLastUpdate=0;
float SecondsSinceLastUpdate=0;
unsigned long elapsedTimeMicros = 0;
unsigned long lastTimeCO2read = 0;
////////////////////////////////////////////////////////////////////////////////////
// ================================================================
// ===               SETUP AND READ IMU FUNCTIONS               ===
// ================================================================
//IMU variables
float roll,pitch, yaw;

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
    Wire.setModule(3);
    //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    mpu.initialize();
    
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();
    /*
    Sensor readings with offsets:  -11 7 16388 0 1 0
    Your offsets: -3271 380 767 217 -2  27
    */
    mpu.setXAccelOffset(-3271);
    mpu.setYAccelOffset(380);
    mpu.setZAccelOffset(767);
    mpu.setXGyroOffset(217);
    mpu.setYGyroOffset(-2);
    mpu.setZGyroOffset(27);
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);
      // enable Arduino interrupt detection
      //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
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
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
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
      digitalWrite(GREEN_LED, blinkState); 
    } 
}
#endif
////////////////////////////////////////////////////////////////////////////////////
//dynamixel variables
int gripper_out = -100,
    gripper_load=0,
    gripper_pos=0,
    gripper_inc=0;

bool ledDynamixel = false;

 
#if USING_DYNAMIXEL
//Dynamixell inicialization, pines Serial3
DynamixelClass dynamixel_obj(&Serial3, DATA_CONTROL_PIN);//Cambiar a serial1 en si se usa arduino leonardo
SimpleDynamixelClass dynamixel(&dynamixel_obj, 600, 850, 100);//ancho de pulso minimo 600, ancho de pulso maximo 850, operacion de -100 a 100

void setupDynamixel(){
    dynamixel_obj.begin(1000000UL, DATA_CONTROL_PIN);
    //dynamixel_obj.setMaxTorque(1, 1023);//Se define el maximo par disponible
    //dynamixel.write(gripper_out);//Start dynamixel totally open
    delay(100);
  }

void updateDynamixel(){
  if (dynamixel_obj.ping(1) != 0) {
      dynamixel_obj.reset(1);
      delay(100);
      gripper_out=-100;
      setupDynamixel();
  }
  gripper_load = dynamixel_obj.readLoad(1);
  gripper_pos = dynamixel_obj.readPosition(1);
  if(gripper_load < 1400 || gripper_inc < 0){
    gripper_out += gripper_inc; 
    gripper_out = constrain(gripper_out,-100,100);
    dynamixel.write(gripper_out);
    ledDynamixel = !ledDynamixel;
    digitalWrite(BLUE_LED,ledDynamixel);
  }
}
#endif

////////////////////////////////////////////////////////////////////////////////////
//pan-tilt servo angles
int pan,tilt;
// ================================================================
// ===               CO2 SENSOR FUNCTIONS                       ===
// ================================================================
int co2;
#if USING_CO2_SENSOR
////////////////////////////////////////////////////////////////////////////////////
//CO2 sensor variables
double multiplier = 4;// 1 for 2% =20000 PPM, 10 for 20% = 200,000 PPM
uint8_t buffer[25];
uint8_t ind =0;
uint8_t index_ =0;
bool blinkCO2 = false;

void setupCO2Sensor(){
    ///////////CO2 set-up
    Serial2.begin(9600);
    //Serial2.setTimeout(10);
    Serial2.println("K 0");  // Set Command mode
    Serial2.println("M 6"); // send Mode for Z and z outputs
    // "Z xxxxx z xxxxx" (CO2 filtered and unfiltered)
    Serial2.println("K 1");  // set streaming mode
    pinMode(RED_LED,OUTPUT);
}
void fill_buffer(void){
  // Fill buffer with sensor ascii data
  ind = 0;
  while(buffer[ind-1] != 0x0A&& Serial2.available() ){  // Read sensor and fill buffer up to 0XA = CR
    if(Serial2.available()) {
      buffer[ind] = Serial2.read();
      ind++;
      blinkCO2 = !blinkCO2;
      digitalWrite(RED_LED,blinkCO2);
    }
  }
  // buffer() now filled with sensor ascii data
  // ind contains the number of characters loaded into buffer up to 0xA =  CR
  ind = ind -2; // decrement buffer to exactly match last numerical character
}

void format_output(){ // read buffer, extract 6 ASCII chars, convert to PPM and print
  co2 = buffer[15-index_]-0x30;
  co2 = co2+((buffer[14-index_]-0x30)*10);
  co2 +=(buffer[13-index_]-0x30)*100;
  co2 +=(buffer[12-index_]-0x30)*1000;
  co2 +=(buffer[11-index_]-0x30)*10000;
}
void readCO2sensor(){
  fill_buffer();  // function call that reads CO2 sensor and fills buffer
  index_ = 8;  // In ASCII buffer, filtered value is offset from raw by 8 bytes
  format_output();
  co2 = co2*multiplier;
}
#endif


// ================================================================
// ===               Serial Reading Function                    ===
// ================================================================
void readFromSerial(){
   while(Serial.available() > 0){
       int data = Serial.read();
       messengerHandler.process(data);
       //led=!led;
       //digitalWrite(GREEN_LED,led);
       gripper_inc = messengerHandler.readLong();
    }  
}

void Reset(){
  digitalWrite(GREEN_LED,HIGH);
  delay(1000);
  digitalWrite(RESET_PIN,LOW);
  digitalWrite(GREEN_LED,LOW);
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
    Serial.print("\t");
  #endif
  
  #ifdef OUTPUT_READABLE_QUATERNION
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.print(q.z);
    Serial.print("\t");
    Serial.print(q.w);
    Serial.print("\t");
  #endif
      Serial.print(0.0);
    Serial.print("\t");
    Serial.print(0.0);
    Serial.print("\t");
    Serial.print(0.0);
    Serial.print("\t");
    Serial.print(1.0);
    Serial.print("\t");
  
  Serial.print(co2);
  Serial.print("\t");
  Serial.print(gripper_pos);
  Serial.print("\n");
}

void setup() {
  Serial.begin(115200);//cuando se ve en el ide de arduino
  pinMode(GREEN_LED,OUTPUT);
  pinMode(RESET_PIN,OUTPUT);
  pinMode(BLUE_LED,OUTPUT);
  
  ///Conectar el pin de reset al pin reset de la placa si se usa arduino
  digitalWrite(RESET_PIN,HIGH);
  messengerHandler.attach(onMessageCompleted);
  
  #if USING_CO2_SENSOR
    setupCO2Sensor();
  #endif
  
  #if USING_DYNAMIXEL
    setupDynamixel();
  #endif

  #if USING_IMU
    setupIMU();
  #endif
  
}

void loop() {
 readFromSerial();
  
  updateTime(); 
  
  #if USING_DYNAMIXEL
    updateDynamixel();
  #endif

  #if USING_IMU
    if( !(!mpuInterrupt && fifoCount < packetSize) )//imu ready-to-read condition
      readIMU();
  #endif
  
  #if USING_CO2_SENSOR
    if(CurrentMicrosecs - elapsedTimeMicros > 500000){//Time to read CO2?
      elapsedTimeMicros = LastUpdateMicrosecs;
      readCO2sensor();
    }
  #endif

  updateData();//send data to computer  
  //delay(5);
  
}
