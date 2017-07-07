/*
 * Este programa controla el Dynamixel AX12-A con el microcontrolador Arduino pro-mini
 * y se comunica con la Tiva C de los sensores para que esta envié el estado del servo y reciba los comandos a la computadora.
 * Esto lo tuve que hacer ya que la Tiva dejaba de funcionar cuando se intentaba leer la imu y controlar el dynamixel en el 
 * mismo loop. Al parecer la función mpu.initialize() trababa  a la stellaris. Intenté modificar las bibliotecas pero no tuve éxito
 * la comunicación con la tiva es por serial utilizando la biblioteca SoftwareSerial y se utiliza el UART nativo del Arduino 
 * para el control del dynamixel.
*/
#include <SoftwareSerial.h>
//#include <DynamixelSerial.h>
//#include <SimpleDynamixel.h>

#define DATA_CONTROL_PIN 2

//DynamixelClass dynamixel_obj(&Serial, DATA_CONTROL_PIN);//Cambiar a serial1 en si se usa arduino leonardo
//SimpleDynamixelClass dynamixel(&dynamixel_obj, 600, 850, 100);//ancho de pulso minimo 600, ancho de pulso maximo 850, operacion de -100 a 100
SoftwareSerial tivaSerial(3, 4); // RX, TX
/*
void setupDynamixel(){
    dynamixel_obj.begin(1000000UL, DATA_CONTROL_PIN);
    dynamixel_obj.setMaxTorque(1, 1023);//Se define el maximo par disponible
    dynamixel.write(gripper_out);//Start dynamixel totally open
    delay(100);
}*/
void setup() {
  // Open serial communications and wait for port to open:
  tivaSerial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.begin(9600);
  //setupDynamixel();

}

void loop() {
  if(tivaSerial.available()>0){
    Serial.println(tivaSerial.read());
  }
 if(Serial.available()) {
    tivaSerial.write(Serial.read());
  }
}
