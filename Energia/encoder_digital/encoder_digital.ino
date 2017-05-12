/*
 * Bien, este es un programa de prueba para 
 * usar los encoders ne forma digital, de acuerdo al
 * mapa de pines de la stellaris, y al codigo que le robé a 
 * jakob es bastante sencillo, declaramos un pin que servirá como  
 * clock y otro que va aservir como MOSI
 * conectar PD_2 a D0, PD_1 a CSN y   
 */

#define pindo PD_2
#define pinclk PD_0
#define pincsn PD_1 

void setup() {
  // put your setup code here, to run once:
//aqui hay quew declarar los pines de pinclk y pind0
//Y iniciar la comunicacion serial
Serial.begin(9600);
pinMode(pinclk,OUTPUT);
pinMode(pindo,INPUT);
pinMode(pincsn,OUTPUT);
digitalWrite(pincsn,HIGH);
delay(100);
}

void loop() {
  
  // put your main code here, to run repeatedly: 
  // Reading a single sensor data by software
  //Hasta este ppunto, el programa de solo lectura, es un éxito
  //En el siguiente programa, hay que envolver en una funcion lo que hace 
  //asignando pines y la variable destino
  //Y filtrando el valor de ingreso en funcion de los valores de los bits de status
  //

  digitalWrite(pincsn, LOW);
  delayMicroseconds(4);
    unsigned int data = 0;
  digitalWrite(pinclk, LOW);
  delayMicroseconds(1);
  for (uint8_t k = 0; k < 16; k++) 
    {
    digitalWrite(pinclk, HIGH);
    delayMicroseconds(1);
    data = (data << 1) | (digitalRead(pindo) ? 0x0001 : 0x0000);
    digitalWrite(pinclk, LOW);
    delayMicroseconds(1);
     }

  digitalWrite(pincsn, HIGH);
  delayMicroseconds(1);
  digitalWrite(pinclk, HIGH);
     
  Serial.print(data,BIN);
  Serial.print('\n');
}
