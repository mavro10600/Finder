/*
 * Bien, este es un programa de prueba para 
 * usar los encoders ne forma digital, de acuerdo al
 * mapa de pines de la stellaris, y al codigo que le robé a 
 * jakob es bastante sencillo, declaramos un pin que servirá como  
 * clock y otro que va aservir como MOSI
 * conectar PD_2 a D0, PD_1 a CSN y   
 */

#define pindoIzq PA_6
#define pinclkIzq PA_7
#define pincsnIzq PE_3 

#define pindoDer PF_1
#define pinclkDer PF_3
#define pincsnDer PF_2 



#define pindo1 PD_2
#define pinclk1 PD_0
#define pincsn1 PD_1 

#define pindo2 PE_2
#define pinclk2 PD_3
#define pincsn2 PE_1 

#define pindo3 PC_4
#define pinclk3 PB_3
#define pincsn3 PC_5 

#define pindo4 PA_4
#define pinclk4 PA_3
#define pincsn4 PA_2 

int stat_complete=0;


int vueltasIzq;
unsigned int last_lecIzq;
boolean statIzq;

int vueltasDer;
unsigned int last_lecDer;
boolean statDer;

int vueltas1;
unsigned int last_lec1;
boolean stat1;

int vueltas2;
unsigned int last_lec2;
boolean stat2;

int vueltas3;
unsigned int last_lec3;
boolean stat3;

int vueltas4;
unsigned int last_lec4;
boolean stat4;

void setup() {
  // put your setup code here, to run once:
//aqui hay quew declarar los pines de pinclk y pind0
//Y iniciar la comunicacion serial
Serial.begin(9600);
///*
pinMode(pinclkIzq,OUTPUT);
pinMode(pindoIzq,INPUT);
pinMode(pincsnIzq,OUTPUT);
digitalWrite(pincsnIzq,HIGH);
digitalWrite(pinclkIzq, HIGH);  
delay(100);
//*/ 
///*
pinMode(pinclkDer,OUTPUT);
pinMode(pindoDer,INPUT);
pinMode(pincsnDer,OUTPUT);
digitalWrite(pincsnDer,HIGH);
digitalWrite(pinclkDer, HIGH);  
delay(100);
 //*/

pinMode(pinclk1,OUTPUT);
pinMode(pindo1,INPUT);
pinMode(pincsn1,OUTPUT);
digitalWrite(pincsn1,HIGH);
digitalWrite(pinclk1, HIGH);  
delay(100);
pinMode(pinclk2,OUTPUT);
pinMode(pindo2,INPUT);
pinMode(pincsn2,OUTPUT);
digitalWrite(pincsn2,HIGH);
digitalWrite(pinclk2, HIGH);  
delay(100);
pinMode(pinclk3,OUTPUT);
pinMode(pindo3,INPUT);
pinMode(pincsn3,OUTPUT);
digitalWrite(pincsn3,HIGH);
digitalWrite(pinclk3, HIGH);  
delay(100);

pinMode(pinclk4,OUTPUT);
pinMode(pindo4,INPUT);
pinMode(pincsn4,OUTPUT);
digitalWrite(pincsn4,HIGH);
digitalWrite(pinclk4, HIGH);
delay(100);

}


void loop() {
Update_Encoders();

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////Funcion para leer los encoders XD

void Update_Encoders()
{
 int left_lec=encoder_digital(pindoIzq,pinclkIzq,pincsnIzq,&last_lecIzq,&vueltasIzq,&statIzq);
 int right_lec=encoder_digital(pindoDer,pinclkDer,pincsnDer,&last_lecDer,&vueltasDer,&statDer);
 int enc1=encoder_digital(pindo1,pinclk1,pincsn1,&last_lec1,&vueltas1,&stat1);
 int enc2=encoder_digital(pindo2,pinclk2,pincsn2,&last_lec2,&vueltas2,&stat2);
 int enc3=encoder_digital(pindo3,pinclk3,pincsn3,&last_lec3,&vueltas3,&stat3);
 int enc4=encoder_digital(pindo4,pinclk4,pincsn4,&last_lec4,&vueltas4,&stat4);
 set_status(); 
 Serial.print("e");
 Serial.print("\t");
 Serial.print(enc1);
 Serial.print("\t");
 Serial.print(enc2);
 Serial.print("\t");
 Serial.print(enc3);
 Serial.print("\t");
 Serial.print(enc4);
 Serial.print("\t");
 Serial.print(left_lec);
 Serial.print("\t");
///*
  Serial.print(right_lec);
 Serial.print("\t");
 //*/
 Serial.print(stat_complete,BIN);
 Serial.print("\n");

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////Funcion para leer cada encoder pasando como valores de referencia 
int encoder_digital(int d0,int clk,int csn, unsigned int* last_lect,int* vueltas,boolean* statn)
{
int angle;
    digitalWrite(csn, LOW);
  delayMicroseconds(4);
    unsigned int data = 0;
  digitalWrite(clk, LOW);
  delayMicroseconds(1);
  for (uint8_t k = 0; k < 16; k++) 
    {
    digitalWrite(clk, HIGH);
    delayMicroseconds(1);
    data = (data << 1) | (digitalRead(d0) ? 0x0001 : 0x0000);
    digitalWrite(clk, LOW);
    delayMicroseconds(1);
     }


  digitalWrite(csn, HIGH);
  delayMicroseconds(1);
  digitalWrite(clk, HIGH);

//Condicionar la salida para que sea solo cuando es válido el valor del encoder
     if (!bitRead(data,3) && !bitRead(data,4) && bitRead(data,5) && !( bitRead(data,1) && bitRead(data,2) ))
     {
      *statn=true;
      if( abs((data>>6)-*last_lect) > 900 )
      {
              if((data>>6)>*last_lect)
                *vueltas-=1;
              else
                *vueltas+=1;     
      }
      else
      {*vueltas=*vueltas;}
        
     angle=(*vueltas)*(1023)+(data>>6) ;

     *last_lect=(data>>6);
     return (angle);
     }
     else
     {
      *statn=false;
      return(data>>6);
     }
}

void set_status()
{
  if(stat1)  bitWrite(stat_complete,0,1); else bitWrite(stat_complete,0,0);
  if(stat2)  bitWrite(stat_complete,1,1); else bitWrite(stat_complete,1,0);
  if(stat3)  bitWrite(stat_complete,2,1); else bitWrite(stat_complete,2,0);
  if(stat4)  bitWrite(stat_complete,3,1); else bitWrite(stat_complete,3,0);
  if(statIzq)  bitWrite(stat_complete,4,1); else bitWrite(stat_complete,4,0);
  if(statDer)  bitWrite(stat_complete,5,1); else bitWrite(stat_complete,5,0);
}

