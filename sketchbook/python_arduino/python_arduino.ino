


void setup()
{
  pinMode(13,OUTPUT);
  Serial.begin(115200);
}

void loop()
{

if(Serial.available())
{
  char temp=Serial.read();
  
 if (temp=='a')
 {
   Serial.println("apagado")
   digitalWrite(13,LOW);
 }
  else if(temp=='a')
  {
    Serial.println("encendido");
    digitalWrite(13,HIGH);
  }
}
}
