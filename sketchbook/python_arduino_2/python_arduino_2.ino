
byte dataIn[11]; //el buffer de entrada

void setup()
{
  pinMode(13,OUTPUT);
  Serial.begin(115200);
}

void loop()
{

if(Serial.available()>11)
{
  char temp=Serial.read();
  
 if (temp=='#')//en caso de ser # la cadena inicia
 {
  for (int i=0;i<11;i++)
  {
    dataIn[i]=Serial.read();  
  }
  byte crc=0;
  for(int i=0;i<10;i++)
  {
    crc+=dataIn[i];
  }
  if(crc==dataIn[10])
  {
    if(dataIn[0]==0)
    {
      digitalWrite(13,LOW);
    }
    else
    {
      digitalWrite(13,HIGH);
    }
    
    Serial.print("Robot: ");
    Serial.print(random(0,10));
    Serial.print(" ");
    Serial.print(random(10,20));
    Serial.print(" ");
    Serial.print(random(20,30));
    Serial.print(" ");
    Serial.print(random(30,40));
    Serial.print(" ");
    Serial.print(random(40,50));
    Serial.print(" ");
    Serial.println(random(50,60));
  }
 }
  else if(temp=='a')
  {

  }
}
}
