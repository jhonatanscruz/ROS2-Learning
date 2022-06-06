//Programa: Comunicacao I2C Arduino e Raspberry Pi
//Autor: Arduino e Cia

#include <Wire.h>
#include <string.h>

char str[10] = "TESTE";
String received_msg = "";

void setup()
{
  Serial.begin(9600);
  Wire.begin(0x18);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
}

void requestEvent()
{
  Serial.println("Requisicao recebida!");
  Wire.write(str);
}

void receiveEvent(int numbBytes)
{
  received_msg = "";
  Serial.print("Mensagem recebida: ");
    while(Wire.available() > 0){
      char c = Wire.read();
      if (c != '\n'){
        received_msg.concat(c);
      }
    }
  Serial.println(received_msg);
}

void loop()
{
  delay(50);
}
