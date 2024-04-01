#include <Wire.h>
#define SLAVE_ADDRESS 0x04

int val, flag = 0;

void setup(){
  Serial.begin(9600);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  Serial.println("Ready!");
}

void loop(){
  if (flag == 1){
    Serial.println(val);
    flag = 0;
  }
}

void receiveData(int byteCount){
  while (Wire.available() > 0){
    val = Wire.read();
    flag = 1;
  }
}

void sendData(){}
