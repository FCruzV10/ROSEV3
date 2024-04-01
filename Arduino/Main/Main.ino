#include <Wire.h>
#include <ros.h>
#include <std_msgs/UInt8.h>

#define SLAVE_ADDRESS 0x04

uint8_t codeSensorEV3 = 0;
bool recepcionEV3 = false;
uint8_t codeMotorEV3 = 50;

void procesarMotores( const std_msgs::UInt8& comando){
  codeMotorEV3 = comando.data;
}

ros::NodeHandle rosNode;
std_msgs::UInt8 rosMsj;
ros::Publisher sensores("sensores", &rosMsj);
ros::Subscriber<std_msgs::UInt8> motores("motores", procesarMotores);

void setup() {
  // Conexión EV3
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEV3);
  Wire.onRequest(sendEV3);

  // Conexión ROS
  rosNode.initNode();
  rosNode.advertise(sensores);
  rosNode.subscribe(motores);
}

void loop() {
  // Procesar Motores
  // Los motores se procesan en las funciones sendEV3 y procesarMotores
  
  // Procesar Sensores
  if (recepcionEV3) {
    rosMsj.data = codeSensorEV3;
    sensores.publish( &rosMsj );
    recepcionEV3 = false;
  }

  rosNode.spinOnce();
}

void receiveEV3(int byteCount) {
  while (Wire.available() > 0) {
    codeSensorEV3 = Wire.read();
    recepcionEV3 = true;
  }
}

void sendEV3() {
  Wire.write(codeMotorEV3);
}
