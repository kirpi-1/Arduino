#include <Wire.h>
#include <ESP_LSM9DS0_UDP.h>


ESP_LSM9DS0 myESP;
///////////////////////////////
// Interrupt Pin Definitions //
///////////////////////////////

const byte SDA_PIN = 4;
const byte SCL_PIN = 5;
const byte INT1XM = 12; // INT1XM tells us when accel data is ready
const byte INT2XM = 14; // INT2XM tells us when mag data is ready
const byte DRDYG = 13;  // DRDYG tells us when gyro data is ready
const byte INT_M = 16;

void setup() {    
  Serial.begin(115200); // Start serial at 115200 bps
  Wire.begin(SDA_PIN,SCL_PIN); // SDA, SCL     
  myESP.setInterrupts(INT1XM, INT2XM, DRDYG);
  myESP.begin();
  
  myESP.ssid = "cnslabw01";
  myESP.pw = "wcnslabw01@#$";
  myESP.host = "C13120201SPRO";//"C13120201SPRO";
  myESP.port = 49931;

  myESP.connectToWiFi();
  //myESP.connectToBaseStation();
}

void loop() {
  bool newData = myESP.read();
//  if(newData)
//    myESP.printSensorData();
  myESP.send();
  //delay(10000);
  //Serial.println(millis());
  
//  if(myESP.client.connected()){
//    digitalWrite(2,LOW);
//    digitalWrite(0,HIGH);
//  }
//  else{
//    digitalWrite(2,HIGH);
//    digitalWrite(0,LOW);
//  }

  

}



