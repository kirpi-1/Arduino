#define ESP_TCP
#include <Wire.h>
#include <ESP_LSM9DS0_TCP.h>

const byte SDA_PIN = 4;
const byte SCL_PIN = 5;
const byte INT1XM = 12; // INT1XM tells us when accel data is ready
const byte INT2XM = 13; // INT2XM tells us when mag data is ready
const byte DRDYG = 14;  // DRDYG tells us when gyro data is ready

ESP_LSM9DS0 myESP;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // Start serial at 115200 bps
  Wire.begin(SDA_PIN,SCL_PIN); // SDA, SCL  
  Serial.println();
  //myESP.UDPtest();

  myESP.setInterrupts(INT1XM, INT2XM, DRDYG);
  //myESP.setNetworkMode(netMode);
  myESP.begin();
  
  myESP.ssid = "cnslabw01";
  myESP.pw = "wcnslabw01@#$";
  myESP.host = "cns25";
  myESP.port = 49931;

  myESP.connectToWiFi();
}

void loop() {
  // put your main code here, to run repeatedly:    
  myESP.read();  
  myESP.send();
}




/*
// TODO
it seems that there are problems with #ifdef ESP_UDP, especially in connectToWiFi(), and it's not actually including
any of that when compiling.  Might have to just make a UDP version and TCP version separately, or use another IDE.
*/
