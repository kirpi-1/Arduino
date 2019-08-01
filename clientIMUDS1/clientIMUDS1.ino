#include <Wire.h>
#include <ESP_LSM9DS1_UDP.h>

const byte SDA_PIN = 4;
const byte SCL_PIN = 5;

const byte INT1_PIN = 14;
const byte INT2_PIN = 12;
const byte RDYM_PIN = 13;

ESP_LSM9DS1 myESP;

void setup() {
 
  Serial.begin(115200);
  Wire.begin(SDA_PIN,SCL_PIN);  
  myESP.setInterruptPins(INT1_PIN,INT2_PIN,RDYM_PIN); 
  myESP.begin();
  Serial.println(myESP.MAC);  
  myESP.ssid = "CNSLab_Experiment";//"cnslabw01";
  myESP.pw = "wcnslabw01@#$";
  myESP.host = "192.168.1.12";//"C13120201SPRO";//"cns25";
  myESP.port = 49931;

  myESP.connectToWiFi();
  myESP.connectToBaseStation();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  bool newData = myESP.read();
  myESP.send();
}
