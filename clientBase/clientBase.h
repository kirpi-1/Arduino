#include <Wire.h>
#include <SFE_LSM9DS0.h>
#include <pfodESP8266WiFi.h>
#include <pfodESP8266BufferedClient.h>

#define BUFFSIZE 1024
#define MAC_OFFSET 32
const unsigned int BASE_OFFSET = sizeof(unsigned int);

///////////////////////////////
// Interrupt Pin Definitions //
///////////////////////////////
// TODO: set these to the correct pin numbers
const byte INT1XM = 12; // INT1XM tells us when accel data is ready
const byte INT2XM = 13; // INT2XM tells us when mag data is ready
const byte DRDYG = 14;  // DRDYG tells us when gyro data is ready
const byte SDA_PIN = 4;
const byte SCL_PIN = 5;

///////////////////////////////
//     Wi-Fi Definitions     //
///////////////////////////////
const char* ssid = "cnslabw01";
const char* pw = "wcnslabw01@#$";
const char* host = "cns25";
unsigned int port = 49931;


struct IMUData{
  unsigned int time;
  unsigned int idx;  
  float gx,gy,gz,ax,ay,az,mx,my,mz;
};
enum SensorType{RAW = 0, IMU = 1, GSR = 8, RESPIRATION = 16};

bool readSensor(SensorType st);
bool connectToBaseStation(WiFiClient &c, const char *host, unsigned int port);
void printSensors();
