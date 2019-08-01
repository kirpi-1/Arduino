#include "clientBase.h"

#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW

LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
SensorType sensorType = RAW;

WiFiClient client;

char buff[BUFFSIZE];
unsigned int pos = 0;
unsigned long curTime;
String MACAddress;
unsigned int lastTime = 0;
void setup() {  
  Serial.begin(115200); // Start serial at 115200 bps
  Wire.begin(SDA_PIN,SCL_PIN); // SDA, SCL
  
  // Set up interrupt pins as inputs:
  pinMode(INT1XM, INPUT);
  pinMode(INT2XM, INPUT);
  pinMode(DRDYG, INPUT);

  pinMode(0, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(0,LOW);
  digitalWrite(2,LOW);
  // Use the begin() function to initialize the LSM9DS0 library.
  // You can either call it with no parameters (the easy way):
  // TODO determine proper scales
  uint16_t status = dof.begin();
  // Or call it with declarations for sensor scales and data rates:  
  //uint16_t status = dof.begin(dof.G_SCALE_2000DPS, 
  //                            dof.A_SCALE_6G, dof.M_SCALE_2GS);

  Serial.print("LSM9DS0 WHO_AM_I's returned: 0x");
  Serial.println(status, HEX);
  Serial.println("Should be 0x49D4");
  Serial.println();

  Serial.print("connecting to wifi");
  WiFi.begin(ssid,pw);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  MACAddress = WiFi.macAddress();
  connectToBaseStation(client, host, port);
}

bool newData;
unsigned int idx = 0;
unsigned int numFrames = 0;
IMUData imuData;

void loop() {
  newData = readSensor(sensorType);  
  
  if(client.connected()){
    //Serial.println("client is connected!");      
//    digitalWrite(2,LOW);
//    digitalWrite(0,HIGH);
    if(!client.isSendWaiting()){
      if(newData){
        client.write((const uint8_t *)buff, pos);
        pos = BASE_OFFSET;
        newData = false;
        numFrames = 0;       
      }      
    }
  }
  else{
    connectToBaseStation(client,host,port);
  }

}




void printSensors(){
  Serial.print(dof.gx);
  Serial.print(dof.gy);
  Serial.print(dof.gz);
  Serial.print("\t")   ; 
  Serial.print(dof.ax);
  Serial.print(dof.ay);
  Serial.print(dof.az);
  Serial.print("\t");
  Serial.print(dof.mx);
  Serial.print(dof.my);
  Serial.print(dof.mz);
}

bool connectToBaseStation(WiFiClient &c, const char *host, unsigned int port){
  if(!client.connect(host,port)){
    Serial.println(">>> Could not connect!");
    digitalWrite(2,HIGH);
    digitalWrite(0,LOW);
  }
  else{
    memset(buff,0,BUFFSIZE);
    strcpy(buff,MACAddress.c_str());
    memcpy(buff+MAC_OFFSET,&sensorType,sizeof(SensorType));  
    client.write((const uint8_t *)&buff,MAC_OFFSET+sizeof(SensorType));
    pos = BASE_OFFSET;
    newData = false;
    numFrames = 0;
    digitalWrite(2,LOW);
    digitalWrite(0,HIGH);
  }
  return client.connected();
}


bool readSensor(SensorType st){
  switch(st){
    case IMU:
      return readSensorIMU();
      break;
    default:
      return false;
  }
}

bool readSensorRAW(){
  unsigned int t = millis();
  if(t - lastTime >100){
    
  }
  return true;
}

bool readSensorIMU(){
  if ((digitalRead(INT2XM)) && (digitalRead(INT1XM)) && (digitalRead(DRDYG))){
    //Serial.println("sensors ready");
    dof.readGyro();
    dof.readAccel();
    dof.readMag();    
    imuData.idx = idx++;
    imuData.time = millis();
    imuData.gx = dof.gx;
    imuData.gy = dof.gy;
    imuData.gz = dof.gz;
    imuData.ax = dof.ax;
    imuData.ay = dof.ay;
    imuData.az = dof.az;
    imuData.mx = dof.mx;
    imuData.my = dof.my;
    imuData.mz = dof.mz;
    
    //update the number of frames at the beginning of the send buffer
    numFrames++;
    memcpy(buff,&numFrames,sizeof(unsigned int));    
    /* frame is 
     *  current time
     *  index
     *  Grav X, Y, Z
     *  Accel X, Y, Z
     *  Mag X, Y, Z
     */
    //write the frame
    memcpy(buff+pos,&imuData,sizeof(IMUData));
    pos+=sizeof(IMUData);
    return true;
  }
  else
    return false;  
}
