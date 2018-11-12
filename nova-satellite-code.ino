#include <RH_RF95.h>
#include <SPI.h>

#include <Wire.h>
#include <MPU6050.h>
#include <BMP085.h>
#include <HMC5883L.h>
#include <NMEAGPS.h>
#include <NeoSWSerial.h>

#define RFM95_CS  10
#define RFM95_RST 9
#define RFM95_INT 2

#define GPS_PORT_NAME "NeoSWSerial"

RH_RF95        rf95(RFM95_CS, RFM95_INT);
NeoSWSerial    gpsPort(5, 4);
MPU6050  mpu;
BMP085   bmp;
HMC5883L hmc;
gps_fix  fix;
static NMEAGPS gps;


uint8_t packet[35] = {};
uint16_t packet_ctr = 0;
int16_t temperature, accX, accY, accZ;
uint32_t pressure;

const char* START_MODULE   = ": Starting module";
const char* SUCCESS_MODULE = ": Module initialized";
const char* FAILURE_MODULE = ": Could not initialize module";

void printTimeLead(){
  Serial.print("[");
  Serial.print(millis()/1000.0, 3);
  Serial.print("] ");
}

void debug(const char* str){
  printTimeLead();
  Serial.println(str);
}

void moduleStatusMessage(const char* name, const char* msg){
  printTimeLead();
  Serial.print(name);
  Serial.print(": ");
  Serial.println(msg);
}


bool customSend(const uint8_t* data, uint8_t len) {
  if (len > 0xff)
	  return false;

  rf95.waitPacketSent(); // Make sure we don't interrupt an outgoing message
  rf95.setModeIdle(); //And we're off the air... 

  rf95.spiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0); //Set the FIFO pointer to the start
  rf95.spiBurstWrite(RH_RF95_REG_00_FIFO, data, len); //Write len bytes of data to the FIFO
  rf95.spiWrite(RH_RF95_REG_22_PAYLOAD_LENGTH, len); //Set length

  rf95.setModeTx(); //*coughs into mic* is this thing on?
  return true;
}

void handleGPSChar(uint8_t c){
  gps.handle(c);
}

void initRadio() {
  moduleStatusMessage("Radio", START_MODULE);
 
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  if(!rf95.init()) {
    debug("Radio: module init failed - halting");
    while(1) {;}
  }

  if(!rf95.setFrequency(915.00)) {
    debug("Radio:   Could not set frequency to 915.00 MHz");
    while(1) {;}
  } else {
    debug("Radio:   Frequency: 915.00 MHz");
  }
  
  rf95.spiWrite(0x1d, 0x68);  //BW: 31.25 kHz, CR: 4/8, implicit header
  rf95.spiWrite(0x1e, 0x94);  //SF: 10
  rf95.spiWrite(0x26, 0x04);  //TBD
  rf95.setPreambleLength(6);  //Awful waste of air time, but whatever...
  rf95.setTxPower(12, false); 

  debug("Radio:   BW: 125 kHz, CR: 4/8, SF: 10,  AGC: on, CRC: off");
  debug("Radio:   Preamble length: 6 symbols");
  debug("Radio:   TX power: 12 dBm");

  moduleStatusMessage("Radio", SUCCESS_MODULE);
}

void initPosition() {
  moduleStatusMessage("GPS", START_MODULE);

  gpsPort.attachInterrupt(handleGPSChar);
  gpsPort.begin(9600);
  gps.send(&gpsPort, "$PMTK001,886,3");
  gps.send(&gpsPort, "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");

  moduleStatusMessage("GPS", SUCCESS_MODULE);
}

void initMPU() {
  moduleStatusMessage("MPU", START_MODULE);
  return;

  mpu.initialize();

  if(mpu.testConnection()) {
    moduleStatusMessage("MPU", SUCCESS_MODULE);
  } else {
    moduleStatusMessage("MPU", FAILURE_MODULE);
  }
}

void initBMP() {
  moduleStatusMessage("BMP", START_MODULE);

  bmp.initialize();

  if(bmp.testConnection()) {
    moduleStatusMessage("BMP", SUCCESS_MODULE);
  } else {
    moduleStatusMessage("BMP", FAILURE_MODULE);
  }
}

void initHMC(){
  moduleStatusMessage("HMC", START_MODULE);

  hmc.initialize();

  if(hmc.testConnection()) {
    moduleStatusMessage("HMC", SUCCESS_MODULE);
  } else {
    moduleStatusMessage("HMC", FAILURE_MODULE);
  }
}

void sendMeasurementRequests(){
  bmp.setControl(BMP085_MODE_TEMPERATURE);
  temperature  = (int16_t) (bmp.getTemperatureC() * 10.0f);

  bmp.setControl(BMP085_MODE_PRESSURE_3);
  pressure = bmp.getPressure();

  mpu.getAcceleration(&accX, &accY, &accZ);
}

void formPacket(uint8_t *packet){
  packet_ctr++;
  uint32_t time_boot = millis();
  uint32_t speed     = (uint32_t) (fix.speed_kph() * 27.77777f);
  
  packet[0] = packet_ctr;
  packet[1] = packet_ctr >> 8;
  packet[2] = time_boot;
  packet[3] = time_boot >> 8;
  packet[4] = time_boot >> 16;
  packet[5] = temperature;
  packet[6] = temperature >> 8;
  packet[7] = pressure;
  packet[8] = pressure >> 8;
  packet[9] = pressure >> 16;
  packet[10] = accX;
  packet[11] = accX >> 8;
  packet[12] = accY;
  packet[13] = accY >> 8;
  packet[14] = accZ;
  packet[15] = accZ >> 8;
  packet[16] = fix.latitudeL();
  packet[17] = fix.latitudeL() >> 8;
  packet[18] = fix.latitudeL() >> 16;
  packet[19] = fix.latitudeL() >> 24;
  packet[20] = fix.longitudeL();
  packet[21] = fix.longitudeL() >> 8;
  packet[22] = fix.longitudeL() >> 16;
  packet[23] = fix.longitudeL() >> 24;
  packet[24] = fix.altitude_cm();
  packet[25] = fix.altitude_cm() >> 8;
  packet[26] = fix.altitude_cm() >> 16;
  packet[27] = speed;
  packet[28] = speed >> 8;
  packet[29] = speed >> 16;
  packet[30] = 0; //magnetometer x8
  packet[31] = 0; //magnetometer x4, y4
  packet[32] = 0; //magnetometer y8
  packet[33] = 0; //magnetometer y4, z4
  packet[34] = 0; //magnetometer z4
  packet[34] |= fix.satellites << 4;
  packet[34] |= fix.valid.location << 7;
  Serial.println(time_boot);
}

void setup() {
  Serial.begin(115200);

  debug("Hello world! This is NOVA-1 waking up!");

  initRadio();
  initPosition();
  initMPU();
  initBMP();
  initHMC();

  debug("All modules are go for launch.");

  sendMeasurementRequests();
}

void loop() {
  if(gps.available()){
    fix = gps.read();
    formPacket(packet);
    rf95.send(packet, 35);
    sendMeasurementRequests();
  }
}
