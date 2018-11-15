#include <RH_RF95.h>
#include <SPI.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP085_U.h>
#include <DHT_U.h>
#include <NMEAGPS.h>
#include <NeoSWSerial.h>

#define RFM95_CS  10
#define RFM95_RST 9
#define RFM95_INT 2

#define GPS_PORT_NAME "NeoSWSerial"

RH_RF95        rf95(RFM95_CS, RFM95_INT);
NeoSWSerial    gpsPort(5, 4);
gps_fix  fix;
static NMEAGPS gps;


uint8_t packet[35] = {};
uint16_t packet_ctr = 0;
int16_t temperature, accX, accY, accZ, mx, my, mz;
uint32_t pressure;
uint32_t lastPacketSent = 0;

const char* START_MODULE   = "Starting module";
const char* SUCCESS_MODULE = "Module initialized";
const char* FAILURE_MODULE = "Could not initialize module";

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
  
  rf95.setModemConfig(2);
  //rf95.spiWrite(0x1d, 0x78);  //BW: 125 kHz, CR: 4/8, implicit header
  //rf95.spiWrite(0x1e, 0x94);  //SF: 10
  //rf95.spiWrite(0x26, 0x04);  //TBD
  rf95.setPreambleLength(6);  //Awful waste of air time, but whatever...
  rf95.setTxPower(23, false); 

  debug("Radio:   BW: 32.5 kHz, CR: 4/8, SF: 9,  AGC: on, CRC: on");
  debug("Radio:   Preamble length: 6 symbols");
  debug("Radio:   TX power: 23 dBm");

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

void formPacket(uint8_t *packet){
  packet_ctr++;
  uint32_t time_boot = millis();
  uint32_t speed     = (uint32_t) (fix.speed_kph() * 27.77777f);
  
  packet[0] = packet_ctr;
  packet[1] = packet_ctr >> 8;
  packet[2] = time_boot;
  packet[3] = time_boot >> 8;
  packet[4] = time_boot >> 16;
  packet[5] = fix.latitudeL();
  packet[6] = fix.latitudeL() >> 8;
  packet[7] = fix.latitudeL() >> 16;
  packet[8] = fix.latitudeL() >> 24;
  packet[9] = fix.longitudeL();
  packet[10] = fix.longitudeL() >> 8;
  packet[11] = fix.longitudeL() >> 16;
  packet[12] = fix.longitudeL() >> 24;
  packet[13] = fix.altitude_cm();
  packet[14] = fix.altitude_cm() >> 8;
  packet[15] = fix.altitude_cm() >> 16;
  packet[16] = speed;
  packet[17] = speed >> 8;
  packet[18] = speed >> 16;
  packet[19] |= fix.satellites << 4;
  packet[19] |= fix.valid.location << 7;
}

void setup() {
  Serial.begin(115200);

  debug("Hello world! This is NOVA-1 waking up!");

  initRadio();
  initPosition();

  debug("All modules are go for launch.");
}

void loop() {
  if(gps.available()){
    fix = gps.read();
    formPacket(packet);
    rf95.send(packet, 20);
    debug("Packet sent");
    delay(3000);
  }
}
