#include <RH_RF95.h>
#include <SPI.h>

#include <Wire.h>
#include <MPU6050.h>
#include <BMP085.h>
#include <HMC5883L.h>

#define RFM95_CS  10
#define RFM95_RST 9
#define RFM95_INT 2


RH_RF95  rf95(RFM95_CS, RFM95_INT);
MPU6050  mpu;
BMP085   bmp;
HMC5883L hmc;

void debug(String str) {
  /*
   * Prints a given message to serial in a friendly debug format, like this:
   * [1.334] Radio: TX power increased to 23 dBm
   * 
   * @param str Message to print to Serial output
   */
  Serial.print("[");
  Serial.print(millis()/1000.0, 3);
  Serial.print("] ");
  Serial.println(str);
}

bool customSend(const uint8_t* data, uint8_t len) {
  if (len > 0xff)
	return false;

  rf95.waitPacketSent(); // Make sure we dont interrupt an outgoing message
  rf95.setModeIdle();

  rf95.spiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);
  rf95.spiBurstWrite(RH_RF95_REG_00_FIFO, data, len);
  rf95.spiWrite(RH_RF95_REG_22_PAYLOAD_LENGTH, len);

  rf95.setModeTx();
  return true;
}

void initRadio() {
  debug("Radio: Starting module");
 
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

  rf95.spiWrite(0x1d, 0x77); //BW: 125 kHz, CR: 4/7, implicit header
  rf95.spiWrite(0x1e, 0x90); //SF: 9, no CRC
  rf95.spiWrite(0x26, 0x0c); //Mobile node, AGC on
  rf95.setPreambleLength(6);
  rf95.setTxPower(10, false);

  debug("Radio:   BW: 125 kHz, CR: 4/7, SF: 9,  AGC: on, CRC: off");
  debug("Radio:   Preamble length: 6 symbols");
  debug("Radio:   TX power: 10 dBm");

  debug("Radio: Module initialized");
}

void initPosition() {
  debug("GPS: Starting module");
  debug("GPS: Module initialized");
}

void initMPU() {
  debug("MPU: Starting module");

  mpu.initialize();

  if(mpu.testConnection()) {
    debug("MPU: Module initialized");
  } else {
    debug("MPU: Could not initialize module");
  }
}

void initBMP() {
  debug("BMP: Starting module");

  bmp.initialize();

  if(bmp.testConnection()) {
    debug("BMP: Module initialized");
  } else {
    debug("BMP: Could not initialize module");
  }
}

void initHMC(){
  debug("HMC: Starting module");

  hmc.initialize();

  if(hmc.testConnection()) {
    debug("HMC: Module initialized");
  } else {
    debug("HMC: Could not initialize module");
  }
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
}

void loop() {
  uint32_t start = millis();
  uint32_t counter = 0;
  return;
  while(1){
    //customSend((uint8_t *)&packet, packet.length());
    uint32_t elapsed = millis() - start;
    Serial.print("Sent " + String(counter) + " bytes in ");
    Serial.print(elapsed / 1000.0, 3);
    Serial.print(" seconds (");
    Serial.print(((counter-37)*8.0)/(elapsed/1000.0));
    Serial.println(" bps)");
  }
  
}
