#include <RH_RF95.h>
#include <SPI.h>

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  Serial.println("Firmware init... please wait");

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
