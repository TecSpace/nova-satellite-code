#include <RH_RF95.h>
#include <SPI.h>

#define RFM95_CS  10
#define RFM95_RST 9
#define RFM95_INT 2

RH_RF95 rf95(RFM95_CS, RFM95_INT);

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

void initRadioSubsystem() {
  debug("Radio: Starting subsystem");
 
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  if(!rf95.init()) {
    debug("Radio: Subsystem init failed - halting");
    while(1) {;}
  }

  debug("Radio: Module initialized");
}


void setup() {
  Serial.begin(115200);
  debug("Hello world! This is NOVA-1 waking up!");
  initRadioSubsystem();
}

void loop() {

}
