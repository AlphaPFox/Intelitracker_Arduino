#include <SIM800.h>
#include <LowPower.h>

//   This sketch is used for 
//   testing raw AT commands 
//   in the serial monitor.

unsigned long bauds = 9600;

void setup() {
  Serial.begin(bauds);
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  while (!Serial) {;}
  gsmModule.begin(bauds);
}

void loop() {
  gsmModule.directSerialMonitor();
}
