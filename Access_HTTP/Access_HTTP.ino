#include <SIM800.h>

unsigned long bauds = 9600;

void setup() {
  Serial.begin(bauds);  
  while (!Serial) {;}
  gsmModule.begin(bauds);
  delay(100);
  
  //List all stored SMS
  gsmModule.send(EXE,P("CBC"), P(""));
  Serial.println(readVcc());
}

void loop() {}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}
