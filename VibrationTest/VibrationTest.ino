#include <LowPower.h>

bool lastVibrationStatus;

void setup() {
  Serial.begin(9600);
  pinMode(2, OUTPUT);
  pinMode(5, INPUT);
  digitalWrite(2, HIGH);

  //Read initial sensor state (will change state if vibration detected)
  lastVibrationStatus = digitalRead(5);
}

void loop() {

  //Check if vibration is detected
  if(lastVibrationStatus != digitalRead(5))
  {
    //Debug data
    Serial.println(F("VIBRATION DETECTED!"));
    Serial.flush();
  }

  //Store last vibration status
  lastVibrationStatus = digitalRead(5);

  digitalWrite(LED_BUILTIN, lastVibrationStatus);
}
