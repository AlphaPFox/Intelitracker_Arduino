#include <NMEAGPS.h>
#include <AltSoftSerial.h>
#include <SoftwareSerial.h>
#include <LowPower.h>

#define GPS_PORT_NAME "AltSoftSerial"

AltSoftSerial gpsPort; // 8 & 9 for an UNO

//Create software serial object to communicate with SIM800L
SoftwareSerial gsmPort(6, 7); //SIM800L Tx & Rx is connected to Arduino #3 & #2

//------------------------------------------------------------
// This object parses received characters
//   into the gps.fix() data structure

static NMEAGPS  gps;

//------------------------------------------------------------
//  Define a set of GPS fix information.  It will
//  hold on to the various pieces as they are received from
//  an RMC sentence.  It can be used anywhere in your sketch.

static gps_fix fix;

//Used to calculate how long the controller is awake
unsigned long awakeTimer;

bool gpsEnabled = false;
bool gsmEnabled = false;
bool vibrationEnabled = false;

boolean checkVibration = true;
int vibrationSleepCicles = 0;
int vibrationCheckPeriodicity = 30000;

//------------------ Pinout section ----------------
byte vibrationPowerPin = 2;
byte gpsModulePowerPin = 3;
byte gsmModulePowerPin = 4;
byte vibrationInputPin = 5;

//------------------- Timeout section --------------
// GPS -> Max time to obtain fix
// GSM -> Max time to register on network 
// Vibration -> Max time to detect vibration
int gpsTimeout = 30000;
int gsmTimeout = 30000;
int vibrationTimeout = 1000;

//List of states
enum State
{
  initializing,
  going_to_sleep,
  sleeping,
  reading_vibration,
  waiting_for_gps,
  waiting_for_gsm,
  sending_location
};

//Control arduino state
State current_state = initializing;

void setup()
{
  //Vibration on/off switch
  pinMode(vibrationPowerPin, OUTPUT);

  //GPS module on/off switch
  pinMode(gpsModulePowerPin, OUTPUT);

  //GSM module on/off switch
  pinMode(gsmModulePowerPin, OUTPUT);
  
  //Vibration input values
  pinMode(vibrationInputPin, INPUT);

  //GPS and vibration off by default
  digitalWrite(vibrationPowerPin, LOW);
  digitalWrite(gpsModulePowerPin, LOW);

  //GSM module is on by default (on sleep mode)
  digitalWrite(gsmModulePowerPin, LOW);
  
  //Begin serial communication with Arduino and SIM800L
  gsmPort.begin(9600);

  //Begin serial communication with Arduino and UBLOX NEO 6M GPS
  gpsPort.begin(9600);

  //Begin serial communication with Arduino and Hardware Serial (DEBUG)
  Serial.begin(9600);

  //Initialize finished, go to sleep
  current_state = sleeping;
    
}

void loop()
{
  if(gsmPort.available())
      Serial.write(gsmPort.read());
      
  switch(current_state)
  {   
    case sleeping:

      Serial.println("Sleeping...");
      Serial.flush();
      
      // Enter power down state for 8 s with ADC and BOD module disabled
      LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);

      // If Vibration detection is enabled
      if(checkVibration)
      { 
        //Increase sleep cicles counter
         vibrationSleepCicles++;
          
        // Check if should wake up (Each sleep cicle counts for 8 secs or 8000 milliseconds)
        if(vibrationSleepCicles * 8000 > vibrationCheckPeriodicity)
        {
          
          Serial.println("Awake, checking for vibration...");
          Serial.flush();
 
          //Reset sleep cicle counter
          vibrationSleepCicles = 0;
        
          //Change state to read_vibration
          current_state = reading_vibration;

          //Enable vibration module
          digitalWrite(vibrationPowerPin, HIGH);

          //Start awake timer
          awakeTimer = millis();
        }
      }

      //Finish state
      break;

    case reading_vibration:

      //Check for how long is reading vibration
      if(millis() - awakeTimer < vibrationTimeout)
      {
        //Check if vibration is detected
        if(digitalRead(vibrationInputPin) == HIGH)
        {
          Serial.println("VIBRATIONNN, SEND SMS!!");
          Serial.flush();
          
          //Vibration detected, sending current location
          current_state = sending_location;
  
          //Turn off vibration module
          digitalWrite(vibrationPowerPin, LOW);
        }
      }
      else
      {
        //No vibration detected, going back to sleep
        current_state = sleeping;

        //Turn off vibration module
        digitalWrite(vibrationPowerPin, LOW);
      }
      
      break;

    case sending_location:

      //Check if GPS is module enabled
      if(!gsmEnabled)
      {
        //Enable GPS module
        digitalWrite(gsmModulePowerPin, HIGH);
        
        //Update flag
        gsmEnabled = true;
        Serial.println("GSM_MODULE -> ON");
        Serial.flush();
        
        //Initialize GSM module
        Serial.println("Initializing GSM...");
        
        //Wait for GSM module to power on
        delay(5000);
        
        //Simple AT command to initalize communication
        sendGSM("AT", 300);
      
        //Signal quality test, value range is 0-31 , 31 is the best
        sendGSM("AT+CSQ", 300);
      
        //Read SIM information to confirm whether the SIM is plugged
        sendGSM("AT+CCID", 300); 
      
        //Check whether it has registered in the network
        sendGSM("AT+CREG?", 500);
      }
      else if(!gpsEnabled)
      {
        //Enable GPS module
        digitalWrite(gpsModulePowerPin, HIGH);
        
        //Update flag
        gpsEnabled = true;
        Serial.println("GPS_MODULE -> ON");
        Serial.flush();
      }
      //Check if GPS already supplied a valid position
      else if(!fix.valid.location && gps.available(gpsPort))
      {
        //Read GPS data
        fix = gps.read();

        //If gps obtained a valid location
        if (fix.valid.location) 
        {
          sendGSM("AT+CMGF=1", 300); // Configuring TEXT mode
          
          sendGSM("AT+CMGS=\"+5567981977017\"", 300); // Set destination

          char coordinates[20];
          
          dtostrf(fix.latitude(), 0, 4, coordinates);
          sendGSM(coordinates, 300);
          sendGSM("," , 300);
          dtostrf(fix.longitude(), 0, 4, coordinates);
          sendGSM(coordinates, 300); 
          sendGSM("Test data", 300);

          //End char
          sendGSM((char)26, 300);
          
          Serial.println("SENT! Going back to sleep");
          Serial.flush();

          //Clear fix object
          fix.valid.location = false;

          //Turn off GPS module
          digitalWrite(gpsModulePowerPin, LOW);

          //Update flag
          gpsEnabled = false;
          Serial.println("GPS_MODULE -> OFF");
          Serial.flush();
          

          //Turn off GSM module
          digitalWrite(gsmModulePowerPin, LOW);

          //Update flag
          gsmEnabled = false;
          Serial.println("GSM_MODULE -> OFF");
          Serial.flush();
          
          //Go back to sleep
          current_state = sleeping;
        }
      }   
      break;  
  }
  
}

void sendGSM(const char* command, int timeout)
{
  Serial.print("Sending command: ");
  Serial.println(command);
  gsmPort.println(command);

  int sentAt = millis();
  String response = "";

  while(millis() - sentAt < timeout)
  {
    if(gsmPort.available())
      Serial.write(gsmPort.read());
  }
}

void sendGSM(char command, int timeout)
{
  sendGSM(&command, timeout);
}

void readGSM(uint16_t timeout) 
{                                                               // The main function which parses the responses provided by the SIM800 chip into the ioBuffer char array.
    clearBuffer();                                              // First the ioBuffer is cleared from old data.
    timeOut(INIT);                                              // Initial timestamp is taken to calculate the reply timeout.
    while (!simCom.available()) {                               // If the module does not reply at all, indicate timeout.
        if (timeOut(RUN)) {
            strcat_P(ioBuffer, P("TIMEOUT"));
            break;
        }
    }
    uint32_t i = 0;                                                 // The counter which counts each char received from the SIM800 module.
    while (!timeOut(RUN)) {                                         // While time has not run out (timeLimit has not been reached).
        if (simCom.available()) {                                   // If next char becomes available
            timeOut(DELAY);                                         // If benchmark has not been enabled, take new timestamp to delay timeout.
            if (slicePoint == 0 && i < DEF_BUFFER_SIZE) {           // If slicing is disabled and the ioBuffer is not full,
                ioBuffer[i] = simCom.read();                        // read the available char into ioBuffer index i.
                if (ioBuffer[i] == endChar && endOfTx(i)) break;    // If the value read equals 10 and endOfTx at index i detects presence of "OK" or "ERROR", end read().
                i++;                                                // Else, increment char counter.
            } else if (slicePoint != 0 && slicePoint == i + 1) {    // If slicing has been enabled and the char counter has reached the value indicated by the slicePoint,
                i = slicePoint = 0;                                 // reset char counter and disable slicing, which in turn allows chars to be saved into the ioBuffer.
            } else {                                                // If slicing is enabled and slice point has not been reached, OR, the ioBuffer is full,
                simCom.read();                                      // read the available chars into sink (discard them)
                i++;                                                // and increment the char counter.
            }
        }
    }
    timeOut(GETBM);                                              // If benchmark has been enabled, calculate the time it took to receive the reply and save it into global variable benchTime.
    overrideTimeout(0);                                          // If override is enabled, disable override and put the old timeLimit value back into the timeLimit variable.
    replaceEscapeChars();                                        // If enabled, replace unreadable ASCII control chars 10 and 13 with spaces. This is necessary for the reply() method to work properly.
    slicePoint = 0;                                              // If slicing had been enabled, but the reply never reached the slice point (slicePoint value is larger than reply char count), disable slicing to prevent errors.

    #ifdef DEBUG
    debug(REPLY);                                                    // If #define DEBUG is uncommented in the SIM800.h file, print the contents of ioBuffer to Serial Monitor.
    #endif
}

bool timeOut(TimerType cmd) {                                   // The main timeout function which calculates if the current read() execution has exceeded the set time limit.
    static unsigned long timeStamp;                             // If execution duration has exceeded set time limit, returns true. Else returns false.
    static bool benchmark = false;                              // Benchmark is most useful if detectEndStr is enabled, otherwise it will just return the max timeLimit.
    
    unsigned long currentTime = micros();                       // Each call to timeOut gets the current time first before deciding what to output.

    switch (cmd) {
        case DELAY:
            if (!benchmark) timeStamp = currentTime;                        // If benchmark is disabled, set new timeStamp to delay the timeout.
            break;                                                          // This causes the read() method to count timeout from the last char it has received.
        case RUN:
            if (currentTime - timeStamp >= this->timeLimit) return true;    // If timeLimit is exceeded, return true.
            break;
        case INIT:
            timeStamp = currentTime;                                        // Initialize timeStamp. This is done once first each time read() is called.
            break;
        case GETBM:
            if (benchmark) this->benchTime = currentTime - timeStamp;       // If benchmark is enabled, save read() execution duration into global variable benchTime.
            break;
        case SETBM:
            benchmark = true;                                               // Enables read() benchmarking.
            break;
        case UNSETBM:
            benchmark = false;                                              // Disables read() benchmarking.
            break;
    }

    return false;
}
