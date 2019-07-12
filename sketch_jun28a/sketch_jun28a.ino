#include <NMEAGPS.h>
#include <AltSoftSerial.h>
#include <SoftwareSerial.h>
#include <LowPower.h>
#include <SIM800.h>

#define GPS_PORT_NAME "AltSoftSerial"

AltSoftSerial gpsPort; // 8 & 9 pins

//------------------------------------------------------------
// This object parses received characters
//   into the gps.fix() data structure

static NMEAGPS  gps;

//------------------------------------------------------------
//  Define a set of GPS fix information.  It will
//  hold on to the various pieces as they are received from
//  an RMC sentence.  It can be used anywhere in your sketch.

static gps_fix fix;

//------------------ Boolean flags ---------------------------
// gpsEnabled -> GPS module status (ON/OFF)
// gsmEnabled -> GSM/GPRS module status (ON/OFF)
// gpsPositionAvailable -> True if a GPS position is available (and not sent yet)
// gsmNetworkAvailable -> Indicates whether GSM module is registered on mobile network

bool gpsEnabled = false;
bool gsmEnabled = false;
bool gpsPositionAvailable = false;
bool gsmNetworkAvailable = false;
bool checkVibration = true;

//------------------ Pinout section --------------------------
// vibrationPowerPin -> Turn on and off SW-420 vibration module (connected directly to module VIN)
// gpsModulePowerPin -> Turn on and off UBLOX NEO-6M GPS module (connected to a NPN transistor)
// gsmModulePowerPin -> Turn on and off SIM 800L module (also connected to a NPN transistor)
// vibrationInputPin -> Read data from SW-420 digital ouput pin (HIGH if vibration detected)

byte vibrationPowerPin = 2;
byte gpsModulePowerPin = 3;
byte gsmModulePowerPin = 4;
byte vibrationInputPin = 5;

//------------------- Timer section ------------------------
// GPS -> Max time to obtain fix
// GSM -> Max time to register on network 
// Vibration -> Max time to detect vibration
int gpsTimeout = 30000;
int gsmTimeout = 30000;
int maxReadingTime = 1000;

int sleepCicleCounter = 0;
int vibrationCheckPeriodicity = 30000;
unsigned long awakeTimer;

//List of states
enum State
{
  sleeping,
  reading_vibration,
  waiting_for_gps,
  waiting_for_gsm,
  sending_location
};

//Control arduino state
State current_state;

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
  
  //Begin serial communication with Arduino and UBLOX NEO 6M GPS
  gpsPort.begin(9600);

  //Begin serial communication with Arduino and Hardware Serial (DEBUG)
  Serial.begin(9600);

  //Setup finished, go to sleep by default
  current_state = sleeping;

}

void loop() {
      
  switch(current_state)
  {   
    case sleeping:

      Serial.println(F("Sleeping..."));
      Serial.flush();
      
      // Enter power down state for 8 s with ADC and BOD module disabled
      LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);

      // If Vibration detection is enabled
      if(checkVibration)
      { 
        //Increase sleep cicles counter
         sleepCicleCounter++;
          
        // Check if should wake up (Each sleep cicle counts for 8 secs or 8000 milliseconds)
        if(sleepCicleCounter * 8000 > vibrationCheckPeriodicity)
        {
          
          Serial.println(F("Awake, checking for vibration..."));
          Serial.flush();
          
          //Reset sleep cicle counter
          sleepCicleCounter = 0;
        
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
      if(sleepCicleCounter * 30 < maxReadingTime)
      {
        //Check if vibration is detected
        if(digitalRead(vibrationInputPin) == HIGH)
        {
          Serial.println(F("VIBRATIONNN, SEND SMS!!"));
          
          //Vibration detected, sending current location
          current_state = sending_location;
  
          //Turn off vibration module
          digitalWrite(vibrationPowerPin, LOW);

          //Reset sleep cicle counter
          sleepCicleCounter = 0;
        }
        else
        {
          //Use led to indicate reading time
          digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

          //Sleep for 30 ms before reading again
          LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);

          //Increment short sleep cicles
          sleepCicleCounter++;
        }
      }
      else
      {
        //No vibration detected, going back to sleep
        current_state = sleeping;
        
        //Reset sleep cicle counter
        sleepCicleCounter = 0;

        //Turn off vibration module
        digitalWrite(vibrationPowerPin, LOW);

        //Turn off indicator led
        digitalWrite(LED_BUILTIN, LOW);
      }
      
      break;

    case sending_location:

      //Check if there is a valid position to send
      if(!gpsPositionAvailable)
      {
        //Position unavailable, check if gps module is turned on
        if(!gpsEnabled)
        {
          //Module turned off, enable GPS module to obtain new position
          digitalWrite(gpsModulePowerPin, HIGH);

          //Update flag
          gpsEnabled = true;

          //Debug message
          Serial.println(F("GPS_MODULE -> ON (WAITING FOR FIX)"));
          Serial.flush();
        }
                  
        //Change state, waiting for gps fix
        current_state = waiting_for_gps;
      }
      else if(!gsmNetworkAvailable) //Check if GSM module is registered to network
      {
        //GSM Network unavailable, check if gps module is turned on
        if(!gsmEnabled)
        {
          //Module turned off, enable GSM module
          digitalWrite(gsmModulePowerPin, HIGH);

          //Update flag
          gsmEnabled = true;

          //Debug message
          Serial.println(F("GSM_MODULE -> ON (WAITING FOR REGISTRATION)"));
          Serial.flush();

          //Initialize module comunication
          gsmModule.begin(9600);
          gsmModule.setTimeout(3000);
          gsmModule.cmdBenchmark(true);
          delay(500);
        }
                  
        //Change state, waiting for gsm network registration
        current_state = waiting_for_gsm;
      }
      else
      {
        //GSM AND GPS AVAILABLE, send data
        Serial.println(F("Sending location SMS"));

        //Build SMS message
        char sms[50] = "InteliTracker - Location data";
        char coordinates[10];
        
        //Convert latitude to char
        dtostrf(fix.latitude(), 0, 7, coordinates);       

        //Append latitude data
        strcat(sms, "\nLat.: ");
        strcat(sms, coordinates);       
        
        //Convert longitude to char
        dtostrf(fix.longitude(), 0, 7, coordinates);       

        //Append latitude data
        strcat(sms, "\nLng.: ");
        strcat(sms, coordinates);

        //Send SMS
        gsmModule.sendSMS("\"+5567981977017\"", sms);

        //Check if SMS Sent successfully
        if(gsmModule.reply("OK"))
        {
          //Location sent using SMS, turn off GSM module
          digitalWrite(gsmModulePowerPin, LOW);
         
          //Debug data
          Serial.println(F("GSM_MODULE -> OFF (SMS SENT)"));
          
          //Update flags (position already sent, no longer available)
          gpsPositionAvailable = false;
          gsmNetworkAvailable = false;
          gsmEnabled = false;
          
          //GPS position obtained, send location
          current_state = sleeping;
        }
        else if(gsmModule.reply("+CMTI:"))
        {
          //Debug data
          Serial.println(F("GSM_MODULE -> NO CREDIT TO SEND SMS"));
          
          //Location sent using SMS, turn off GSM module
          digitalWrite(gsmModulePowerPin, LOW);
         
          //Debug data
          Serial.println(F("GSM_MODULE -> OFF (SMS SENT)"));
          
          //Update flags (position already sent, no longer available)
          gpsPositionAvailable = false;
          gsmNetworkAvailable = false;
          gsmEnabled = false;
          
          //GPS position obtained, send location
          current_state = sleeping;
        }
        else
        {
          //Debug data
          Serial.println(F("GSM_MODULE -> WAITING FOR NETWORK (SMS ERROR)"));
          
          //Wait for GSM registration again
          current_state = waiting_for_gsm;
        }
      }
      break;       

    case waiting_for_gps:

      //Check for how long is reading GPS data (awake status)
      if(millis() - awakeTimer < maxReadingTime)
      {
        //Use LED to indicate reading time
        digitalWrite(LED_BUILTIN, HIGH);
        
        //TODO: TIMEOUT
        if(gps.available(gpsPort))
        {
          //Read GPS data
          fix = gps.read();
  
          //If gps obtained a valid location
          if (fix.valid.location) 
          {
            //Turn off GPS module
            digitalWrite(gpsModulePowerPin, LOW);
  
            //Update flags (position already obtained, no need to keep module ON)
            gpsEnabled = false;
  
            //Update flag
            gpsPositionAvailable = true;
                             
            //Turn off indicator LED
            digitalWrite(LED_BUILTIN, LOW);
            
            Serial.println(F("GPS_MODULE -> OFF (VALID LOCATION OBTAINED)"));
            
            //GPS position obtained, send location
            current_state = sending_location;
          }
        }
      }
      else
      {
        //No fix available yet, turn off LED
        digitalWrite(LED_BUILTIN, LOW);

        //Enter sleep for 8 seconds
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

        //Awake from sleep, reset timer
        awakeTimer = millis();
      }
      
      break;

     case waiting_for_gsm: //TODO: TIMEOUT

        //Send command to check network registration
        gsmModule.send(GET, "CREG");

        //Check module (",1" -> Registered / ",5" -> Registered, roaming / ",6" Registered, SMS only)
        if (gsmModule.reply(",1") || gsmModule.reply(",5") || gsmModule.reply(",6"))
        {
          //Update flag, network avaliable
          gsmNetworkAvailable = true;
          
          Serial.println(F("GSM_MODULE -> REGISTERED TO NETWORK"));
          
          //Change state, ready to send location
          current_state = sending_location;
        }
        else
        {
          //Debug data
          Serial.println(F("GSM_MODULE -> NOT REGISTERED YET..."));
          Serial.flush();
            
          // Enter power down state for 8 s with ADC and BOD module disabled
          LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
        }
        break;
  }
}
