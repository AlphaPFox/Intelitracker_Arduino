#include <NMEAGPS.h>
#include <AltSoftSerial.h> 
#include <SoftwareSerial.h>
#include <LowPower.h>
#include <SIM800.h>

//Set the library used by NeoGPS to communicate with UBLOX GPS NEO 6M Module
#define GPS_PORT_NAME "AltSoftSerial"

//-------------- GSM serial comunication  --------------------
// Arduino pin 6 (TX) -> Connected to RX pin of the SIM800 module
// Arduino pin 7 (RX) -> Connected to TX pin of the SIM800 module
#define SIM800_RX_PIN  6
#define SIM800_TX_PIN  7

//-------------- GPS serial comunication  --------------------
// Arduino pin 8 (RX) -> Connected to GPS module TX
// Arduino pin 9 (TX) -> Cconnected to GPS module RX
AltSoftSerial gpsPort; 

//-------------- NeoGPS library objects ----------------------
//  gps -> This object parses received characters into the gps.fix() data structure 
//  fix -> Define a set of GPS fix information, hold on to the various pieces as they are received from an RMC sentence. 
static NMEAGPS gps;
static gps_fix fix;

//------------------ Boolean flags ---------------------------
// gpsModuleEnabled     -> GPS module status (ON/OFF)
// gsmModuleEnabled     -> GSM/GPRS module status (ON/OFF)
// gpsPositionAvailable -> True if a GPS position is available (and not sent yet)
// gsmNetworkAvailable  -> Indicates whether GSM module is registered on mobile network
// gsmLocationEnabled   -> Check if should use GSM data to obtain location instead of GPS

bool gpsModuleEnabled = false;
bool gpsPositionAvailable = false;
bool gsmModuleEnabled = false;
bool gsmNetworkAvailable = false;
bool gsmLocationEnabled = false;
bool gprsConnectionEnabled = false;
bool periodicUpdateEnabled = true;
bool vibrationCheckEnabled = true;
bool lastVibrationStatus = false;
bool vibrationDetected = false;
bool gsmInterruptAvailable = false;

//------------------ Pinout section --------------------------
// vibrationPowerPin -> Turn on and off SW-420 vibration module (connected directly to module VIN)
// gpsModulePowerPin -> Turn on and off UBLOX NEO-6M GPS module (connected to a NPN transistor)
// gsmModulePowerPin -> Turn on and off SIM 800L module (also connected to a NPN transistor)
// vibrationInputPin -> Read data from SW-420 digital ouput pin (HIGH if vibration detected)

byte vibrationPowerPin  = 2;
byte gsmInterruptPin    = 3;
byte gpsModulePowerPin  = 4;
byte vibrationInputPin  = 5;
byte gsmModulePowerPin  = 10;

//----------- Timer section (milliseconds unit) ---------------
// sensorReadingTimeout   -> Max time reading data from SW-420 and from NEO-6M GPS modules before going back to sleep
// gpsLocationTimeout     -> Max time waiting for GPS module to obtain current location (FIX)
// gsmRegistrationTimeout -> Max time waiting for GSM module to register on mobile network
// locationUpdateInterval -> Interval between sending device locations (using SMS/GPRS)
// vibrationCheckInterval -> Vibration sensor check periodicity
// sleepCicleCounter      -> Vibration sensor check periodicity

unsigned long sensorReadingTimeout = 1000;
unsigned long gpsLocationTimeout = 18000;
unsigned long gsmRegistrationTimeout = 180000;
unsigned long locationUpdateInterval = 1500000;
unsigned long vibrationCheckInterval = 15000;
unsigned long waitingForLocation = 0;
unsigned long waitingForNetwork  = 0;
unsigned long lastVibrationCheck = 0;
unsigned long lastLocationUpdate = 0;
unsigned long upTime = 0;
unsigned long awakeTimer = 0;

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

  //GSM interrupt pin
  pinMode(gsmInterruptPin, INPUT);

  //GPS and vibration off by default
  digitalWrite(vibrationPowerPin, LOW);
  digitalWrite(gpsModulePowerPin, LOW);

  //GSM module is on by default (on sleep mode)
  digitalWrite(gsmModulePowerPin, LOW);
  
  //Begin serial communication with Arduino and Hardware Serial (DEBUG)
  Serial.begin(9600);
  
  //Begin serial communication with Arduino and UBLOX NEO 6M GPS
  gpsPort.begin(9600);

  //Begin serial communication with Arduino and SIM800L GSM
  gsmModule.begin(9600);

  //Setup finished, go to sleep by default
  current_state = sleeping;

  //Initialize uptime
  upTime = millis();
  
  // Allow wake up pin (D3 - IN1) to trigger interrupt on Falling edge.
  attachInterrupt(digitalPinToInterrupt(gsmInterruptPin), gsmInterrupt, FALLING);
}

void loop() {
      
  switch(current_state)
  {   
    case sleeping:

      // Check if any GSM interrupt happened before last sleeping cicle
      if(gsmInterruptAvailable)
      {
        //Make sure module is not in sleep mode
        gsmModule.wakeUp();

        //Check module response
        if(gsmModule.reply("ING"))
        {
          //Wait for 8 seconds before ending call
          powerDown(SLEEP_8S);
          
          //Hang up call
          gsmModule.send(P("ATH"));
          
          //Debug data
          Serial.println(F("Call received, sending response..."));
        
          //Change state to read_vibration
          current_state = sending_location;
        }
        else
        {
          //Call method to check any new SMSs
          readSMS();
        }

        //Reset flag
        gsmInterruptAvailable = false;
      }
      // Before going to sleep, check if should execute vibration detection
      else if(vibrationCheckEnabled && upTime - lastVibrationCheck > vibrationCheckInterval)
      {
        //Debug data
        Serial.println(F("Awake, checking for vibration..."));

        //Enable vibration module
        digitalWrite(vibrationPowerPin, HIGH);

        //Sleep for 500 ms before reading sensor
        powerDown(SLEEP_500MS);

        //Save initial vibration check time
        lastVibrationCheck = upTime;
      
        //Change state to read_vibration
        current_state = reading_vibration;
      }
      // Check if shoud send a periodic location update
      else if(periodicUpdateEnabled && upTime - lastLocationUpdate > locationUpdateInterval)
      {
        //Debug data
        Serial.println(F("Awake, sending periodic location update..."));
      
        //Change state to read_vibration
        current_state = sending_location;
      }
      else
      {
        //Debug data
        Serial.print(F("Sleeping... Uptime: "));
        Serial.print((float) upTime / 60000, 1);
        Serial.println(F(" minutes"));

        // No activity required, enter power down state for 8s with ADC and BOD module disabled
        powerDown(SLEEP_8S);
      }

      //Finish state
      break;

    case reading_vibration:

      //Read initial sensor state (will change state if vibration detected)
      lastVibrationStatus = digitalRead(vibrationInputPin);

      //While vibration not detected (and not timeout)
      while(!vibrationDetected && upTime - lastVibrationCheck < sensorReadingTimeout)
      {
        //Check if vibration is detected
        if(lastVibrationStatus != digitalRead(vibrationInputPin))
        {
          //Update flag 
          vibrationDetected = true;
        }
        else
        {
          //Store last vibration status
          lastVibrationStatus = digitalRead(vibrationInputPin);
      
          //Use led to indicate reading time
          digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

          //Sleep for 30 ms before reading again
          powerDown(SLEEP_30MS);
        }
      }

      //Turn off indicator led
      digitalWrite(LED_BUILTIN, LOW);

      //Turn off vibration module
      digitalWrite(vibrationPowerPin, LOW);

      //Save current vibration check time
      lastVibrationCheck = upTime;

      //Check if vibration detecetd
      if(vibrationDetected)
      {
        //Vibration detected, send position
        current_state = sending_location;
      }
      else
      {
        //No vibration detected, going back to sleep
        current_state = sleeping;
      }
      break;

    case waiting_for_gps:

      //Check for how long is reading GPS data (awake status)
      if(millis() - awakeTimer < sensorReadingTimeout)
      {
        //Use LED to indicate reading time
        digitalWrite(LED_BUILTIN, HIGH);
        
        //Check if GPS data is available
        if(gps.available(gpsPort))
        {
          //Read GPS data
          fix = gps.read();
  
          //If gps obtained a valid location
          if (fix.valid.location) 
          {
            //Debug data
            Serial.println(F("GPS_MODULE -> OFF (VALID LOCATION OBTAINED)"));
            
            //Turn off GPS module
            digitalWrite(gpsModulePowerPin, LOW);
  
            //Update flags (position already obtained, no need to keep module ON)
            gpsModuleEnabled = false;
  
            //Update flag
            gpsPositionAvailable = true;
            
            //Turn off indicator LED
            digitalWrite(LED_BUILTIN, LOW);

            //Save how much time used to obtain GPS fix
            waitingForLocation = upTime - waitingForLocation;
            
            //GPS position obtained, send location
            current_state = sending_location;
          }
        }
      }
      //Check if reached timeout while waiting for gps location fix
      else if(upTime - waitingForLocation > gpsLocationTimeout)
      {
        //Debug data
        Serial.println(F("GPS_MODULE -> OFF (TIMEOUT ERROR)"));
        
        //Turn off GPS module
        digitalWrite(gpsModulePowerPin, LOW);

        //Update flags (timeout reached, no need to keep module ON)
        gpsModuleEnabled = false;
        gpsPositionAvailable = false;   
        gsmLocationEnabled = true;

        //Go to GSM state without GPS data available (uses GSM location)
        current_state = sending_location;
      }
      else
      {
        //No fix available yet, turn off LED
        digitalWrite(LED_BUILTIN, LOW);

        //Enter power down state for 8s
        powerDown(SLEEP_8S);

        //Debug data
        Serial.print(F("Waiting GPS fix for: "));
        Serial.print((upTime - waitingForLocation) / 1000);
        Serial.println(F(" seconds"));
      }
      
      break;

    case waiting_for_gsm: 

        //Wake module from sleep mode
        gsmModule.wakeUp();
        
        //Send command to check network registration
        gsmModule.send(GET, P("CREG"));

        //Check if reached timeout while waiting for gsm network registration
        if(upTime - waitingForNetwork > gsmRegistrationTimeout)
        {
          //Could not register network, timeout
          digitalWrite(gsmModulePowerPin, LOW);
         
          //Debug data
          Serial.println(F("GSM_MODULE -> OFF (TIMEOUT ERROR)"));
          
          //Update flags (failed to send location)
          gpsPositionAvailable = false;
          gsmNetworkAvailable = false;
          vibrationDetected = false;
          gsmModuleEnabled = false;
          
          //Save current update time
          lastLocationUpdate = upTime;
          
          //GPS position obtained, send location
          current_state = sleeping;
        }
        //Check network status (",1" -> Registered / ",5" -> Registered, roaming / ",6" Registered, SMS only)
        else if (gsmModule.reply(",1") || gsmModule.reply(",5") || gsmModule.reply(",6"))
        {
          //Check if there is any pending SMS
          readSMS();
              
          //Check if should connect to GPRS connection
          if(gprsConnectionEnabled)
          {
            //Check if already connected
            gsmModule.send(SET, P("SAPBR"), P("2,1"));

            //Check response
            if(gsmModule.getBuffer()[25] != '1')
            {          
              //Initialize GPRS connection
              gsmModule.send(SET, P(" "), P("3,1,\"Contype\",\"GPRS\""));
              gsmModule.send(SET, P("SAPBR"), P("3,1,\"APN\",\"zap.vivo.com.br\""));
              gsmModule.send(SET, P("SAPBR"), P("1,1"));

              //Wait to obtain connection
              powerDown(SLEEP_1S);

              //Check connection again
              break;
            }
          }

          //Connected to network, set as available
          gsmNetworkAvailable = true;

          //Debug data
          Serial.println(F("GSM_MODULE -> REGISTERED TO NETWORK"));

          //Save how much time needed to register on mobile network
          waitingForNetwork = upTime - waitingForNetwork;
          
          //Change state, ready to send location
          current_state = sending_location;
        }
        //Check if confirmed SMS sent after previous timeout
        else if (gsmModule.reply("CMGS"))
        {
          //Location sent using SMS, turn off GSM module
          digitalWrite(gsmModulePowerPin, LOW);
         
          //Debug data
          Serial.println(F("GSM_MODULE -> OFF (SMS SENT AFTER TIMEOUT)"));
          
          //Update flags (position already sent, no longer available)
          gpsPositionAvailable = false;
          gsmNetworkAvailable = false;
          gsmModuleEnabled = false;
          vibrationDetected = false;
          gsmLocationEnabled = false; // TODO: Suppress GPS option
                    
          //GPS position obtained, send location
          current_state = sleeping;
        }
        else
        {
          //Debug data
          Serial.println(F("GSM_MODULE -> NOT REGISTERED YET..."));
          
          // Enter power down state for 8s with ADC and BOD module disabled
          powerDown(SLEEP_8S);

          //Debug data
          Serial.print(F("Waiting GSM registration for: "));
          Serial.print((upTime - waitingForNetwork) / 1000);
          Serial.println(F(" seconds"));
        }
        
        break;

    case sending_location:
    
      //Check if should obtain GPS location (gsmLocation = false) and no location available yet
      if(!gsmLocationEnabled && !gpsPositionAvailable)
      {
        //Location unavailable, check if gps module is turned on
        if(!gpsModuleEnabled)
        {
          //Module turned off, enable GPS module to obtain new position
          digitalWrite(gpsModulePowerPin, HIGH);

          //Update flag
          gpsModuleEnabled = true;

          //Debug message
          Serial.println(F("GPS_MODULE -> ON (WAITING FOR FIX)"));
        }
                  
        //Change state, waiting for gps fix
        current_state = waiting_for_gps;

        //Save when waiting started
        waitingForLocation = upTime;
      }
      else if(!gsmNetworkAvailable) //Check if GSM module is registered to network
      {
        //GSM Network unavailable, check if gps module is turned on
        if(!gsmModuleEnabled)
        {
          //Module turned off, enable GSM module
          digitalWrite(gsmModulePowerPin, HIGH);

          //Update flag
          gsmModuleEnabled = true;

          //Debug message
          Serial.println(F("GSM_MODULE -> ON (WAITING FOR REGISTRATION)"));

          //Initialize module comunication
          gsmModule.begin(9600);
        }
                  
        //Change state, waiting for gsm network registration
        current_state = waiting_for_gsm;

        //Store when step started
        waitingForNetwork = upTime;
      }
      else
      {
        //Initialize array containing SMS text
        char sms[160], batteryPercent[5] = "", batteryVoltage[7] = "";

        //Check if sending data after vibration detection
        if(vibrationDetected)
        {
          //Vibration detected SMS
          sprintf_P(sms, P("InteliTracker - Vibration detected"));
        }
        else
        {
          //Location update SMS
          sprintf_P(sms, P("InteliTracker - Location update"));
        }

        //Update battery values
        getBatteryStatus(batteryPercent, batteryVoltage);
        
        //If sending position obtained from GPS module
        if(gpsPositionAvailable)
        {
          //GSM AND GPS AVAILABLE, send data
          Serial.println(F("Sending location SMS with GPS location data"));
          Serial.println(sms);
          
          //Array for GPS coordinates
          char latitude[12], longitude[12];
  
          //Convert coordinates to string
          dtostrf(fix.longitude(), 10, 7, longitude);
          dtostrf(fix.latitude(), 10, 7, latitude);
          
          //Build SMS message
          sprintf_P(sms + strlen(sms), P("\nBat.: %s, %s\nLat.: %s\nLng.: %s\nDT.: %02d/%02d/%02d - %02d:%02d:%02d UTC\nGSM registration: %lu ms\nGPS location: %lu ms"), 
             batteryVoltage, batteryPercent, latitude, longitude, fix.dateTime.day, fix.dateTime.month, fix.dateTime.year, fix.dateTime.hours, fix.dateTime.minutes, fix.dateTime.seconds, waitingForNetwork, waitingForLocation);
        }
        else
        {
          //Debug data
          Serial.println(F("Sending location SMS with GSM location data"));

          //Initialize SMS content
          sprintf_P(sms + strlen(sms), P("\nBat.: %s, %s\nGSM Location - MCC,MNC,LAC,CID:"), batteryVoltage, batteryPercent);
          
          //GSM location required, send command to get data from network cells
          gsmModule.send(SET, P("CENG"), P("3,0"));
          gsmModule.send(GET, P("CENG"));

          //Pattern used to search gsmResponse
          char searchPattern[10];

          //Search for up three network cells
          for(int i = 0; i < 3; i++)
          {
            //Build search pattern for the current index
            sprintf_P(searchPattern, P("+CENG: %d,"), i);
        
            //Search for that index on gsmResponse buffer
            char *pointer = strstr(gsmModule.getBuffer(), searchPattern);
        
            //If available and not empty (,)
            if(pointer != 0 && pointer[32] == '"')
            {
              //Append newline
              strcat_P(sms, P("\nCELL ID: "));
              
              //Append GSM data to SMS text
              strncat(sms, &pointer[10], 22);
            }
          }
        }
        
        //Send SMS
        gsmModule.sendSMS("+5567981977017", sms);

        //Save current update time
        lastLocationUpdate = upTime;

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
          gsmModuleEnabled = false;
          vibrationDetected = false;
          gsmLocationEnabled = false; // TODO: Suppress GPS option
                    
          //GPS position obtained, send location
          current_state = sleeping;
        }
        else if(gsmModule.reply("+CMTI:"))
        {
          //Debug data
          Serial.println(F("GSM_MODULE -> OFF (NO CREDIT TO SEND SMS)"));
          
          //Location sent using SMS, turn off GSM module
          digitalWrite(gsmModulePowerPin, LOW);
          
          //Update flags (failed to send position)
          gpsPositionAvailable = false;
          gsmNetworkAvailable = false;
          gsmModuleEnabled = false;
          vibrationDetected = false;
          gsmLocationEnabled = false; // TODO: Suppress GPS option
          
          //Location update failed, go back to sleep
          current_state = sleeping;
        }
        else
        {
          //Debug data
          Serial.println(F("GSM_MODULE -> WAITING FOR NETWORK (SMS ERROR)"));

          //Wait for GSM registration again
          current_state = waiting_for_gsm;

          //Store when step started
          waitingForNetwork = upTime;
        }
      }
      break;       
  }
}

void gsmInterrupt()
{
  //Indicate received SMS
  digitalWrite(HIGH, LED_BUILTIN);

  //Wait 1s for AT response
  powerDown(SLEEP_1S);

  //Update flag
  gsmInterruptAvailable = true;
}

//Put Arduino to sleep and increment upTime accordingly
void powerDown(period_t sleepTime)
{
  //Increment how much time is awake to uptime
  upTime += millis() - awakeTimer;
  
  //Finish any pending serial communications
  Serial.flush();
  
  //Call method to sleep
  LowPower.powerDown(sleepTime, ADC_OFF, BOD_OFF);

  //After sleep, increment time
  switch(sleepTime)
  {
    case SLEEP_30MS:
      upTime+= 30;
      break;
    case SLEEP_60MS:
      upTime+= 60;
      break;
    case SLEEP_120MS:
      upTime+= 120;
      break;
    case SLEEP_250MS:
      upTime+= 250;
      break;
    case SLEEP_500MS:
      upTime+= 500;
      break;
    case SLEEP_1S:
      upTime+= 1000;
      break;
    case SLEEP_2S:
      upTime+= 2000;
      break;
    case SLEEP_4S:
      upTime+= 4000;
      break;
    case SLEEP_8S:
      upTime+= 8000;
      break;
  }

  //Reset awake timer
  awakeTimer = millis();
}

void readSMS()
{
  //List all stored SMS
  gsmModule.send(SET,P("CMGL"), P("\"ALL\""));

  //Check if any new SMS arrived
  if(gsmModule.reply("CMGL:"))
  {
    //Index used to search char arrays
    int index;
    
    //Store sender phone number and command
    char smsIndex[5], phoneNumber[20], command[500], gsmBuffer[DEF_BUFFER_SIZE];

    //Copy response from GSM module (required because gsmModule buffer may change during process)
    strcpy(gsmBuffer, gsmModule.getBuffer());
    
    //Pattern used to search gsmResponse
    char *beginText = strstr(gsmBuffer, "+CMGL:");

    //If SMS content fully available (search for next message CMGL or command end OK)
    while (beginText != 0 && (strstr(beginText, "\n+CMGL") || strstr(beginText, "OK")))
    {
      //Increment point to match sender phone number first char
      beginText += 7;   

      //SMS index ends with , char
      for(index = 0; beginText[index] != ','; index++)
      {
        //Extract SMS index
        smsIndex[index] = beginText[index];
      }

      //Append terminator to char array
      smsIndex[index] = '\0';
            
      //Search for another message
      beginText = strstr(beginText, "READ") + 7;

      //Phone number ends with " char
      for(index = 0; beginText[index] != '"'; index++)
      {
        //Extract phone number from modem response
        phoneNumber[index] = beginText[index];
      }

      //Append terminator to char array
      phoneNumber[index] = '\0';

      //Search for initial SMS content char
      beginText = strstr(beginText, "-") + 6;

      //Phone number ends with " char
      for(index = 0; beginText[index] != '\n'; index++)
      {
        //Extract command from SMS text
        command[index] = beginText[index];
      }
      
      //Append terminator to char array
      command[index] = '\0';
      
      //Parse SMS current content
      parseCommand(smsIndex, phoneNumber, command);

      //Search for next message (if available)
      beginText = strstr(beginText, "+CMGL:");
    }
  }
}

void parseCommand(char* smsIndex, char* phoneNumber, char* command)
{
  if (strncmp_P (command, P("TestCommand"), 11) == 0) 
  {
    //Debug data
    Serial.println(F("TestCommand SMS Received"));
    
    //Send SMS
    gsmModule.sendSMS(phoneNumber, "TestResponse");

    if(gsmModule.reply("OK"))
    {
      //Debug data
      Serial.println(F("TestResponse SMS Sent, erasing SMS"));

      //Erase SMS
      gsmModule.deleteSMS(smsIndex);
    }
  }
  else
  {
    //Debug data
    Serial.println(F("Unknown command... erasing SMS"));
    Serial.println(command);
    
    //Erase SMS
    gsmModule.deleteSMS(smsIndex);
  }
}

void getBatteryStatus(char percent[], char voltage[])
{
  //Ask for battery status
  gsmModule.send(EXE,P("CBC"), P(""));

  //If response ok
  if(gsmModule.reply("OK"))
  {
    //Initialize index
    int i = 0;
    
    //Find initial response position
    char *beginText = strstr(gsmModule.getBuffer(), "+CBC:") + 8;

    //Copy data from response
    for(i = 0; beginText[i] != ','; i++)
    {
      //Copy variable length 
      percent[i] = beginText[i];
    }

    //Copy voltage text
    strncpy(voltage, beginText + i + 1, 4);
    
    //Add space to voltage array
    memmove(voltage + 1, voltage, 4);

    //Format data (add '.' to voltage value, and null terminators);
    voltage[1] = '.';

    //Format values
    strcat(percent, "%");
    strcat(voltage, "V");
  }
  else
  {
    //No battery info available
    sprintf_P(voltage, P("N/A"));
    sprintf_P(percent, P("N/A"));           
  }
}
