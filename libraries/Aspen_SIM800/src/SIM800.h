
//   "Aspen SIM800" is a comprehensive SIM800 library for simplified and in-depth chip access.
//   Copyright (C) 2016  Mattias Aabmets (https://github.com/aspenforest)
//
//   This API library is free software: you can redistribute it and/or modify
//   it under the terms of the GNU Affero General Public License as published
//   by the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
//   This API library is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
//   See the GNU Affero General Public License for more details.
//
//   You should have received a copy of the GNU Affero General Public License
//   along with this API library.  If not, see <http://www.gnu.org/licenses/>.


#ifndef SIM800_h
#define SIM800_h

#include <Arduino.h>
#include <avr/pgmspace.h>                       // All AT commands are stored in PROGMEM to unburden the SRAM and to make it possible to implement all 336 of them.
#define P(str) PSTR(str)                        // Helper definition. P() does the same for char[] what F() does for String objects (stores them in PROGMEM).

#define DEBUG                                   // Uncomment this line to enable Serial Monitor output from this library.

#define DEF_SPEED   9600                       // The default baudrate which is used if the user calls "void bauds(unsigned long)" (in simserial.h) without arguments.
#define DEF_TIME_LIMIT  1000                    // The default max time duration in milliseconds to wait for a reply from the SIM800 chip.
#define DEF_BUFFER_SIZE  300                    // Size of the char[] ioBuffer which is used to buffer outgoing commands and incoming replies.
#define SIM800_RX_PIN  6
#define SIM800_TX_PIN  7

enum CmdType {TEST, GET, SET, EXE};             // Valid cmd types for AT cmd wrapper methods. Most AT methods don't support all CmdTypes. User must consult with official AT cmd manual.

class SIM800 {
    #ifdef DEBUG 
    public: enum DebugCaller {BEGIN, REPLY};    // Debug helper type. Used to select debug method serial monitor output.
    #endif

    // CORE
    #include <core/simserial.h>                 // Contains core SoftwareSerial communication with SIM800 module functionality.
    #include <core/strfn.h>                     // Contains char[] manipulation functions which work with outgoing commands and incoming replies.
    #include <core/debug.h>                     // Contains the main Serial Monitor output functionality. This is compiled only if #define DEBUG is uncommented in this file.
         
}; extern SIM800 gsmModule;                           // Instantiate a single SIM800 object in SIM800.cpp on which all methods of this library must be called. Example: SIM.test();

#endif