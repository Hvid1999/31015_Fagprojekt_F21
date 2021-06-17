/*
Markus Hvid Monin (s194011)
Anders Eiersted Molzen (s194024)
Magnus Flemming Jørgensen (s194013)
DTU Elektro
31015 - Fagprojekt - F21
*/

#include "Adafruit_FONA.h"

#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
#define FONA_RI  7
#define moboRX1 3
#define moboRX2 6

//Disclaimer: Some of the code is based on example code from Adafruit.

// this is a large buffer for replies
char replybuffer[255];

// We default to using software serial.
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;


// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t type;

//Initialising variables
char PIN[4] = {'0', '0', '0', '0'};
char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
uint8_t imeiLen;
char timebuffer[23];
char sendto[21]    = "60837273"; //{'6', '0', '8', '3', '7', '2', '7', '3'};
char chargerID1[6]  = "#CHGR1"; //{'C', 'H', 'G', 'R', '1'};
char chargerID2[6]  = "#CHGR2"; //{'C', 'H', 'G', 'R', '2'};
char message[141];    // Buffer to hold GSM SMS message
int timeUnlocked = 0; // Time since succesful SIM unlock
int flag = 0;
int RX1state = 0;
int RX2state = 0;
int RX1old = 0;
int RX2old = 0;

char statusError[5] = "$ERR$"; //{'E', 'R', 'R'};
char statusVacant[5] = "$VAC$"; //{'V', 'A', 'C'};
char statusOccupied[5] = "$OCP$"; //{'O', 'C', 'P'};
char statusBooted[19] = "Succesfully booted";

// Transmission schedule
unsigned long timeinterval = 5000; // Time between message buffer updates.
unsigned long timeoffset    = 0;      // system millis() time at last transmission/status check.
unsigned long millisTransmit = 0;
unsigned long transmissionInterval = 180000; // Time between auto-transmission of message stored in buffer.

void setup() {
  pinMode(moboRX1, INPUT);
  pinMode(moboRX2, INPUT);
  Serial.begin(115200);


  //Initializing FONA unit
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }

  type = fona.type();
  Serial.print(F("Found board: "));
  switch (type) {
    case FONA800L:
      Serial.println(F("FONA 800L")); break;
    case FONA800H:
      Serial.println(F("FONA 800H")); break;
    case FONA808_V1:
      Serial.println(F("FONA 808 (v1)")); break;
    case FONA808_V2:
      Serial.println(F("FONA 808 (v2)")); break;
    case FONA3G_A:
      Serial.println(F("FONA 3G (American)")); break;
    case FONA3G_E:
      Serial.println(F("FONA 3G (European)")); break;
    default:
      Serial.println(F("???")); break;
  }

  // Print module IMEI number.
  imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }

  // enable network time sync. Do this before unlocking SIM.
  if (!fona.enableNetworkTimeSync(true)) {
    Serial.println(F("Failed to enable time sync"));
  } else {
    Serial.println(F("Network time sync enabled!"));
  }

  //Unlocking SIM
  if (! fona.unlockSIM(PIN)) {
    Serial.println(F("Failed to unlock SIM"));
  } else {
    Serial.println(F("OK! SIM unlocked."));
  }
  timeUnlocked = millis();
  timeoffset = millis();

  // hold µC idle for ten seconds to ensure proper SMS system boot.
  while ((millis() - timeUnlocked) < 10000) {
    if (fona.available()) {
      Serial.write(fona.read());
    }
  }

  //Send SMS on succesful boot
  strncpy(message, statusBooted, 19);
  if (!fona.sendSMS(sendto, message)) {
    Serial.println(F("Failed to send message."));
  } else {
    Serial.println(F("Message sent!"));
  }

}

void loop() {

  while (Serial.available()) {
    Serial.read();
  }
  while (fona.available()) {
    Serial.write(fona.read());
  }

  // Logic to store current state, to detect change on pins RX1 and RX2.
  RX1state = digitalRead(moboRX1);
  RX2state = digitalRead(moboRX2);


  if ((millis() - timeUnlocked) >= 10000) {
    //If 3 minutes have passed or the read state of the sensor system has changed, the message is updated and and SMS is sent
    if (((millis() - timeoffset) >= timeinterval) || ((RX1state != RX1old) || (RX2state != RX2old))) {
      timeoffset = millis();

      // Message structure:
      // byte 0-14     = time
      // byte 15-20    = "#XXXXX" ID denoted as "#" and five integers
      // byte 21       = "$" indicate start of data frame
      // byte 22-24    = status
      // byte 25       = "$" indicate end of data frame

      //Commands:
      //VAC - Vacant, no object detected
      //OCP - Occupied; object detected
      //ERR - Error, unspecified

      //Getting network time
      fona.getTime(timebuffer, 23);  // make sure replybuffer is at least 23 bytes!

      //Loading time information into the message
      for (int i = 1; i < 15; i++) {
        message[i - 1] = timebuffer[i];
      }

      strncpy(message + 14, chargerID1, 6);

      if (digitalRead(moboRX1) == HIGH) {
        strncpy(message + 20, statusOccupied, 5);
      } else {
        strncpy(message + 20, statusVacant, 5);
      }

      strncpy(message + 25, chargerID2, 6);

      if (digitalRead(moboRX2) == HIGH)  {
        strncpy(message + 31, statusOccupied, 5);
      } else {
        strncpy(message + 31, statusVacant, 5);
      }

      // send an SMS!
      Serial.write(message, 140);
      Serial.println();

      if ((millis() - millisTransmit) >= 5000) {
        if (((millis() - millisTransmit) >= transmissionInterval) || ((RX1state != RX1old) || (RX2state != RX2old))) {
          if (!fona.sendSMS(sendto, message)) {
            Serial.println(F("Failed to send message."));
          } else {
            Serial.println(F("Message sent!"));
          }
          millisTransmit = millis();
        }
      }
    }
  }
  RX1old = RX1state;
  RX2old = RX2state;
}
