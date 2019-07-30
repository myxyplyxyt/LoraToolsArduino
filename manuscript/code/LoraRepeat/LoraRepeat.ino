// LoRaRepeat
// -*- mode: C++ -*-
// Example sketch which waits until it receives a compatible
// LoRa packet, then waits 100 msec, and re-sends the same packet.
//
#include "RH_RF95.h"
#include <SPI.h>

#define DEBUG    // comment out to use without Arduino environment 

#ifndef DEBUG
#define Serial Serial1
#endif

/*
 * * * * * * * * * * * * NOTICE * * * * * * * * * * * * * * 
 * This sketch requires that the manifest constant RH_RF95_HEADER_LEN in your
 * Local RADIO_HEAD library's RH_RF95.h header file be set to 0.
 * * * * * * * * * * * * * * * * * * * * *  * * * * * * * *
 */

/* for feather32u4 
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
*/

/* for feather m0 */ 
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3


/* for shield 
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 7
*/

/* Feather 32u4 w/wing
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     2    // "SDA" (only SDA/SCL/RX/TX have IRQ!)
*/

/* Feather m0 w/wing 
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     6    // "D"
*/

#if defined(ESP8266)
  /* for ESP w/featherwing 
  #define RFM95_CS  2    // "E"
  #define RFM95_RST 16   // "D"
  #define RFM95_INT 15   // "B"
  */
#elif defined(ESP32)  
  /* ESP32 feather w/wing
  #define RFM95_RST     27   // "A"
  #define RFM95_CS      33   // "B"
  #define RFM95_INT     12   //  next to A
  */
#elif defined(NRF52)  
  /* nRF52832 feather w/wing
  #define RFM95_RST     7   // "A"
  #define RFM95_CS      11   // "B"
  #define RFM95_INT     31   // "C"
  */  
#elif defined(TEENSYDUINO)
  /* Teensy 3.x w/wing 
  #define RFM95_RST     9   // "A"
  #define RFM95_CS      10   // "B"
  #define RFM95_INT     4    // "C"
  */
#endif

// Change to 434.0 or other frequency, must match TX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
  delay(100);

  Serial.println("SAMD Lora Feather Repeat Test!");

  // reset radio
  digitalWrite(RFM95_RST, LOW);
  digitalWrite(LED,HIGH);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(LED,LOW);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  delay(100);

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23,false);  // set high power
  rf95.setModeRx();

  // Defaults after init are 915.0MHz, 23dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
}

void loop()
{
   /*
    *  if LoRa input received, send it out .1 sec later
    */
    if(rf95.available()) {
      
         // Should be a message for us now
         uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
         uint8_t len = sizeof(buf);
         uint8_t indx = 0;
    
        if(rf95.recv(buf,&len)) {
             delay(100);
            rf95.send(buf,len);
/*
 * wait til transmit is complete
 * modeIsTx method added to RH_RF95 for this purpose. 
 */
            digitalWrite(LED, HIGH);
            delay(2);
            while(rf95.modeIsTx()) delay(1);
            digitalWrite(LED, HIGH);

        }
    }
}
