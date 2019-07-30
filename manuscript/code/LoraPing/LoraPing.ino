
// LoRaPing
// -*- mode: C++ -*-
// Periodically sends a ping out through
// the LoRa radio.
//
// If DEBUG is defined, Serial output 
// is directed to the Arduino Console, and the program
// will only run within connected to the Arduino environment.
// If DEBUG is not defined, the program will run on Battery
// without the Arduino Console, and serial IO will
// be pass through the RX/TX pins.
//
#include "RH_RF95.h"
#include <SPI.h>

//#define DEBUG    // comment out to use without Arduino environment 

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

/* for arduino feather SAMD m0 */ 
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define LED   13

#if 0
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
#endif


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

#define PING_LENGTH 42

static uint8_t pingdex=0;
static char *ptr=0;

static char *ping_array[PING_LENGTH]= {
    "PING ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890\12",
    "PING BCDEFGHIJKLMNOPQRSTUVWXYZ1234567890A\12",
    "PING CDEFGHIJKLMNOPQRSTUVWXYZ1234567890AB\12",
    "PING DEFGHIJKLMNOPQRSTUVWXYZ1234567890ABC\12",
    "PING EFGHIJKLMNOPQRSTUVWXYZ1234567890ABCD\12",
    "PING FGHIJKLMNOPQRSTUVWXYZ1234567890ABCDE\12",
    "PING GHIJKLMNOPQRSTUVWXYZ1234567890ABCDEF\12",
    "PING HIJKLMNOPQRSTUVWXYZ1234567890ABCDEFG\12",
    "PING IJKLMNOPQRSTUVWXYZ1234567890ABCDEFGH\12",
    "PING JKLMNOPQRSTUVWXYZ1234567890ABCDEFGHI\12",
    "PING KLMNOPQRSTUVWXYZ1234567890ABCDEFGHIJ\12",
    "PING LMNOPQRSTUVWXYZ1234567890ABCDEFGHIJK\12",
    "PING MNOPQRSTUVWXYZ1234567890ABCDEFGHIJKL\12",
    "PING NOPQRSTUVWXYZ1234567890ABCDEFGHIJKLM\12",
    "PING OPQRSTUVWXYZ1234567890ABCDEFGHIJKLMN\12",
    "PING PQRSTUVWXYZ1234567890ABCDEFGHIJKLMNO\12",
    "PING QRSTUVWXYZ1234567890ABCDEFGHIJKLMNOP\12",
    "PING RSTUVWXYZ1234567890ABCDEFGHIJKLMNOPQ\12",
    "PING STUVWXYZ1234567890ABCDEFGHIJKLMNOPQR\12",
    "PING TUVWXYZ1234567890ABCDEFGHIJKLMNOPQRS\12",
    "PING UVWXYZ1234567890ABCDEFGHIJKLMNOPQRST\12",
    "PING VWXYZ1234567890ABCDEFGHIJKLMNOPQRSTU\12",
    "PING WXYZ1234567890ABCDEFGHIJKLMNOPQRSTUV\12",
    "PING XYZ1234567890ABCDEFGHIJKLMNOPQRSTUVW\12",
    "PING YZ1234567890ABCDEFGHIJKLMNOPQRSTUVWX\12",
    "PING Z1234567890ABCDEFGHIJKLMNOPQRSTUVWXY\12",
    "PING 1234567890ABCDEFGHIJKLMNOPQRSTUVWXYZ\12",
    "PING 234567890ABCDEFGHIJKLMNOPQRSTUVWXYZ1\12",
    "PING 34567890ABCDEFGHIJKLMNOPQRSTUVWXYZ12\12",
    "PING 4567890ABCDEFGHIJKLMNOPQRSTUVWXYZ123\12",
    "PING 567890ABCDEFGHIJKLMNOPQRSTUVWXYZ1234\12",
    "PING 67890ABCDEFGHIJKLMNOPQRSTUVWXYZ12345\12",
    "PING 7890ABCDEFGHIJKLMNOPQRSTUVWXYZ123456\12",
    "PING 890ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567\12",
    "PING 90ABCDEFGHIJKLMNOPQRSTUVWXYZ12345678\12",
    "PING 0ABCDEFGHIJKLMNOPQRSTUVWXYZ123456789\12",
 };


void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);

  delay(100);

  Serial.begin(9600);
  while(!Serial) {
    delay(1);
  }

  Serial.println("Feather LoRa Ping App!");

  // manual radio reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 915.0MHz, 
  // modulation LoRa_Bw125Cr45Sf128 (the chip default), 
  // power output +20dbM
  
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 915.0MHz, 20dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  rf95.setTxPower(20, false);
}

void loop()
{
  ptr=ping_array[pingdex++];
  if(pingdex>35) pingdex=0;
  Serial.print(ptr); 
  delay(10);
  digitalWrite(LED,HIGH);
  rf95.send((uint8_t *)ptr, 42);
  delay(3);
  rf95.waitPacketSent();
  digitalWrite(LED,LOW);
  delay(5000); // Wait 1 second between transmits, could also 'sleep' here!
}
