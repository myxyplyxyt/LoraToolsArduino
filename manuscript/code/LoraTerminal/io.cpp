/*
 * io.cpp -- manage LoRa and Serial input and output
 *
 * MIT License
 *
 * Copyright (c) 2019 David L Clifton
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <Arduino.h>
#include "RH_RF95.h"    // RH_RF95 module and header modified
                        // bool modeIsTx method added, RH_RF95_HEADER_LEN set to zero
                        // and validateRxBuf modified to accept buffers without headers.
                        // if addresses and flags needed, they can be added with comma
                        // separated text fields
                        // (io.cpp works only with non-zero characters)
#include <SPI.h>
#include "led.h"
#include "loraio.h"
#include "serialio.h"
#include "io.h"
/* *************************************** 
 *  Serial1 definitions
 * ***************************************/
 /* Arduino Serial port serves the arduino
    serial console terminal */


/* ***************************************
 * loRa radio definitions and declarations
 * ***************************************/
/* radio pins for lora feather m0 */ 
#define RFM95_CS  8
#define RFM95_RST 4
#define RFM95_INT 3

 // singleton instance of a slightly modified
 // radiohead lora driver.  The files RH_RF95.h
 // and RH_RF95.cpp were modified to support
 // this program.  The RH_LORA_HEADER_LEN was
 // set to 0, the validateRx routine was modified
 // to eliminate separate header bytes, and
 // the bool modeIsTx() method was added to
 // simplify the handleIo routine.
 
RH_RF95 rf95(RFM95_CS, RFM95_INT);
#define RF95_FREQ 915.0

// Lora input and output buffer variables

byte inbuf[LORA_MAX_MESSAGE_LEN + 1];
uint8_t inlen = 0;
int indx;

byte outbuf[LORA_MAX_MESSAGE_LEN+1];
byte outlen = 0;
static int outdx;


static void radio95Init( void );
static void serialInit(void);
static void clrInbuf(void);
static void clrOutbuf(void);

static byte count;
static volatile unsigned int startime;

 /*
 * initialize BLE Uart and Lora Radio IO
 */
void ioInit( void )
{    
   serialInit();
   radio95Init();
   startime=0;
}
/*
 * Manage transfer of data between BLE Uart and LoRa Radio
 * There is no access to lora or serial fifos from interrupts
 */
void handleIo(void)
{
    int ch;
/*
 * ** if output ready for Arduino Serial console, write one byte and
 * ** wait until it is sent
  */
    if((ch=serialFifoPeek())>=0) {
            count= Serial.write((uint8_t)ch);
            if(count) serialFifoPop();
     }
/*
 *  ** if Serial console input is present, read it, and push
 *  ** it into loraOutput fifo
 */
   while(Serial.available()) {
        startime = millis();
        while((millis() - startime)<2000) {
            if((ch = Serial.read())>=0) {
                 loraFifoPush((uint8_t)ch);
                 startime = millis();
            }
        }
   }
   
   /*
    *  if LoRa received, send it to Serial Console
    *  via serialFifo
    */
    if(rf95.available()) {
        clrInbuf();
        while(rf95.recv(inbuf,&inlen)) {
             for (indx=0; indx<inlen;indx++) {
                serialFifoPush(inbuf[indx]);
            }
            clrInbuf();
            delay(5);
        }
    }
 /*
  *  ** if lora output is ready to send, transfer it
  *  ** into radio chip and trigger transmission.
  *  ** modeIsTx() method was added to rf95 to
  *  ** support soonest possible reception of
  *  ** of incoming lora data.
  */
  if ( loraFifoPeek()>=0 ) do {
        clrOutbuf();
        while((ch=loraFifoPop())>=0) {
            outbuf[outdx++]=ch;
            if(outdx>=LORA_MAX_MESSAGE_LEN || ch==10 || ch==13 ) {
                rf95.send(outbuf,outdx);
                while(rf95.modeIsTx()) delay(1);  // modeIsTx method added to rf95 for this purpose.
                delay(1);
                break;                
            }
        } // endwhile
        
    } while (loraFifoPeek()>=0);
}

/*
 * ** initialize the lora radio
 */
void radio95Init( void )
{    
 // setup lora radio
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  
// reset LoRa Radio chip
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  
  Serial.println("Radio Reset Done");
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
  Serial.print("Set Freq to: ");
  
  Serial.println(RF95_FREQ);

  rf95.setTxPower(23,false);  // set high power
    
  // Defaults after init are 915.0MHz, 20dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
   clrInbuf();
   clrOutbuf();
   loraFifoInit();
   rf95.setModeRx();
}

void serialInit( void )
{
    // initialize serial communication at 115200 bits per second:
    Serial.begin(115200);
    while(!Serial) {
        delay(1);
    }    
    serialFifoInit();    
}

/*
 * *** lora IO buffer clearing
 */
void clrInbuf(void)
{
    indx=0;
    inlen=LORA_MAX_MESSAGE_LEN;
    memset(inbuf,0,LORA_MAX_MESSAGE_LEN);
}

void clrOutbuf(void)
{
    outdx=0;
    memset(outbuf,0,LORA_MAX_MESSAGE_LEN);
}
