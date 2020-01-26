/*
 * loraio.c -- init lora radio and manage lora fifo.
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
#include "loraio.h"

/*
 * LoRa FIFO declarations
 */
#define LORA_FIFOSIZE 1023  // must be power of two minus 1
 
static uint8_t loraFifo[LORA_FIFOSIZE+1];
static volatile unsigned int fifoIn=0;
static volatile unsigned int fifoOut=0;

/*
 * ***  LoRa FIFO access
 */

 /*
 * Initializes the lora fifo
 */
void loraFifoInit( void )
{
    fifoIn=0;
    fifoOut = LORA_FIFOSIZE+1;
    while(fifoOut) {     
        loraFifo[--fifoOut]=0;
    }
}

unsigned int loraFifoBytes(void)
{
   return ((fifoIn - fifoOut) & LORA_FIFOSIZE);
}


boolean loraFifoEmpty(void)
{
    return (loraFifoBytes()==0);
}

boolean loraFifoFull(void)
{
  return ( loraFifoBytes()==LORA_FIFOSIZE);
}

/*
 * loraFifoPush
 * Put a single byte into the lora output fifo
 * return false if fifo is full
 * else return true
 */
uint8_t loraFifoPush(uint8_t ch)
{
    if(loraFifoFull()) return 0;
    loraFifo[fifoIn++]=ch;
    fifoIn &= LORA_FIFOSIZE;
    return 1;
}

/*
 * Obtains next character from the FIFO
 *
 * return
 * -1 if FIFO is empty or error encountered
 * next character from FIFO
 */
int loraFifoPop( void )
{
    int outbyte;
    
    if(loraFifoEmpty()) return -1;
    outbyte=(int)loraFifo[fifoOut++];
    fifoOut &= LORA_FIFOSIZE;
    return outbyte;
}

int loraFifoPeek( void )
{
    int outbyte;
    
    if(loraFifoEmpty()) return -1;
    outbyte=(int)loraFifo[fifoOut];
    return outbyte;
}
