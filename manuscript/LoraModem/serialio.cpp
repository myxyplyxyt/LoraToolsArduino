/*
 * serialio.cpp -- manage serial input and output
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
#include "led.h"
#include "serialio.h"

#define SERIAL_FIFOSIZE 1023    // must be power of two minus 1

static byte serialFifo[SERIAL_FIFOSIZE+1];
static volatile unsigned int fifoIn=0;
static volatile unsigned int fifoOut=0;

/*
 * ***  Serial FIFO access
 */
void serialFifoInit( void )
{
    fifoIn=0;
    fifoOut = SERIAL_FIFOSIZE+1;
    while(fifoOut) {     
        serialFifo[--fifoOut]=0;
    }
}

unsigned int serialFifoBytes(void)
{
   return ((fifoIn - fifoOut) & SERIAL_FIFOSIZE);
}

unsigned int serialFifoFree(void)
{
    return (SERIAL_FIFOSIZE - serialFifoBytes());
}

bool serialFifoEmpty(void)
{
    return (fifoIn==fifoOut);
}

bool serialFifoFull(void)
{
    return ( serialFifoBytes()==SERIAL_FIFOSIZE);
}

/*
 * serialFifoPush
 * Put a single character into the serial fifo
 * return 0 if fifo is full
 * else return 1
 */
uint8_t serialFifoPush(uint8_t ch)
{
    if(serialFifoFull()) return 0;
    serialFifo[fifoIn++]=ch;
    fifoIn &= SERIAL_FIFOSIZE;
    return 1;
}


/*
 * Obtain next character from the FIFO
 *
 * return
 * -1 if FIFO is empty or error encountered
 * next character from FIFO
 */
int serialFifoPop( void )
{
    int outbyte;
    
    if(serialFifoEmpty()) return -1;
    outbyte=(int)serialFifo[fifoOut++];
    fifoOut &= SERIAL_FIFOSIZE;
    return outbyte;
}

int serialFifoPeek( void )
{
    int outbyte;

    if(serialFifoEmpty()) return -1;
    outbyte=(int)serialFifo[fifoOut];
    return outbyte;
}
