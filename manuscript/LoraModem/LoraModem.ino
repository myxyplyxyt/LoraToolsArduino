#include "led.h"
#include "io.h"

void setup(void)
{

  ledInit();

  ioInit();

  ledOff();

}

void loop(void) {

/*
*  Manage the transfer of lora input to the Uart for output,
*  and the transfer of Uart input to the lora radio for output.
 */
   ioStateMachine();
}

void ledInit(void)
{
    pinMode(LED,OUTPUT);
    digitalWrite(LED,HIGH);  
}

void ledOn(void)
{
    digitalWrite(LED,HIGH);
}

void ledOff(void)
{
    digitalWrite(LED,LOW);
}
