#ifndef LORA_IO
#define LORA_IO

#define LORA_MAX_MSG_LEN 255
#define LORA_FIFOSIZE 1023

void loraFifoInit( void );
unsigned int loraFifoBytes(void);
boolean loraFifoEmpty(void);
boolean loraFifoFull(void);
uint8_t loraFifoPush(uint8_t ch);
int loraFifoPop(void);
int loraFifoPeek(void);

#endif  // LORA_IO
