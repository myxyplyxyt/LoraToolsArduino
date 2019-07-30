#ifndef LORA_IO
#define LORA_IO

#define LORA_MAX_MESSAGE_LEN 64
#define LORA_FIFOSIZE 255

void loraFifoInit( void );
unsigned int loraFifoBytes(void);
boolean loraFifoEmpty(void);
boolean loraFifoFull(void);
uint8_t loraFifoPush(uint8_t ch);
int loraFifoPop(void);
int loraFifoPeek(void);

#endif  // LORA_IO
