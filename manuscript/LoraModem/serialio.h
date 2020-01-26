#ifndef SERIAL_IO
#define SERIAL_IO

/*
 * Initialize SERIAL_IO
 */
void serialFifoInit( void );
boolean serialFifoEmpty(void);
boolean serialFifoFull(void);
unsigned int serialFifoBytes(void);
unsigned int serialFifoFree(void);
uint8_t serialFifoPush(uint8_t uch);
int serialFifoPop( void );
int serialFifoPeek( void );

#endif  // SERIAL_IO
