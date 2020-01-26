#ifndef IO_H
#define IO_H

#define LF    10
#define CR    13
#define BEL    7
#define TAB    8
#define ESC   27
#define SPACE 32  
 
void ioInit(void);
void ioStateMachine(void);
#endif
