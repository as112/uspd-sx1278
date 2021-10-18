/*************************************************** Это вывод отладочной информации в UART2 ************************************/
/*
#include <stdio.h>
#include "main.h"
struct __FILE {
  int handle;
};
FILE __stdout;

int fputc(int ch, FILE *f) 
{  HAL_UART_Transmit(&huart2, (uint8_t *) &ch, 1, 0xFFFF);
	//ITM_SendChar ( (uint32_t )&ch );
  return ch;}

int ferror(FILE *f) {
  return 0;
}
*/

/*************************************************** Это вывод отладочной информации в Debug viewer ************************************/

#include <stdio.h>
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000


struct __FILE { int handle;}; // Add whatever you need here  
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
   if (DEMCR & TRCENA) {

while (ITM_Port32(0) == 0)
	;
    ITM_Port8(0) = ch;
  }
  return(ch);
}
