
/**********************************************************************************
* SWO trace must be connected (PCB)
* Trace Asynchronous SW programming mode must be enabled (manually or use CubeMX)
* The next Projectt settings Checkbuttons must be set (Keil uVision):
* 	Debud-Debugger Settings-Debug-Port->SW
* 	                        Trace-Enable->0x00000001
* 													      Privelege->0x00000000
* 																Core Clock->...
* ---------------------------------------------------
* How to use 
*      printf("Hello from stm32 printf!\r\n"); 
* The result available in window "Debug (printf) viewer" in debug session
**********************************************************************************/

#pragma once
#include <stdio.h>



#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))

#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

struct __FILE { int handle; /* Add whatever you need here */ };

int fputc(int ch, FILE *f);

/* EOF printf.h */
