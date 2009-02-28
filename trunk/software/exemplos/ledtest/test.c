#include <p18f4620.h>

#define LED0 LATAbits.LATA0
#define LED1 LATAbits.LATA1

#pragma romdata CONFIG1H = 0x300001
const rom unsigned char config1H = 0b00000110;      // HSPLL oscillator

#pragma romdata CONFIG2L = 0x300002
const rom unsigned char config2L = 0b00011111;      // Brown-out Reset Enabled in hardware @ 2.0V, PWRTEN disabled

#pragma romdata CONFIG2H = 0x300003
const rom unsigned char config2H = 0b00010010;      // HW WD disabled, 1:512 prescaler

#pragma romdata CONFIG3H = 0x300005
const rom unsigned char config3H = 0b10000000;      // PORTB digital on RESET

#pragma romdata CONFIG4L = 0x300006
const rom unsigned char config4L = 0b10000001;      // DEBUG disabled,
                                                    // XINST disabled
                                                    // LVP disabled
                                                    // STVREN enabled
void sleep(void);
void InitializePort(void);

void main(void)
{
	InitializePort();

	LED0 = 1;
	LED1 = 0;

	while(1)
	{
		LED0 = 1;
		LED1 = 0;
		sleep();
		LED0 = 0;
		LED1 = 1;
		sleep();
	}
}

void InitializePort()
{
	ADCON1 = 0x0F;	// porta A como E/S digital	
	LATA = 0x04;
	TRISA = 0xE0;
}

void sleep(void)
{
	int j=0,i=0;
	for(i=0; i < 100; i++)
		for(j=0; j < 1000; j++);
}