//Ebrahim Simmons, simmoe1, 400200042

#include <stdint.h>
#include "tm4c1294ncpdt.h"
//#include "PLL.h"
#include "SysTick.h"
#include "onboardLEDs.h"

#define DELAY 1

//******************************************************************************************************************************************

//Flashing D4
void FlashLED4(int count) {
		while(count--) {
			GPIO_PORTF_DATA_R ^= 0b00000001; 							
			SysTick_Wait10ms(DELAY);													
			GPIO_PORTF_DATA_R ^= 0b00000001;			
			SysTick_Wait10ms(DELAY);														
		}
}

//Flashing D3
void FlashLED3(int count) {
		while(count--) {
			GPIO_PORTF_DATA_R ^= 0b00010000; 							
			SysTick_Wait10ms(DELAY);													
			GPIO_PORTF_DATA_R ^= 0b00010000;			
			SysTick_Wait10ms(DELAY);															
		}
}

//Flashing D2
void FlashLED2(int count) {
		while(count--) {
			GPIO_PORTN_DATA_R ^= 0b00000001; 								
			SysTick_Wait10ms(DELAY);														
			GPIO_PORTN_DATA_R ^= 0b00000001;			
			SysTick_Wait10ms(DELAY);														
		}
}

//Flashing D1
void FlashLED1(int count) {
		while(count--) {
			GPIO_PORTN_DATA_R ^= 0b00000010; 							
			SysTick_Wait10ms(DELAY);													
			GPIO_PORTN_DATA_R ^= 0b00000010;			
			SysTick_Wait10ms(DELAY);													
		}
}

//Flasing ALL 
void FlashAllLEDs(){
		GPIO_PORTN_DATA_R ^= 0b00000011; 							
		GPIO_PORTF_DATA_R ^= 0b00010001; 							
		SysTick_Wait10ms(25);												
		GPIO_PORTN_DATA_R ^= 0b00000011;			
		GPIO_PORTF_DATA_R ^= 0b00010001; 								
		SysTick_Wait10ms(25);														
}

//******************************************************************************************************************************************

void FlashI2CTx(){
	FlashLED1(1);
}

void FlashI2CRx(){
	FlashLED2(1);
}

//******************************************************************************************************************************************

//Flashing Error for D1 D2 D3 D4
void FlashI2CError(int count) {
		while(count--) {
			FlashAllLEDs();
		}
}

//******************************************************************************************************************************************

//onboardLEDs_Init() initializes the LEDs
void onboardLEDs_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;//Port N clock
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};	
	GPIO_PORTN_DIR_R |= 0x03; //PN0 out
  GPIO_PORTN_AFSEL_R &= ~0x03;     								
  GPIO_PORTN_DEN_R |= 0x03;        								
  GPIO_PORTN_AMSEL_R &= ~0x03;     								

	//PortF - Onboard LEDs	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;//Port N clock
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};	
	GPIO_PORTF_DIR_R |= 0x11; //PN4,0 out 
  GPIO_PORTF_AFSEL_R &= ~0x11;     								
  GPIO_PORTF_DEN_R |= 0x11;        																														
  GPIO_PORTF_AMSEL_R &= ~0x011;     								
	FlashAllLEDs();
	return;
}




