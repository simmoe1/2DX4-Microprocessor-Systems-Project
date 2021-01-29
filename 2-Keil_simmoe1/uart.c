//Ebrahim Simmons, simmoe1, 400200042

#include "uart.h"
#include "tm4c1294ncpdt.h"
#include <stdint.h>
#include <stdio.h>

//******************************************************************************************************************************************

//UART_Init() initizializes UART0
void UART_Init(void) {
	SYSCTL_RCGCUART_R |= 0x0001; //activate UART0   
	SYSCTL_RCGCGPIO_R |= 0x0001; //activate port A   
	while((SYSCTL_PRUART_R&SYSCTL_PRUART_R0) == 0){};
		
  UART0_CTL_R &= ~UART_CTL_UARTEN;//disable UART
  UART0_IBRD_R = 8; //16000000/(16*115200)= int(8.681) = 8 
  UART0_FBRD_R = 44;//0.6806*64 = 44
		
  UART0_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN); //UART gets clock from other clock source
  UART0_CC_R = (UART0_CC_R&~UART_CC_CS_M)+UART_CC_CS_PIOSC; //PIOSC alternate source
  SYSCTL_ALTCLKCFG_R = (SYSCTL_ALTCLKCFG_R&~SYSCTL_ALTCLKCFG_ALTCLK_M)+SYSCTL_ALTCLKCFG_ALTCLK_PIOSC;
  UART0_CTL_R &= ~UART_CTL_HSE; //high-speed disable
	UART0_LCRH_R = 0x0070;
	UART0_CTL_R = 0x0301;	
		//UART 
	GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFFFFF00)+0x00000011; 
	GPIO_PORTA_AMSEL_R &= ~0x03;	 
	GPIO_PORTA_AFSEL_R |= 0x03;		
	GPIO_PORTA_DEN_R |= 0x03;			
}

//******************************************************************************************************************************************

//UART_InChar() waits for input and returns ASCII
	char UART_InChar(void){
		while((UART0_FR_R&0x0010) != 0);		// wait until RXFE is 0   
		return((char)(UART0_DR_R&0xFF));
	} 
	
//******************************************************************************************************************************************
	
	//UART_printf()
	void UART_printf(const char* array){
		int ptr=0;
		while(array[ptr]){
			UART_OutChar(array[ptr]);
			ptr++;
		}
	}
	
//******************************************************************************************************************************************

	//UART_OutChat waits for buffers and proceeds to output 
	void UART_OutChar(char data){
		while((UART0_FR_R&0x0020) != 0);	
		UART0_DR_R = data;
	} 

//******************************************************************************************************************************************

	//Status_Check()
	void Status_Check(char* array, int status){
			if (status != 0){
				UART_printf(array);
				sprintf(printf_buffer," failed with (%d)\r\n",status);
				UART_printf(printf_buffer);
			}
			else
			{
				UART_printf(array);
				UART_printf(" Successful.\r\n");
			}
	}
	
//******************************************************************************************************************************************
