// Ebrahim Simmons, 400200042, simmoe1
// Assigned Bus Speed - 60MHz
// DistanceStatus - PF4
// DisplacementStatus - PL3

#include <stdint.h>
#include "PLL.h"
#include <math.h>  
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "onboardLEDs.h"
#include "Systick.h"
#include "uart.h"

//******************************************************************************************************************************************

//Port E for motor
void PortE_Init(void){	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; //Activate clock - Port E
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4) == 0){}; //Time for clock
	GPIO_PORTE_DEN_R= 0b00001111;
	GPIO_PORTE_DIR_R |= 0b00001111;                             
	GPIO_PORTE_DATA_R = 0b00000000;                           
	return;
}

//******************************************************************************************************************************************

//Port L3 for output 
void PortL_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10; //Activate clock - Port L
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){}; //Time for clock
	GPIO_PORTL_DIR_R = 0b00001001;       								    
  GPIO_PORTL_DEN_R = 0b00001001;
	GPIO_PORTL_DATA_R = 0b00000000;  
	return;
}

//******************************************************************************************************************************************

//LED onboard lights
void PortF_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; //Activate clock - Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};
	GPIO_PORTF_DIR_R=0b00010001;
	GPIO_PORTF_DEN_R=0b00010001;
	return;
}

//******************************************************************************************************************************************

// Default address
uint16_t	dev=0x52; 
int arr[8] = {0,0,0,0,0,0,0,0};
int trackStatus=0; //tracks status of sensor

#define isInterrupt 1

//******************************************************************************************************************************************

void I2C_Init(void);
void UART_Init(void);
void PortG_Init(void);
void VL53L1X_XSHUT(void);

//Debugger vars
  uint8_t byteData, sensorState=0; 
  uint8_t RangeStatus;
  uint8_t dataReady;
  uint16_t wordData;
  uint8_t ToFSensor = 1; 
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 

//******************************************************************************************************************************************

//ToF_Init() sets up sensor 
void ToF_Init(void){
  trackStatus = VL53L1_RdByte(dev, 0x010F, &byteData);
  trackStatus = VL53L1_RdByte(dev, 0x0110, &byteData);
	trackStatus = VL53L1_RdWord(dev, 0x010F, &wordData);
	trackStatus = VL53L1X_GetSensorId(dev, &wordData);

	while(sensorState==0){
		trackStatus = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	
	trackStatus = VL53L1X_ClearInterrupt(dev); //clear interrupt
  trackStatus = VL53L1X_SensorInit(dev); //initialize sensor
	Status_Check("SensorInit", trackStatus);

  trackStatus = VL53L1X_StartRanging(dev);
	Status_Check("StartRanging", trackStatus);
}

//******************************************************************************************************************************************

//getTOFDistance() obtains a measurement with a distance value and returns it 
int getTOFDistance(void) {
	
	 while (dataReady == 0){ 
		  trackStatus = VL53L1X_CheckForDataReady(dev, &dataReady);
          FlashLED3(1);
          VL53L1_WaitMs(dev, 5);
	  } //while loop waiting 
      dataReady = 0; 
	    trackStatus = VL53L1X_GetRangeStatus(dev, &RangeStatus); 
	    trackStatus = VL53L1X_GetDistance(dev, &Distance); 
      FlashLED4(1); 
	    trackStatus = VL53L1X_ClearInterrupt(dev); 
			return Distance;
}

//******************************************************************************************************************************************

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

//******************************************************************************************************************************************

//I2C_Init() initializes communication to sensor 
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; //PortB
	
  while((SYSCTL_PRGPIO_R&0x0002) == 0){}; 
    GPIO_PORTB_AFSEL_R |= 0x0C;  
    GPIO_PORTB_ODR_R |= 0x08;        
    GPIO_PORTB_DEN_R |= 0x0C;     
		GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;
    I2C0_MCR_R = I2C_MCR_MFE; //master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011; 
}

//******************************************************************************************************************************************

//PortG_Init() resets VL53L1X
void PortG_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){
		};
    GPIO_PORTG_DIR_R &= 0x00; 
  GPIO_PORTG_AFSEL_R &= ~0x01; 
  GPIO_PORTG_DEN_R |= 0x01; 
		//disable analog 
  GPIO_PORTG_AMSEL_R &= ~0x01; 
    return;
}

//******************************************************************************************************************************************

//VL53L1X_XSHUT() 
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01; //PG0
    GPIO_PORTG_DATA_R &= 0b11111110; //=0
    FlashAllLEDs();
    SysTick_Wait10ms(10); //delay
    GPIO_PORTG_DIR_R &= ~0x01;
    
}

//******************************************************************************************************************************************

//motorTurn() turns on the stepper motor
void motorTurn(int speed){
	SysTick_Wait10ms(speed);
	GPIO_PORTE_DATA_R = 0b00001100;
	SysTick_Wait10ms(speed);
	GPIO_PORTE_DATA_R = 0b00000110;
	SysTick_Wait10ms(speed);
	GPIO_PORTE_DATA_R = 0b00000011;
	SysTick_Wait10ms(speed);
	GPIO_PORTE_DATA_R = 0b00001001;
}

//******************************************************************************************************************************************

void DisableInt(void)
{    __asm("    cpsid   i\n");
}

//******************************************************************************************************************************************

void EnableInt(void)
{    __asm("    cpsie   i\n");
}

//******************************************************************************************************************************************

void WaitForInt(void)
{    __asm("    wfi\n");
}

//******************************************************************************************************************************************

//ExternalButton_Init() sets PJ1 as interrupt
void ExternalButton_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;//clock for PortJ
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	
  GPIO_PORTJ_DIR_R &= ~0x02;  

  GPIO_PORTJ_DEN_R |= 0x02;     
	GPIO_PORTJ_PCTL_R &= ~0x000000F0; 
	GPIO_PORTJ_AMSEL_R &= ~0x02;	
	GPIO_PORTJ_PUR_R |= 0x02;		
  GPIO_PORTJ_IS_R &= ~0x02;  //edge sensitive 
  GPIO_PORTJ_IBE_R &= ~0x02; //not both edges sensitive   
  GPIO_PORTJ_IEV_R &= ~0x02;
  GPIO_PORTJ_ICR_R = 0x02;      
  GPIO_PORTJ_IM_R |= 0x02;    
  NVIC_PRI13_R = (NVIC_PRI13_R&0xFF00FFFF)|0x000A0000; 
  NVIC_EN1_R |= 0x00080000;             
  EnableInt();        
}

int currentState = 0; //current state variable 

//******************************************************************************************************************************************

//GPIOJ_IRQHandler() when PJ1 flagged 
void GPIOJ_IRQHandler(void){
  GPIO_PORTJ_ICR_R = 0x02; 
	
	if(currentState ==0){ //toggling state
		currentState = 1;
	}
	else{
		currentState=0;
	}
}

//******************************************************************************************************************************************

#define PI 3.1415927 //pi definition 

//******************************************************************************************************************************************

//main() for main logic, sensor, and motor
int main(void){
	
	UART_Init();
	ToF_Init();
	ExternalButton_Init();
	onboardLEDs_Init();
	I2C_Init();
  PLL_Init();
	SysTick_Init();
	PortE_Init();
	PortL_Init();
	PortF_Init();
	
	//xDisplacement = 0 
	int xDisplacement = 0;
	
	//Infinite loop
	while(1){
		
		//checkpoint for main loop 
		loop:
		
		GPIO_PORTL_DATA_R = 0b00001000; 
		
		if(currentState == 1){ //if currentState = ON 
			int a = 0;
			
			while(a<=7){ 
				if(currentState == 1){
				int c = 1; //variable to count to 64
					
				GPIO_PORTL_DATA_R = 0b00000000;
					while(c<=64){
						motorTurn(1); //calling motor 
						c++; //increase by 1
					
						if(c == 64){ 
							GPIO_PORTF_DATA_R = 0b000010000;
							SysTick_Wait10ms(100);
							GPIO_PORTF_DATA_R = 0b000000000;
						}
					}				
					arr[a] = getTOFDistance(); //return distance and store in array 
				}
				
				else{
					goto loop;
				}
				a++; //increase by 1 
			}
			currentState = 0; 
		}
		
		else if (currentState == 0){
			if(arr[0] != 0){
				int degree = 0;
				
				for(int d = 0;d <= 7;d++){
					if(arr[d] != 0){
					sprintf(printf_buffer,"%d %d %d\r\n", xDisplacement, (int)round(arr[d]*cos(degree*PI/180)) ,(int)round(arr[d]*sin(degree*PI/180))); //send x y z coordinates 
					UART_printf(printf_buffer);
					}
					
					degree +=45; //increase degree by 45
					arr[d] = 0; //array to 0 
				}
				
				xDisplacement+=200; //increasing xDisplacement by 200mm
				sprintf(printf_buffer,"End\r\n"); 
				UART_printf(printf_buffer);
			}
			GPIO_PORTL_DATA_R = 0b00001000; //turning on LED
		}
	}
}
//******************************************************************************************************************************************