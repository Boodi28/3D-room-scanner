//Abdalla Mahdy, mahdya, 400411114 2DX3 Final Project Code

#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"

// Define I2C status and control codes
#define I2C_MCS_ACK     0x00000008    // Data Acknowledge Enable
#define I2C_MCS_DATACK  0x00000008    // Acknowledge Data
#define I2C_MCS_ADRACK  0x00000004    // Acknowledge Address
#define I2C_MCS_STOP    0x00000004    // Generate STOP
#define I2C_MCS_START   0x00000002    // Generate START
#define I2C_MCS_ERROR   0x00000002    // Error
#define I2C_MCS_RUN     0x00000001    // I2C Master Enable
#define I2C_MCS_BUSY    0x00000001    // I2C Busy
#define I2C_MCR_MFE     0x00000010    // I2C Master Function Enable

#define MAX_RETRIES     5             // Maximum number of receive attempts before giving up

void I2C_Init(void) {
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;          // Activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;         // Activate port B
  while((SYSCTL_PRGPIO_R & 0x0002) == 0){};        // Wait for port B to be ready

  // Configure PB2 and PB3 as I2C pins
  GPIO_PORTB_AFSEL_R |= 0x0C;                      // Enable alternate function on PB2 and PB3
  GPIO_PORTB_ODR_R |= 0x08;                        // Enable open drain on PB3 only
  GPIO_PORTB_DEN_R |= 0x0C;                        // Enable digital I/O on PB2 and PB3
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) + 0x00002200; // Configure PB2 and PB3 for I2C communication

  I2C0_MCR_R = I2C_MCR_MFE;                        // Enable I2C master function
  I2C0_MTPR_R = 0b0000000000000101000000000111011; // Configure for 100 kbps clock with glitch suppression (added 8 clocks of glitch suppression ~50ns)
}

void Init_PortN(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12; // activate the clock for Port N
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){};// wait for clock to stabilize
  GPIO_PORTN_DIR_R = 0b00000010; // make PN1 an output to turn on LED
  GPIO_PORTN_DEN_R = 0b00000010; // enable PN1
  return;
}

// The VL53L1X sensor needs to be reset using XSHUT. We will use PG0 for this.
void Init_PortG(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6; // activate clock for Port G
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R6) == 0){}; // wait for clock to stabilize
  GPIO_PORTG_DIR_R &= 0x00; // make PG0 an input (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01; // disable alternate function on PG0
  GPIO_PORTG_DEN_R |= 0x01; // enable digital I/O on PG0
  // configure PG0 as a GPIO pin
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01; // disable analog functionality on PG0
  return;
}

// This function initializes Port H pins for output
void PortH_Init(void){
  // Use Port H pins for output
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                // Activate clock for Port H
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};    // Allow time for clock to stabilize
  GPIO_PORTH_DIR_R |= 0xFF;                                        // Make PH0 out (PN0 built-in LED1)
  GPIO_PORTH_AFSEL_R &= ~0xFF;                                     // Disable alternate function on PH0
  GPIO_PORTH_DEN_R |= 0xFF;                                        // Enable digital I/O on PH0
  GPIO_PORTH_AMSEL_R &= ~0xFF;                                     // Disable analog functionality on PH0        
  return;
}

// This function enables interrupts
void EnableInt(void)
{
  __asm("cpsie i\n"); // Enable interrupts
}

// This function disables interrupts
void DisableInt(void)
{
  __asm("cpsid i\n"); // Disable interrupts
}

// This function waits for an interrupt to occur
void WaitForInt(void)
{
  __asm("wfi\n"); // Low power wait
}

// Global variable visible in Watch window of debugger, increments at least once per button press
volatile unsigned long numFallingEdges = 0;

// Set up Port J as input GPIO with clock activation
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;	// Enable clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Wait for clock to stabilize
	GPIO_PORTJ_DIR_R &= ~0x02;	// Set PJ1 as input
	GPIO_PORTJ_DEN_R |= 0x02;	// Enable digital I/O on PJ1
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	// Configure PJ1 as GPIO
	GPIO_PORTJ_AMSEL_R &= ~0x02;	// Disable analog functionality on PJ1
	GPIO_PORTJ_PUR_R |= 0x02;	// Enable weak pull up resistor on PJ1
}


// Set up interrupt for GPIO Port J IRQ# 51
void PortJ_Interrupt_Init(void){
	
		// Initialize interrupt counter
		numFallingEdges = 0;

		// Configure interrupt settings for PJ1
		GPIO_PORTJ_IS_R = 0;	// PJ1 is edge-sensitive 
		GPIO_PORTJ_IBE_R = 0;	// PJ1 is not both edges 
		GPIO_PORTJ_IEV_R = 0;	// PJ1 falling edge event 
		GPIO_PORTJ_ICR_R = 0x02;	// Clear interrupt flag by setting proper bit in ICR register
		GPIO_PORTJ_IM_R = 0x02;	// Arm interrupt on PJ1 by setting proper bit in IM register
    
		// Enable interrupt 51 in NVIC
		NVIC_EN1_R = 0x00080000;
	
		// Set interrupt priority to 5
		NVIC_PRI12_R = 0xA0000000;
		
		// Enable global interrupts
		EnableInt();
}



//	(Step 5) IRQ Handler (Interrupt Service Routine).  
//  				This must be included and match interrupt naming convention
//	 				in startup_msp432e401y_uvision.s 
//					(Note - not the same as Valvano textbook).
void GPIOJ_IRQHandler(void){
  numFallingEdges = numFallingEdges + 1;	// Increase the global counter variable ;Observe in Debug Watch Window
	GPIO_PORTJ_ICR_R = 0x02;     					// acknowledge flag by setting proper bit in ICR register
}


//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}
void spin(){
    for(int i=0; i<32; i++){
        GPIO_PORTH_DATA_R = 0b00001100;
        SysTick_Wait10ms(4);
        GPIO_PORTH_DATA_R = 0b00000110;
        SysTick_Wait10ms(4);
        GPIO_PORTH_DATA_R = 0b00000011;
        SysTick_Wait10ms(4);
        GPIO_PORTH_DATA_R = 0b00001001;
        SysTick_Wait10ms(4);
    }
	}

void rev_spin(){
    for(int i=0; i<512; i++){
         GPIO_PORTH_DATA_R = 0b00001001;
        SysTick_Wait10ms(5);
        GPIO_PORTH_DATA_R = 0b00000011;
        SysTick_Wait10ms(5);
        GPIO_PORTH_DATA_R = 0b00000110;
        SysTick_Wait10ms(5);
        GPIO_PORTH_DATA_R = 0b00001100;
        SysTick_Wait10ms(5);
    }
	}

//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	PortH_Init();
	I2C_Init();
	UART_Init();
	Init_PortN();					// Initialize the onboard LED on port N
	PortJ_Init();											// Initialize the onboard push button on PJ1
	PortJ_Interrupt_Init();	
	
	
// Main function to read distance data from the VL53L1X ToF sensor
while(1){
	// Wait for interrupt from button to start
	WaitForInt();
	
	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
		FlashLED1(1);
	}

	status = VL53L1X_ClearInterrupt(dev); // Clear interrupt to enable next interrupt

	// Initialize sensor with default settings
	status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	// Start ranging to enable data capture
	status = VL53L1X_StartRanging(dev);

	// Capture 16 scans
	for(int i = 0; i < 16; i++) {
		// Wait until the ToF sensor's data is ready
		while (dataReady == 0){
			status = VL53L1X_CheckForDataReady(dev, &dataReady);
			VL53L1_WaitMs(dev, 5);
			FlashLED1(1);
		}
		dataReady = 0;
		
		// Read the data values from ToF sensor
		status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
		status = VL53L1X_GetDistance(dev, &Distance); // The measured distance value
		status = VL53L1X_GetSignalRate(dev, &SignalRate);
		status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
		status = VL53L1X_GetSpadNb(dev, &SpadNum);

		status = VL53L1X_ClearInterrupt(dev); // Clear interrupt to enable next interrupt

		// Print the resulted readings to UART
		sprintf(printf_buffer,"%u\r\n", Distance);
		UART_printf(printf_buffer);
		
		// Flash an LED to indicate data capture
		FlashLED1(1);
		GPIO_PORTN_DATA_R = 0b00000010;
		SysTick_Wait10ms(30);
		GPIO_PORTN_DATA_R = 0b00000000;
		
		// Wait and then spin the motor
		spin();
		SysTick_Wait10ms(150);
	}
	
	// Stop ranging and spin the motor in reverse direction
	rev_spin();
	VL53L1X_StopRanging(dev);
}
}
