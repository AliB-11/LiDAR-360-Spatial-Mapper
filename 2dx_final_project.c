/* 
Name: Ali Bandali 
Student #: 400532826
Mac ID: bandaa2
Assigned bus frequency: 14MHz
Assigned LEDs:
Measuremnt status: PN0
UART Tx: PN1
Additional Status: PF4
*/





#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "PLL.h"
#include "SysTick.h"

#include "uart.h"
#include "VL53L1X_api.h"


// Function Prototypes
void PortN_Init(void); //LED 0,1
void PortF_Init(void); //LED 2,3
void PortM_Init(void); //B0,1,2
void PortJ_Init(void); //B0,1
void PortH_Init(void);   // Stepper motor


void PortN_Init(void) //set as outputs
{	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;		              // Activate the clock for Port E
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){};	      // Allow time for clock to stabilize
  
	GPIO_PORTN_DIR_R |= 0x03; //11													// Enable PE0 and PE1 as outputs
	GPIO_PORTN_DEN_R |= 0x03; //11                        		// Enable PE0 and PE1 as digital pins
	GPIO_PORTN_DATA_R &= ~0x03;
	return;
}

void PortF_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){};

  GPIO_PORTF_DIR_R |= 0x11; //0001 0001
  GPIO_PORTF_DEN_R |= 0x11;
	GPIO_PORTF_DATA_R &= ~0x11;
	return;
}


void PortM_Init(void) // set PM0, PM1, PM2 as inputs (buttons)
{	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;		              
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){};	      

  GPIO_PORTM_DIR_R &= ~0x07;   // PM0, PM1, PM2 as inputs (0b00000111)
  GPIO_PORTM_DEN_R |= 0x07;    // Digital enable
  GPIO_PORTM_PUR_R |= 0x07;    // Enable internal pull-ups
	return;
}


void PortH_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;				// activate clock for Port H
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0){};	// allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0x0F;        								// configure Port H pins (PM0-PM3) as output
  GPIO_PORTH_AFSEL_R &= ~0x0F;     								// disable alt funct on Port H pins (PM0-PM3)
  GPIO_PORTH_DEN_R |= 0x0F;        								// enable digital I/O on Port H pins (PM0-PM3)
																									// configure Port H as GPIO
  GPIO_PORTH_AMSEL_R &= ~0x0F;     								// disable analog functionality on Port H	pins (PM0-PM3)	
	return;
}

void PortE_Init(void){
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; // activate the clock for Port E
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4) == 0){}; // allow time for clock to stabilize
    GPIO_PORTE_DIR_R = 0b00001111;
    GPIO_PORTE_DEN_R = 0b00001111; // Enable PE0:PE3 
return;
}




#define MEASUREMENT_LED 0x01  // PN0 is bit 1 
#define UART_LED        0x02  // PN1 is bit 4 
#define ADDL_LED        0x10 // PF4 is bit 4 


////////////////////////////////////////////////////////////////////////////



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

#define DELAY										1				// stepper motor delay



void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
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


////////////////////////////////////////////////////////////////////////////



void cwSingleStep(void){ 
  uint32_t delay = 1;
	
	static uint8_t stepIndex = 0;
  uint8_t patterns[4] = {0b00000011,0b00000110,0b00001100, 0b00001001};	
	
	stepIndex = (stepIndex + 1) % 4;
	
  GPIO_PORTH_DATA_R = patterns[stepIndex]; 

  SysTick_Wait10ms(delay);
}

void ccwSingleStep(void){
  uint32_t delay = 1;
	
	static uint8_t stepIndex = 0;
  uint8_t patterns[4] = {0b00001001,0b00001100,0b00000110, 0b00000011};	
	
	stepIndex = (stepIndex + 1) % 4;
	
  GPIO_PORTH_DATA_R = patterns[stepIndex]; 

  SysTick_Wait10ms(delay);
}

void blinkLED1(void){
	uint32_t delay = 15;
  GPIO_PORTN_DATA_R |= MEASUREMENT_LED;     
  SysTick_Wait10ms(delay);
  GPIO_PORTN_DATA_R &= ~MEASUREMENT_LED;  
  //SysTick_Wait10ms(delay);
}

void blinkLED2(void){
	uint32_t delay = 15;
  GPIO_PORTN_DATA_R |= UART_LED;     
  SysTick_Wait10ms(delay);
  GPIO_PORTN_DATA_R &= ~UART_LED;  
  //SysTick_Wait(delay);
}

void blinkLED3(void){
	uint32_t delay = 15;
  GPIO_PORTF_DATA_R |= ADDL_LED;     
  SysTick_Wait10ms(delay);
  GPIO_PORTF_DATA_R &= ~ADDL_LED;  
  //SysTick_Wait(delay);
}

void measure_bus(){
    while(1){
        GPIO_PORTE_DATA_R ^= 1;
        SysTick_Wait10ms(1);
    }
}


// ====================================================================================
// ==============================     MAIN FUNCTION     ===============================
// ====================================================================================


uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void)
{
	PLL_Init();	
	SysTick_Init();
	
	PortN_Init();
	PortF_Init();
	PortM_Init();
  PortH_Init();
	PortE_Init();
	

/////////////////////////////////////////////////////////////////


	uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
	
	I2C_Init();
	UART_Init();	
	
	
	// 1 Wait for device ToF booted
	
 //measure_bus()
	while(sensorState==0) 
	{
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* 2 Initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);
	
	/* 3 Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
	//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
	//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
	//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

  status = VL53L1X_StartRanging(dev);   // 4 This function has to be called to enable the ranging
	
	
	// Turn off LEDs at reset
	GPIO_PORTN_DATA_R = 0b00000010;
	
	int motorRunning = 0;
	int angleSelect = 0; 
	int steps = 0;  
	int endCycle = 0;
	int homeBonus = 0;
	int buttonPressed0 = 0;
	int buttonPressed2 = 0;
	int buttonPressed3 = 0;
	int dir = 1;       // 1 => forward, -1 => backward	
	
	
	int depth = 0;
  int stepsInCycle = 0;    // steps taken in current direction
  int readings = 0;    // We do 64 readings per cycle	
  
	
	UART_printf("START_CYCLE\r\n");

	while(1)
	{


// Port M2 toggles motor on/off

		
		//If button 0 is pressed
		if((GPIO_PORTM_DATA_R & 0x04) == 0)
		{
			if(!buttonPressed0) //if 0, turn on
			{
				motorRunning = !motorRunning;
				buttonPressed0 = 1;
			}
		} 
		else //reset the button when released 
		{
			buttonPressed0 = 0;
		}


// Port M1 toggles homing and retunrs motor back home and ends program

		
		
		
		if((GPIO_PORTM_DATA_R & 0x02) == 0)
		{
			if(!buttonPressed2) 
			{
				endCycle = !endCycle;
				buttonPressed2 = 1;	
			}
		} 
		else 
		{
			void blinkLED3();
			buttonPressed2 = 0;
			void blinkLED3();
		}	

// Port M0 homing for bonus 


		
		if((GPIO_PORTM_DATA_R & 0x01) == 0)
		{
			if(!buttonPressed3) 
			{
				homeBonus = !homeBonus;
				buttonPressed3 = 1;
			}
		} 
		else 
		{
			buttonPressed3 = 0;
		}
		

////64 steps for 11.25* 
		
		int stepInterval = 64;
		
		// Inside your main loop (replace the existing data transmission section)

		if (motorRunning) 
		{
				// Take measurements
				if (steps % 64 == 0)
				{
						blinkLED1();     
						uint8_t dataReady = 0;

						// Wait for Tof
						while (dataReady == 0)
						{
								status = VL53L1X_CheckForDataReady(dev, &dataReady);
								VL53L1_WaitMs(dev, 5);
						}
						dataReady = 0;

						// Read data from sensor
						status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
						status = VL53L1X_GetDistance(dev, &Distance);
						status = VL53L1X_GetSignalRate(dev, &SignalRate);
						status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
						status = VL53L1X_GetSpadNb(dev, &SpadNum);
						status = VL53L1X_ClearInterrupt(dev);

						blinkLED2();

						// Send data in MATLAB-compatible format:
						// RangeStatus (check_bit), Distance, stepCount (angle), depth, SpadNum
						sprintf(printf_buffer, "%u,%u,%u,%u,%u\r\n", RangeStatus, Distance, steps, depth, SpadNum);
						UART_printf(printf_buffer);

						readings++;

						if (readings >= 32) 
						{
								readings = 0;
						}
						SysTick_Wait10ms(70);
				}

				// Motor 
				if (dir == 1)
				{
						cwSingleStep();
						GPIO_PORTN_DATA_R |= 0x01;
						steps++;
				}
				else
				{
						ccwSingleStep();
						GPIO_PORTN_DATA_R &= ~0x01;
						steps--;
						if(steps < 0) steps += 2048;  // wrap around
				}

				stepsInCycle++;

				if (stepsInCycle >= 2048)
				{
						dir = -dir; // Reverse 
						stepsInCycle = 0;
						steps = 0;
						depth += 1;
						UART_printf("NEW_CYCLE\r\n");

						SysTick_Wait10ms(400);
						
				}
		}

		
		// end (Port M1): Stops motor and indicates end of program. retunrs to home position 
		if (endCycle) 
		{			
				UART_printf("END_CYCLE\r\n");  // Send END_CYCLE via uart
			
				// Move back to zero position
				while (steps > 0) 
				{
						ccwSingleStep();
						steps--;
				}
				while (steps < 0) 
				{
						cwSingleStep();
						steps++;
				}

				endCycle = 0; 
				motorRunning = 0;    
				dir = 1;
				stepsInCycle = 0;
		}

		//Returns to home position -> continue measurments
		if (homeBonus) 
		{			
				// Move back to zero position
				while (steps > 0) 
				{
						ccwSingleStep();
						steps--;
				}
				while (steps < 0) 
				{
						cwSingleStep();
						steps++;
				}

				homeBonus = 0; 
				motorRunning = 1;    
				dir = 1;
				stepsInCycle = 0;

				
		}
	}
}