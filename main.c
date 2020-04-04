#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

void Ports_Init(void);

void vApplicationIdleHook();
static void Car_Pedestrian_Task( void *pvParameters );
static void Train_Task( void *pvParameters );
void Busy_Wait_1ms(unsigned long msec);
void Go_North_South();
void Go_East_west();
void Go_pedestrian();
void Turn_Red_Leds_on();
void Blink_Red_Leds();

xTaskHandle xTrain_Task_Handle;

int tgn = 5000, tgw = 2500, tcross = 10000, tsafety = 20000, fref = 800;  // In miliseconds

unsigned long SW1,SW2;
enum Direction {EAST, WEST};
enum Direction Train_Direction;

int Pedistrian_Request = 0;

int main(void){    
  
	Ports_Init();  
	xTaskCreate( Car_Pedestrian_Task, (const portCHAR *)"Normal&Pedestrian_Mode", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Train_Task, (const portCHAR *)"Train_Mode", configMINIMAL_STACK_SIZE, NULL, 2, &xTrain_Task_Handle );
	
	vTaskSuspend(xTrain_Task_Handle);
	
	/* Start the scheduler. */
	vTaskStartScheduler();
}

static void Car_Pedestrian_Task( void *pvParameters )
{

	for( ;; )
	{
		
		Go_North_South();
		vTaskDelay(tgn / portTICK_RATE_MS);

		if(Pedistrian_Request){
			Go_pedestrian();
			vTaskDelay(tcross / portTICK_RATE_MS);
			Pedistrian_Request = 0;
		}

		Go_East_west();
		vTaskDelay(tgw / portTICK_RATE_MS);
	
		if(Pedistrian_Request){
			Go_pedestrian();
			vTaskDelay(tcross / portTICK_RATE_MS);
			Pedistrian_Request = 0;			
		}
	}
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

static void Train_Task( void *pvParameters )
{

	for( ;; )
	{
    // turn red leds on		
		Turn_Red_Leds_on();
		
		// Blinking red leds for tsafety period
		portTickType xlastWakeTime = xTaskGetTickCount();
		while( (xTaskGetTickCount() - xlastWakeTime) < tsafety ){
			Busy_Wait_1ms(fref);
			Blink_Red_Leds();
		}
		
		// turn red leds on		
		Turn_Red_Leds_on();
		
		if(Train_Direction == EAST){
			while( (GPIO_PORTF_DATA_R&0x01) != 0); // waiting for train sensor 2
		}
		else if(Train_Direction == WEST){
			while( (GPIO_PORTF_DATA_R&0x10) != 0); // waiting for train sensor 1 
		}
		
		vTaskSuspend(NULL); // suspend this task
	}
}
/*-----------------------------------------------------------*/


void vApplicationIdleHook(){
	
	SW1 = !(GPIO_PORTF_DATA_R&0x10);     // read PF4 into SW1
	SW2 = !(GPIO_PORTF_DATA_R&0x01);     // read PF0 into SW2
	unsigned long Pedistrian_Switch = (GPIO_PORTB_DATA_R&0x10); // read PB4
			
	if(Pedistrian_Request == 0){
		Pedistrian_Request = Pedistrian_Switch;
	}
	
	
	if(SW1){
		Train_Direction = EAST;
		vTaskResume(xTrain_Task_Handle);
	}
	else if(SW2){
		Train_Direction = WEST;
		vTaskResume(xTrain_Task_Handle);
	}
}

void Ports_Init(void){ 
  SYSCTL_RCGCGPIO_R |= 0x00000020; // activate clock for port F
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // unlock PortF PF0  
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0  
	GPIO_PORTF_DIR_R = 0x0E;          // PF4,PF0 input, PF3,PF2,PF1 output
	GPIO_PORTF_PUR_R = 0x11;          // enable pullup resistors on PF4,PF0
  GPIO_PORTF_DEN_R = 0x1F;          // enable digital pins PF4-PF0
	
	SYSCTL_RCGCGPIO_R |= 0x00000002; // activate clock for port F
	GPIO_PORTB_DIR_R = 0x0F;          // B4 input, B0-3 output
  GPIO_PORTB_DEN_R = 0x1F;          // enable digital pins 
	
}

void Busy_Wait_1ms(unsigned long msec){
	unsigned long count;
  while(msec > 0 ) { 
    count = 13000; 
    while (count > 0) { 
      count--;
    } 
    msec--;
  }
}

// PB0 north/south red   leds
// PB1 north/south green leds
// PB2 east/west   red   leds
// PB3 east/west   green leds
// PB4 Pedestrian  switch

void Go_North_South(){
	// turn off all leds
	GPIO_PORTB_DATA_R &= ~(0x0F);
	
	// turn on red for east/west
	GPIO_PORTB_DATA_R |= 0x04;
	
	// turn tiva led to red
	GPIO_PORTF_DATA_R = 0x02;
	
	// turn on green for north/south
	GPIO_PORTB_DATA_R |= 0x02;
}

void Go_East_west(){
	// turn off all leds
	GPIO_PORTB_DATA_R &= ~(0x0F);
	
	// turn on red for north/south
	GPIO_PORTB_DATA_R |= 0x01;
	
	// turn tiva led to red
	GPIO_PORTF_DATA_R = 0x02;
	
	// turn on green for east/west
	GPIO_PORTB_DATA_R |= 0x08;
}

void Go_pedestrian(){
	
	// turn off all leds
	GPIO_PORTB_DATA_R &= ~(0x0F);
	
	// turn on intersection red leds
	GPIO_PORTB_DATA_R |= 0x05;
	
	// turn tiva led to red
	GPIO_PORTF_DATA_R = 0x02;
	
	// turn tiva led to green
	GPIO_PORTF_DATA_R = 0x08;
}

void Turn_Red_Leds_on(){
	// turn off all leds
	GPIO_PORTB_DATA_R &= ~(0x0F);
	
	// turn on intersection red leds
	GPIO_PORTB_DATA_R |= 0x05;
	
	// turn tiva led to red
	GPIO_PORTF_DATA_R = 0x02;
	
}

void Blink_Red_Leds(){
	// blink intersection red leds
	GPIO_PORTB_DATA_R ^= 0x05;
	
	// blink tiva red led
	GPIO_PORTF_DATA_R ^= 0x02;
}
