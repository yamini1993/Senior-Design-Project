/*
<Hw7.c >
Yamini Yamini
Date: 01/05/2018
<Synchronization, Shared Data, and Shared Resources>

	This homework indicates the concept of Cooperating Tasks and designing out Race Conditions 
	on a shared resource.In this assignment, We create three continuous tasks with 
	same priority using the same text space, which generate random numbers, 

	place the numbers in a Queue in sequence of each task: t1, t2, t3, t1, etc.
	
	A fourth task will read three numbers from the Queue and return in the Queue the maximum number submitted 
	by one of the three tasks and identify which tasks has the highest value. 

	Switches/buttons will be configured to generate an asynchronous interrupt from the user. 
	When one of the switches is depressed, the Queue will be reset to zero entries. 
	

  	LEDs will be configured to display status of the tasks 1,2,3.

Scope: 
	Followed the follwing steps:
	Create a Queue
	Create 3 continuous tasks (Task1-Task3)
	Create Task4
	check the status of the queue
	if the queue is full press switch 1
	by pressing switch 1, it generates the interrupt which clears the queue
	


Test Plan:
	
	verify the size of the queue 
	check thee status of the queue
	Make sure the LED's function as indicated in the question


*/

#include <p32xxxx.h>
#include <plib.h>

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Demo includes. */
#include "basic_io.h"

/* PIC32 configuration. */
#pragma config FPLLODIV = DIV_1, FPLLMUL = MUL_20, FPLLIDIV = DIV_2
#pragma config FWDTEN = OFF, FPBDIV = DIV_2, POSCMOD = XT, FNOSC = PRIPLL, CP = OFF
#pragma config FSRSSEL = PRIORITY_7 //comment out for PIC32 PIC32MX460F512L

/* Software interrupt bit used within the CAUSE register to generate an 
interrupt. */
#define mainSW1_CAUSE_BIT            ( 0x01UL << 9 )

/* The software interrupt bit within the Interrupt Flag Status Register. */
#define mainSW1_INT_BIT                ( 0x04UL )

/* Macro to force a software interrupt. */
#define mainTRIGGER_INTERRUPT()                 \
{                                               \
unsigned long ulStatus;                         \
                                                \
    /* Trigger software interrupt. */           \
    ulStatus = _CP0_GET_CAUSE();                \
    ulStatus |= mainSW1_CAUSE_BIT;              \
    _CP0_SET_CAUSE( ulStatus );                 \
}

/* Macro to clear the same software interrupt. */
#define mainCLEAR_INTERRUPT()                   \
{                                               \
unsigned long ulStatus;                         \
                                                \
    /* Trigger software interrupt. */           \
    ulStatus = _CP0_GET_CAUSE();                \
    ulStatus &= ~mainSW1_CAUSE_BIT;             \
    _CP0_SET_CAUSE( ulStatus );                 \
}


// hack  this is added code to example 14
#define ENABLE	1		// generic set bit (1)
#define DISABLE	0		// generic unset bit (0)

//PORT Bit Masks
#define SW1	0b1000000	//mask rd6

//CN Bit Masks
#define EN_CN	0x8000	// enable change notice module
#define EN_CN15	(1<<15)	// CN bit 15


//IPC bit masks
#define PRI2	0x80000	// set priority to 2 enabled
//
// my api proto_types
//

extern void configCNInitPullup( unsigned int ); //configures CN interrupts and pull-ups
extern void enableCN();					//enables Change Notification Module
// thus endth the hack added code to example 14

/* The tasks to be created. */
static void vIntegerGenerator( void *pvParameters );
static void vLEDGenerator( void *pvParameters );

/* The service routine for the interrupt. The C compiler extensions are used to
indicate that this is an interrupt entry point to use for the 
_CORE_SOFTWARE_1_VECTOR vector location.  The assembly wrapper for the 
interrupt (vSW1_ISR_Wrapper()) is defined in ISR_Wrapper.S, the C portion of
the handler (vSW1_ISR_Handler()) is defined in this file. */

void __attribute__( (interrupt(ipl1), vector(_CORE_SOFTWARE_1_VECTOR))) vSW1_ISR_Wrapper( void );

/* Basic hardware and debug interface configuration. */
void vSetupEnvironment( void );

/*-----------------------------------------------------------*/

unsigned long ulNext = 0;
unsigned long ulCount;
unsigned long ul[ 100 ];

/* Declare two variables of type xQueueHandle.  One queue will be read from
within an ISR, the other will be written to from within an ISR. */
xQueueHandle xIntegerQueue, xStringQueue;

  char *SoftIntString[] =
{
    "Task 0\n",
    "Task 1\n",
    "Task 2\n",
    "Task 3\n"
};
// hack  just wanted to show you how pointers to pointers to data are dereferenced :)
char **pcStrings = SoftIntString; //initalize pointer to the soft interrupt


BaseType_t task1;
BaseType_t task2;
BaseType_t task3;
BaseType_t task4;


int main( void )
{
	/* Configure both the hardware and the debug interface. */
	vSetupEnvironment();

    /* Before a queue can be used it must first be created.  Create both queues
	used by this example.  One queue can hold variables of type unsigned long, 
	the other queue can hold variables of type char*.  Both queues can hold a 
	maximum of 10 items.  A real application should check the return values to 
	ensure the queues have been successfully created. */
    xIntegerQueue = xQueueCreate( 10, sizeof( unsigned long ) );
//	xLEDQueue = xQueueCreate( 10, sizeof( char * ) );
	if (xIntegerQueue != NULL)
	{
		/* Create the task that uses a queue to pass integers to the interrupt service
		routine.  The task is created at priority 1. */
		task1 = xTaskCreate( vIntegerGenerator, "Task1", 240, NULL, 2, NULL );
	
		task2 = xTaskCreate( vIntegerGenerator, "Task2", 240, NULL, 2, NULL );
	
		task3 = xTaskCreate( vIntegerGenerator, "Task3", 240, NULL, 2, NULL );

		task4 = xTaskCreate( vMaxGenerator, "Task4", 240, NULL, 2, NULL );

		/* Create the task that prints out the strings sent to it from the interrupt
		service routine.  This task is created at the higher priority of 2. */
		xTaskCreate( vLEDGenerator, "LED_TASK", 240, NULL, 1, NULL );

		TRISDCLR = 0x07; //clear bits for leds as outputs
		PORTD = 0x7; // turn ON all three LEDS
		
			configCNInitPullup( EN_CN15 ); //cn for RD6 SW1
			enableCN();
		
		
			IPC6SET = PRI2;  // set priority to 2 enabled
		
		    // explain why these must be here
			IFS1CLR = ENABLE; // Clear the interrupt flag status bit
			IEC1SET = ENABLE; // Enable Change Notice interrupts
		
			INTEnableSystemMultiVectoredInt();	//System interrupts are turned on
												// Enables system wide multi-vectored interrupts
		
			/* Start the scheduler so the created tasks start executing. */
			vTaskStartScheduler();
	}	
	else
	{
		/* Do nothing */
	}
	for( ;; );
	return 0;
}
/*-----------------------------------------------------------*/

static void vMaxGenerator(void *pvParameters) 
{
	portTickType xLastExecutionTime;

	static unsigned long n1;
	static unsigned long n2;
	static unsigned long n3;
	static unsigned long res;
	
	for( ;; )
	{
		vTaskDelayUntil( &xLastExecutionTime, 100 / portTICK_RATE_MS );
		
		vTaskSuspend(task1);
		vTaskSuspend(task2);
		vTaskSuspend(task3);

		xQueueReceive( xIntegerQueue, &n1, portMAX_DELAY );
		xQueueReceive( xIntegerQueue, &n2, portMAX_DELAY );
		xQueueReceive( xIntegerQueue, &n3, portMAX_DELAY );
	
		res = getMax(n1, n2, n3);
		
		xQueueSendToFront( xIntegerQueue, &res, 0 );

		vTaskResume(task1);
		vTaskResume(task2);
		vTaskResume(task3);

	}
}

static int getMax(int a , int b , int c) {
	return (a>b?a:b)>c?(a>b?a:b):c;
}

static unsigned long task;
static void vIntegerGenerator( void *pvParameters )
{
portTickType xLastExecutionTime;
unsigned portLONG ulValueToSend = 0;
int i;
static unsigned long top;
task = task + 1;

	/* Initialize the variable used by the call to vTaskDelayUntil(). */
	xLastExecutionTime = xTaskGetTickCount();


	for( ;; )
	{

		/* This is a periodic task.  Block until it is time to run again.
		The task will execute every 200ms. */
		vTaskDelayUntil( &xLastExecutionTime, 100 / portTICK_RATE_MS );
		
		xQueueReceive( xIntegerQueue, &top, portMAX_DELAY );
		if(top == ulValueToSend) { //If the current task is the maximum task it sends its number to led generation queue.
			xQueueSendToBack( xStringQueue, task, 0 );
			//Glow LED
			/* Send an incrementing number to the queue three times.  These will be 
			read from the queue by the interrupt service routine.  A block time is 
			not specified. */
			for( i = 0; i < 2; i++ )
			{
				xQueueSendToBack( xIntegerQueue, &ulValueToSend, 0 );
				ulValueToSend++;
			}
		} else {
			xQueueSendToFront( xIntegerQueue, &top, 0 );
			continue;
		}
		
		/* Force an interrupt so the interrupt service routine can read the
		values from the queue. */
		vPrintString( "Continuous Task.\n" );
		mainTRIGGER_INTERRUPT();
		vPrintString( "Generator task - Interrupt generated.\n\n" );
	}
	
}
/*-----------------------------------------------------------*/


//Whenever Maximum task is detected, it adds the task number to string queu
// which will glow the respective led with this method.
static void vLEDGenerator( void *pvParameters )
{
int *num=0;
int z=0,y=0;
		

	for( ;; )
	{
		xQueueReceive( xStringQueue, &num, portMAX_DELAY );
		if(num == 1)
		{
			PORTD = 0x1; // turn ON first LED
			vTaskDelay(100);// assigning the delay 
		}
		else if(num ==2)
		{
			PORTD = 0x2; // turn ON all second LEDS
			vTaskDelay(100);// keeping the delay as 100ms
		}
		else if(num == 3)
		{
			PORTD = 0x4; // turn ON all third LEDS
			vTaskDelay(100);// assigning the delay 
		}
		else
		{
			// TRISDCLR = 0x07; //clear bits for leds as outputs
			PORTD = 0x4; // turn ON all three LEDS
		}
	}
}

/*-----------------------------------------------------------*/


void vSW1_ISR_Handler( void )
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
static unsigned long ulReceivedNumber;

/*The strings are declared static const to ensure they are not allocated to the
interrupt service routine stack, and exist even when the interrupt service routine
is not executing. */
static const char *pcStrings[] =
{
    "String 0\n",
    "String 1\n",
    "String 2\n",
    "String 3\n"
};

    /* Loop until the queue is empty. */
    while( xQueueReceiveFromISR( xIntegerQueue, &ulReceivedNumber, &xHigherPriorityTaskWoken ) != errQUEUE_EMPTY )
    {
        /* Truncate the received value to the last two bits (values 0 to 3 inc.), then
        send the string    that corresponds to the truncated value to the other
        queue. */
        ulReceivedNumber &= 0x03;
        xQueueSendToBackFromISR( xStringQueue, &pcStrings[ ulReceivedNumber ], &xHigherPriorityTaskWoken );
    }
    /* Clear the software interrupt flag. */
    mainCLEAR_INTERRUPT();
    /* Then clear the interrupt in the interrupt controller. */
    IFS0CLR = mainSW1_INT_BIT;
    
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/

void vSetupEnvironment( void )
{
	/* Setup the main clock for maximum performance, and the peripheral clock
	to equal the main clock divided by 2. */
	SYSTEMConfigPerformance( configCPU_CLOCK_HZ );
	mOSCSetPBDIV( OSC_PB_DIV_2 );
    
    /* Setup the software interrupt used by mainTRIGGER_INTERRUPT(). */
    mConfigIntCoreSW1( CSW_INT_ON | CSW_INT_PRIOR_1 | CSW_INT_SUB_PRIOR_0 );
	
	/* Enable global interrupt handling. */
	INTEnableSystemMultiVectoredInt();

	/* Initialise the debug utils library to enable strings printed from the
	demo source code to be displayed in the MPLAB IDE. */
	DBINIT();
}
/*-----------------------------------------------------------*/



void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNoticeHandler(void)
{
   	PORTD ^= 0x2; //toggle bit 1 LED
    PORTD ^= 0x1; //toggel bit 0 LED
	IFS1CLR = 1; // Be sure to clear the CN interrupt status
}

void configCNInitPullup( unsigned int bits )
{
	CNEN = bits;  // Input Change Notification Interrupt Enable register 
	CNPUE = bits; // Input Change Notification Pull-up Enable register
}
void enableCN()
{
	CNCON = EN_CN; // Enable Interrupt-on-Change Control register
}

