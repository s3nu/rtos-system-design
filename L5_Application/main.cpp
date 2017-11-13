/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief This is the application entry point.
 * 			FreeRTOS and stdio printf is pre-configured to use uart0_min.h before main() enters.
 * 			@see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 */
#include "tasks.hpp"
#include "examples/examples.hpp"
#include "stdio.h"
#include "printf_lib.h"
#include "i2c2.hpp"
#include <stdint.h>
#include "uart0_min.h"
#include "event_groups.h"
#include "io.hpp"



/**
 * The main() creates tasks or "threads".  See the documentation of scheduler_task class at scheduler_task.hpp
 * for details.  There is a very simple example towards the beginning of this class's declaration.
 *
 * @warning SPI #1 bus usage notes (interfaced to SD & Flash):
 *      - You can read/write files from multiple tasks because it automatically goes through SPI semaphore.
 *      - If you are going to use the SPI Bus in a FreeRTOS task, you need to use the API at L4_IO/fat/spi_sem.h
 *
 * @warning SPI #0 usage notes (Nordic wireless)
 *      - This bus is more tricky to use because if FreeRTOS is not running, the RIT interrupt may use the bus.
 *      - If FreeRTOS is running, then wireless task may use it.
 *        In either case, you should avoid using this bus or interfacing to external components because
 *        there is no semaphore configured for this bus and it should be used exclusively by nordic wireless.
 */

//GPIO Lab1
class lab1: public scheduler_task {
public:
	lab1() :
		scheduler_task("lab1", 2000, PRIORITY_LOW) {
	}

	bool init(void) {
		/*
		 * Runs once only.
		 * Initializations of directions and GPIO is done here.
		 * Make direction of PORT0.2 as input **example from http://www.socialledge.com/sjsu/index.php?title=Embedded_System_Tutorial_GPIO**
		 * LPC_GPIO0->FIODIR &= ~(1 << 2);
		 */

		LPC_GPIO1->FIODIR &= ~(1 << 9);			//Set P1.9 internal switch input
		LPC_GPIO1->FIODIR |= (1 << 0);			//Set P1.0 internal LED output

		//External Circuit
		LPC_PINCON->PINSEL0 &= ~((1 << 7) | (1 << 6));	//Set P1.19 GPIO
		LPC_PINCON->PINSEL0 &= ~((1 << 9) | (1 << 8));	//Set P1.20 GPIO
		LPC_GPIO1->FIODIR &= ~(1 << 19);	//Set P1.19 external switch input
		LPC_GPIO1->FIODIR |= (1 << 20);			//Set P1.20 external LED output
		return true;
	}

	bool run(void *p) {
		/*
		 * Main task for Lab1 handler
		 * Reads internal and external switch
		 * Powers internal and external LED in relation to switch type
		 */
		vTaskDelay(100); 						//Delay
		if (LPC_GPIO1->FIOPIN & (1 << 9)) {		//Read internal switch P1.9
			LPC_GPIO1->FIOPIN |= (1 << 0);		//Turn on internal LED P1.0
		} else {
			LPC_GPIO1->FIOPIN &= ~(1 << 0);		//Turn off internal LED P1.0
		}
		//External Circuit
		if (LPC_GPIO1->FIOPIN & (1 << 19)) { 	//Read switch P1.19
			LPC_GPIO1->FIOPIN |= (1 << 20); 	//Turn LED off P1.20
		} else {
			LPC_GPIO1->FIOPIN &= ~(1 << 20); 	//turn LED off P1.20
		}
		return true;
	}
};

//SPI Lab2
class lab2: public scheduler_task {
public:
	lab2() :
		scheduler_task("lab2", 2000, PRIORITY_LOW) {
	}

	void spi1_Init() {
		LPC_SC->PCONP |= (1 << 10);   	 		// SPI1 Power Enable
		LPC_SC->PCLKSEL0 &= ~(3 << 20); 		// Clear clock Bits
		LPC_SC->PCLKSEL0 |= (1 << 20); 		// CLK / 1

		// Select MISO, MOSI, and SCK pin-select functionality
		LPC_PINCON->PINSEL0 &= ~((3 << 14) | (3 << 16) | (3 << 18));
		LPC_PINCON->PINSEL0 |= ((2 << 14) | (2 << 16) | (2 << 18));
		LPC_SSP1->CR0 = 7;          			// 8-bit mode
		LPC_SSP1->CR1 = (1 << 1);  				// Enable SSP as Master
		LPC_SSP1->CPSR = 8;         			// SCK speed = CPU / 8
		//		LPC_PINCON->PINSEL0 &= ~( (1 << 13)|(1 << 12) ); //Select CS
		LPC_PINCON->PINSEL0 &= ~(3 << 13); //Select CS
		LPC_GPIO0->FIODIR |= (1 << 6); 		//CS output
		LPC_GPIO0->FIOSET = (1 << 6); 			//CS high (not selected)
	}

	bool run(void *p) {
		/*
		 * Main task for Lab2 handler
		 */
		vTaskDelay(1000); 						//Delay

		printf("\nDevice ID\n");
		LPC_GPIO0->FIOCLR = (1 << 6); 			//CS low
		spi1_ExchangeByte(0x9F); 				//Opcode for Device ID
		for (int i = 0; i < 5; i++) {
			printf("\n %d\t %x", i, spi1_ExchangeByte(0x00));
		}
		LPC_GPIO0->FIOSET = (1 << 6);				//CS high
		vTaskDelay(1000); 						//Delay
		printf(
				"\n******************************************************************************\n");\
				printf("Memory's Status Register\n");
				LPC_GPIO0->FIOCLR = (1 << 6); 			//CS low
				spi1_ExchangeByte(0xD7); 				//Opcode for Status Register
				uint16_t temp, temp1;
				temp = spi1_ExchangeByte(0x00);
				printf("\n %x\t", temp);
				temp1 = spi1_ExchangeByte(0x00);
				printf("\n %x\t", temp1);
				LPC_GPIO0->FIOSET = (1 << 6);				//CS high
				vTaskDelay(1000); 						//Delay
				return true;
	}

	char spi1_ExchangeByte(char out) {
		LPC_SSP1->DR = out;
		while (LPC_SSP1->SR & (1 << 4))
			; // Wait until SSP is busy
		return LPC_SSP1->DR;
	}
};

//UART Lab 3
class lab3: public scheduler_task {
public:
	lab3() :
		scheduler_task("lab3", 2000, PRIORITY_LOW) {
	}

	bool init(void) {
		//UART driver
		LPC_SC->PCONP |= (1 << 25); //UART3 Power Enable
		LPC_SC->PCLKSEL1 &= ~(3 << 19); //CLR CLK
		LPC_SC->PCLKSEL1 |= (1 << 18); // SET CLK
		LPC_UART3->LCR = 0x03; //bits 7:0 -- parity
		LPC_UART3->LCR |= (1 << 7); //DLAB is enabled
		uint16_t dll = 48000000 / (16 * 4800); //calculating the baud
		LPC_UART3->DLL = dll; // setting baud rate
		LPC_UART3-> LCR = 3; //DLAB is disable -- set to 8bit transfer
		LPC_PINCON->PINSEL9 |= ((1 << 25) | (1 << 24)); //set bit 25:24 11 for TXD3
		LPC_PINCON->PINSEL9 |= ((1 << 27) | (1 << 26)); //set bit 27:26 11 for RXD3
		LPC_PINCON->PINMODE9 &= ~((1 << 13) | (1 << 12)); //set bit 25:24 for 00 pull up -- disable pull-down resistor
		LPC_PINCON->PINMODE9 &= ~((1 << 13) | (1 << 12)); //set bit 27:26 for 00 pull up

		return true;
	}

	bool run(void *p) {
		char userinput = '0';
		printf("\n 1: board send");
		printf("\n 2: board receive");
		scanf("%c", &userinput);
		uart3_putchar('q');
		vTaskDelay(1000);
		printf("\n********** %c", uart3_getchar());
		if (userinput == '1') {
			for (int i = 0; i < 10; i++) {
				uart3_putchar('a');
				uart3_putchar('n');
				uart3_putchar('a');
				uart3_putchar('h');
				uart3_putchar('i');
				uart3_putchar('t');
			}
			printf("\n******************************************************************************\n");
		}

		if (userinput == '2') {
			for (int i = 0; i < 10; i++) {
				printf("\n %c", uart3_getchar());
			}
			printf("\n******************************************************************************\n");
		}
		return true;
	}

	char uart3_getchar() {
		while (!(LPC_UART3->LSR & (1 << 6)));
		return LPC_UART3->RBR;
	}

	char uart3_putchar(char out) {
		while (!(LPC_UART3->LSR & (1 << 6)));
		LPC_UART3->THR = out;
		return 1;
	}
};

class lab4: public scheduler_task{
#define port0pin0 0
#define port2pin7 1

public: lab4(): scheduler_task("lab4", 2000, PRIORITY_LOW){
	printf("lab4 task started");
}

static void handler(){
	uint32_t ris = LPC_GPIOINT->IO0IntStatF;
	uint32_t fall = LPC_GPIOINT->IO0IntStatR;
	uint32_t ris2  = LPC_GPIOINT->IO2IntStatR;
	uint32_t fall2 = LPC_GPIOINT->IO2IntStatF;
	uint8_t tempo = 0;
	for (int i=0; i<33; i++){
		if(LPC_GPIOINT->IO0IntStatF & (1<<i)){
			tempo = i;
			u0_dbg_printf(" interrupt generated %i", tempo);
		}
	}
	for (int i=0; i<33; i++){
		if(LPC_GPIOINT->IO2IntStatR & (1<<i)){
			tempo = i;
			u0_dbg_printf(" interrupt generated %i", tempo);
		}
	}
	LPC_GPIOINT->IO0IntClr = 0xFFFFFFFF;
	LPC_GPIOINT->IO2IntClr = 0xFFFFFFFF;
}

bool init(void){
	NVIC_EnableIRQ(EINT3_IRQn); //enabling the vector table
	isr_register(EINT3_IRQn, &handler);
	LPC_PINCON->PINSEL0 &= ~(0xFFFF<<1);
	LPC_PINCON->PINSEL1 &= ~(0xFFFF<<1);
	LPC_PINCON->PINSEL4 &= ~(0xFFFF<<1);
	LPC_PINCON->PINSEL5 &= ~(0xFFFF<<1);
	LPC_GPIO0->FIODIR &= ~(1 <<port0pin0);
	LPC_GPIO2->FIODIR &= ~(1 << port2pin7);
	LPC_GPIOINT->IO0IntEnF |= (1<<port0pin0);
	LPC_GPIOINT->IO0IntEnR |= (1<<port0pin0);
	LPC_GPIOINT->IO2IntEnR |= (1<<port2pin7);
	LPC_GPIOINT->IO2IntEnR |= (1<<port2pin7);
	return true;
}

bool run(void *p){
	return true;
}
};


class lab5: public scheduler_task{
public: lab5(): scheduler_task("lab5", 2000, PRIORITY_LOW){
	printf("lab5 task started");
}

bool init(void){
	i2c.slave(slaveadder, buffer, sizeof(buffer));

	return true;
}

bool run(void *p){
	//			I2C2& i2c = I2C2::getInstance(); // Get I2C driver instance
	//			const uint8_t slaveadder = 0x60;  // Pick any address other than the used used at i2c2.hpp
	//			uint8_t buffer[256] = { 0 };     // Our slave read/write buffer
	uint8_t prev = buffer[0];
	while(1)
	{
		if (prev != buffer[0]) {
			prev = buffer[0];
			u0_dbg_printf("buffer[0] changed to %#x\n", buffer[0]);
		}
	}
	return true;
}
private:
I2C2& i2c = I2C2::getInstance(); // Get I2C driver instance
const uint8_t slaveadder = 0x66;  // Pick any address other than the used used at i2c2.hpp
uint8_t buffer[256] = { 0 };
};

void task1(void* p) {
	while(1) {
		uart0_puts("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
		vTaskDelay(1000);
	}
}
void task2(void* p) {
	while(1) {
		uart0_puts("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB");
		vTaskDelay(1000);
	}
}

QueueHandle_t light_handle = 0;
EventGroupHandle_t tevent;
tevent = xEventGroupCreate();
static uint8_t args;
args = 0;
TaskHandle_t handler = NULL;
uint32_t STACK_SIZE = 2048;

int light_sensor_avg(){
	int light = 0;
	for (int i = 0; i < 100; ++i)
	{
		light += LS.getPercentValue();
		printf("%i\n", light);
	}
	return (light/100)
}

void producer(void *prod){
	int light = light_sensor_avg();
}

int main(void) {
	/**
	 * A few basic tasks for this bare-bone system :
	 *      1.  Terminal task provides gateway to interact with the board through UART terminal.
	 *      2.  Remote task allows you to use remote control to interact with the board.
	 *      3.  Wireless task responsible to receive, retry, and handle mesh network.
	 *
	 * Disable remote task if you are not using it.  Also, it needs SYS_CFG_ENABLE_TLM
	 * such that it can save remote control codes to non-volatile memory.  IR remote
	 * control codes can be learned by typing the "learn" terminal command.
	 */

	//	scheduler_add_task(new lab1());
	//scheduler_add_task(new lab2());
	//scheduler_add_task(new lab3());
	//scheduler_add_task(new lab4());
	//scheduler_add_task(new lab5());
	//scheduler_add_task(new lab6());

	//lab7
	// EventGroupHandle_t tevent;
	// tevent = xEventGroupCreate();
	// static uint8_t args;
	// args = 0;
	// TaskHandle_t handler = NULL;
	// uint32_t STACK_SIZE = 2048;
	// xTaskCreate( task1, "task_1", STACK_SIZE, &args, PRIORITY_HIGH,  &handler);
	// vTaskDelay(50);
	// xTaskCreate( task2, "task_2", STACK_SIZE, &args, PRIORITY_HIGH, &handler);
	// vTaskStartScheduler();


	//lab8
	xTaskCreate(producer, "producer", STACK_SIZE, &args, PRIORITY_MEDIUM,  &handler);




	scheduler_add_task(new terminalTask(PRIORITY_HIGH));

	/* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
	scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));

	/* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */
#if 0
	scheduler_add_task(new periodicSchedulerTask());
#endif

	/* The task for the IR receiver */
	// scheduler_add_task(new remoteTask  (PRIORITY_LOW));
	/* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
	 * task to always be responsive so you can poke around in case something goes wrong.
	 */

	/**
	 * This is a the board demonstration task that can be used to test the board.
	 * This also shows you how to send a wireless packets to other boards.
	 */
#if 0
	scheduler_add_task(new example_io_demo());
#endif

	/**
	 * Change "#if 0" to "#if 1" to enable examples.
	 * Try these examples one at a time.
	 */
#if 0
	scheduler_add_task(new example_task());
	scheduler_add_task(new example_alarm());
	scheduler_add_task(new example_logger_qset());
	scheduler_add_task(new example_nv_vars());
#endif

	/**
	 * Try the rx / tx tasks together to see how they queue data to each other.
	 */
#if 0
	scheduler_add_task(new queue_tx());
	scheduler_add_task(new queue_rx());
#endif

	/**
	 * Another example of shared handles and producer/consumer using a queue.
	 * In this example, producer will produce as fast as the consumer can consume.
	 */
#if 0
	scheduler_add_task(new producer());
	scheduler_add_task(new consumer());
#endif

	/**
	 * If you have RN-XV on your board, you can connect to Wifi using this task.
	 * This does two things for us:
	 *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
	 *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
	 *
	 * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
	 * @code
	 *     // Assuming Wifly is on Uart3
	 *     addCommandChannel(Uart3::getInstance(), false);
	 * @endcode
	 */
#if 0
	Uart3 &u3 = Uart3::getInstance();
	u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
	scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
#endif

	scheduler_start(); ///< This shouldn't return
	return -1;
}
