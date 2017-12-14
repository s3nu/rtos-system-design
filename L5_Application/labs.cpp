#include "tasks.hpp"
#include "examples/examples.hpp"
#include "stdio.h"
#include "printf_lib.h"
#include "i2c2.hpp"
#include <stdint.h>
#include "uart0_min.h"
#include "event_groups.h"
#include "io.hpp"
#include "time.h"
#include "storage.hpp"
#include "command_handler.hpp"
#include <string.h>
#include "uart3.hpp"

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
				"\n******************************************************************************\n");

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
		LPC_UART3->LCR = 3; //DLAB is disable -- set to 8bit transfer
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
			printf(
					"\n******************************************************************************\n");
		}

		if (userinput == '2') {
			for (int i = 0; i < 10; i++) {
				printf("\n %c", uart3_getchar());
			}
			printf(
					"\n******************************************************************************\n");
		}
		return true;
	}

	char uart3_getchar() {
		while (!(LPC_UART3->LSR & (1 << 6)))
			;
		return LPC_UART3->RBR;
	}

	char uart3_putchar(char out) {
		while (!(LPC_UART3->LSR & (1 << 6)))
			;
		LPC_UART3->THR = out;
		return 1;
	}
};

class lab4: public scheduler_task {
#define port0pin0 0
#define port2pin7 1

public:
	lab4() :
		scheduler_task("lab4", 2000, PRIORITY_LOW) {
		printf("lab4 task started");
	}

	static void handler() {
		uint32_t ris = LPC_GPIOINT->IO0IntStatF;
		uint32_t fall = LPC_GPIOINT->IO0IntStatR;
		uint32_t ris2 = LPC_GPIOINT->IO2IntStatR;
		uint32_t fall2 = LPC_GPIOINT->IO2IntStatF;
		uint8_t tempo = 0;
		for (int i = 0; i < 33; i++) {
			if (LPC_GPIOINT->IO0IntStatF & (1 << i)) {
				tempo = i;
				u0_dbg_printf(" interrupt generated %i", tempo);
			}
		}
		for (int i = 0; i < 33; i++) {
			if (LPC_GPIOINT->IO2IntStatR & (1 << i)) {
				tempo = i;
				u0_dbg_printf(" interrupt generated %i", tempo);
			}
		}
		LPC_GPIOINT->IO0IntClr = 0xFFFFFFFF;
		LPC_GPIOINT->IO2IntClr = 0xFFFFFFFF;
	}

	bool init(void) {
		NVIC_EnableIRQ(EINT3_IRQn); //enabling the vector table
		isr_register(EINT3_IRQn, &handler);
		LPC_PINCON->PINSEL0 &= ~(0xFFFF << 1);
		LPC_PINCON->PINSEL1 &= ~(0xFFFF << 1);
		LPC_PINCON->PINSEL4 &= ~(0xFFFF << 1);
		LPC_PINCON->PINSEL5 &= ~(0xFFFF << 1);
		LPC_GPIO0->FIODIR &= ~(1 << port0pin0);
		LPC_GPIO2->FIODIR &= ~(1 << port2pin7);
		LPC_GPIOINT->IO0IntEnF |= (1 << port0pin0);
		LPC_GPIOINT->IO0IntEnR |= (1 << port0pin0);
		LPC_GPIOINT->IO2IntEnR |= (1 << port2pin7);
		LPC_GPIOINT->IO2IntEnR |= (1 << port2pin7);
		return true;
	}

	bool run(void *p) {
		return true;
	}
};

class lab5: public scheduler_task {
public:
	lab5() :
		scheduler_task("lab5", 2000, PRIORITY_LOW) {
		printf("lab5 task started");
	}

	bool init(void) {
		//i2c.slave(slaveadder, buffer, sizeof(buffer));

		return true;
	}

	bool run(void *p) {
		//			I2C2& i2c = I2C2::getInstance(); // Get I2C driver instance
		//			const uint8_t slaveadder = 0x60;  // Pick any address other than the used used at i2c2.hpp
		//			uint8_t buffer[256] = { 0 };     // Our slave read/write buffer
		uint8_t prev = buffer[0];
		while (1) {
			if (prev != buffer[0]) {
				prev = buffer[0];
				u0_dbg_printf("buffer[0] changed to %#x\n", buffer[0]);
			}
		}
		return true;
	}
private:
	I2C2& i2c = I2C2::getInstance(); // Get I2C driver instance
	const uint8_t slaveadder = 0x66; // Pick any address other than the used used at i2c2.hpp
	uint8_t buffer[256] = { 0 };
};

void task1(void* p) {
	while (1) {
		uart0_puts("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
		vTaskDelay(1000);
	}
}
void task2(void* p) {
	while (1) {
		uart0_puts("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB");
		vTaskDelay(1000);
	}
}

EventGroupHandle_t tevent = xEventGroupCreate();
typedef enum {
	shared_SensorQueueId,
} sharedHandleId_t;

class consumertask: public scheduler_task {
public:
	consumertask(uint8_t priority) :
		scheduler_task("consumer", 4096, priority) {
	}

	bool run(void *p) {
		QueueHandle_t qid = getSharedObject(shared_SensorQueueId);
		int light;
		if (xQueueReceive(qid, &light, portMAX_DELAY)) {
			char string1[12];
			char string2[12];
			uint64_t time = sys_get_uptime_ms();
			sprintf(string1, "%lu, ", time);
			sprintf(string2, "%i\n", light);
			strcat(string1, string2);
			Storage::append("1:sensor.txt", string1, sizeof(string1), 0);
		}
		xEventGroupSetBits(tevent, 1 << 2);
		vTaskDelay(1000);
		return true;
	}
};

class producertask: public scheduler_task {
public:
	producertask(uint8_t priority) :
		scheduler_task("producer", 4096, priority) {
		QueueHandle_t my_queue = xQueueCreate(1, 4);
		addSharedObject(shared_SensorQueueId, my_queue);
	}
	bool run(void *p) {
		int light_value[100];
		int sum = 0;
		int average;
		TickType_t xLastWakeTime;
		const TickType_t xFrequency = 1;
		xLastWakeTime = xTaskGetTickCount();
		for (int i = 0; i < 100; i++) {
			vTaskDelayUntil(&xLastWakeTime, xFrequency);
			light_value[i] = LS.getRawValue();
			sum += light_value[i];
		}
		average = sum / 100;
		xQueueSend(getSharedObject(shared_SensorQueueId), &average, portMAX_DELAY);
		xEventGroupSetBits(tevent, 1 << 1);
		return true;
	}
};

class watchdogtask: public scheduler_task {
public:
	watchdogtask(uint8_t priority) :
		scheduler_task("watchdog", 2048, priority) {
	}
	bool run(void *p) {
		uint32_t bit = xEventGroupWaitBits(tevent, ((1 << 1) | (1 << 2)), 1, 1, 1000);
		if (bit == 0) {
			char data1[30] = "stuck at producer task\n";
			Storage::append("1:stuck.txt", data1, sizeof(data1) - 1, 0);
		} else if (bit == 4) {
			char data2[30] = "stuck at consumer task\n";
			Storage::append("1:stuck.txt", data2, sizeof(data2) - 1, 0);
		}
		vTaskDelay(1000);
		return true;
	}
};
