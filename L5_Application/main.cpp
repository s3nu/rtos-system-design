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
#include "time.h"
#include "storage.hpp"
#include "command_handler.hpp"
#include <string.h>
#include "MP3.hpp"
#include "Decoder.hpp"
#include "uart3.hpp"
#include "LCD.hpp"
//#include "labs.cpp"

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

EventGroupHandle_t songevent = xEventGroupCreate();
typedef enum {
	song_name_queue,
} sharedHandleId_t;

class mp3play: public scheduler_task{
public:
	mp3play(uint8_t priority) :
		scheduler_task("mp3play", 2000, priority) {
		QueueHandle_t my_queue = xQueueCreate(1, 4);
		addSharedObject(song_name_queue, my_queue);
	}

	bool run(void *p){
		char song_name[20];
		while(1) {
			if(xQueueReceive(getSharedObject(song_name_queue), &song_name[0], portMAX_DELAY)) {

			}
		}
		return true;

	}
};

class mp3stop: public scheduler_task{
public:
	mp3stop(uint8_t priority) :
		scheduler_task("mp3stop", 2000, priority) {
	}

	bool run(void *p){
		QueueHandle_t qid = getSharedObject(song_name_queue);

		return true;
	}
};

class mp3control: public scheduler_task{
public:
	mp3control(uint8_t priority) :
		scheduler_task("mp3control", 2000, priority) {

	}

	bool run(void *p) {
		readtracklist();
		vTaskDelay(1000);
		return true;
	}
	bool init(void) {
		LPC_GPIO1->FIODIR |= (1<<22);   //set pin P1.22 as output -- CS signal
		LPC_GPIO1->FIODIR |= (1<<28);   //set pin P1.28 as output -- RESET signal
		LPC_GPIO1->FIODIR |= (1<<30);   //set pin P1.30 as output -- DCS signal
		LPC_GPIO1->FIODIR &= ~(1<<19);  //set pin P1.19 as input -- DREQ signal
		LPC_GPIO1->FIOSET = (1<<28);    //set pin P1.28 high initially
		LPC_GPIO1->FIOSET = (1<<30);    //set pin P1.30 high initially
		LPC_GPIO1->FIOSET = (1<<22);    //set pin P1.22 high initially

		LPC_SC->PCONP |= (1 << 10);   	 		// SPI1 Power Enable
		LPC_SC->PCLKSEL0 &= ~(3 << 20); 		// Clear clock Bits
		LPC_SC->PCLKSEL0 |= (1 << 20); 		// CLK / 1

		// Select MISO, MOSI, and SCK pin-select functionality
		LPC_PINCON->PINSEL0 &= ~((3 << 14) | (3 << 16) | (3 << 18));
		LPC_PINCON->PINSEL0 |= ((2 << 14) | (2 << 16) | (2 << 18));
		LPC_SSP1->CR0 = 7;          			// 8-bit mode
		LPC_SSP1->CR1 = (1 << 1);  				// Enable SSP as Master
		LPC_SSP1->CPSR = 8;         			// SCK speed = CPU / 8
		LPC_PINCON->PINSEL0 &= ~(3 << 13); //Select CS
		LPC_GPIO0->FIODIR |= (1 << 6); 		//CS output
		LPC_GPIO0->FIOSET = (1 << 6); 			//CS high (not selected)
		return true;
	}
	void readtracklist() {
		char track[15];
		Storage::read("1:list_of_tracks.txt", track, sizeof(track)-1, 0);
		u0_dbg_printf("track playing %s\n", track);
	}
private:
	uint8_t buffer[256] = { 0 };
};

void led_sw()
{
	if (SW.getSwitch(1)) { /* Check if button 1 is pressed */
		LE.on(1);          /* Turn on LED # 1              */
		LD.setNumber(1);   /* LED display will show "1"    */
	}
	else {
		LE.setAll(0); /* Turn off all LEDs */
		LD.clear();   /* Clear the display */
	}
}

class button_pushed: public scheduler_task {
public:
	button_pushed() :
		scheduler_task("button_pushed", 4098, PRIORITY_HIGH) {
	}

	bool init(void) {
		return true;
	}

	bool run(void *p) {
		led_sw();
		return true;
	}
};
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
	//	scheduler_add_task(new consumertask(PRIORITY_MEDIUM));
	//	scheduler_add_task(new producertask(PRIORITY_MEDIUM));
	//	scheduler_add_task(new watchdogtask(PRIORITY_HIGH));
	//

	//mp3 project
	//scheduler_add_task(new mp3control(PRIORITY_MEDIUM));
	scheduler_add_task(new decoderRegister());
	scheduler_add_task(new MP3());
	scheduler_add_task(new LCD());
	//scheduler_add_task(new mp3stop(PRIORITY_MEDIUM));
	//scheduler_add_task(new button_pushed());

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
