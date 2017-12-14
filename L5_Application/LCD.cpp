/*
 * LCD.cpp
 *
 *  Created on: Dec 13, 2017
 *      Author: Anahit Sarao
 */

#include "LCD.hpp"
#include "MP3.hpp"
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
#include "ssp0.h"
#include "Decoder.hpp"
#include "ff.h"
#include "uart3.hpp"



bool LCD::run(){

	return true;
}

bool LCD::init(){
	vTaskDelay(1000);
	Uart3& u3 = Uart3::getInstance();
	u3.init(38400,50,50);
	u3.putline("LCD is initialized");
	return true;
}
void LCD::push_to_lcd(char line){
	Uart3& u3 = Uart3::getInstance();
	u3.init(38400,50,50);
	u3.putline(&line);
}
