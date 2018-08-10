/*
 * LCD.h
 *
 *  Created on: Dec 13, 2017
 *      Author: Anahit Sarao
 */

#ifndef L5_APPLICATION_LCD_HPP_
#define L5_APPLICATION_LCD_HPP_

#include "tasks.hpp"
#include "gpio.hpp"



class LCD: public scheduler_task {
public:
	LCD():
		scheduler_task("LCD", 4098, PRIORITY_HIGH){}
	bool run(void *param);
	bool init(void *param);
	void push_to_lcd(char line);
friend class MP3;
};

#endif /* L5_APPLICATION_LCD_HPP_ */
