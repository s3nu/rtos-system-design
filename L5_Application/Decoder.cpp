#include "Decoder.hpp"
#include "gpio.hpp"
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


void decoderRegister::setRegisterValue(char regAddr, char hi, char low){
	while(!dreqPin.read());

	XCS.setLow();    			//set pin XDCS low 
   	ssp0_exchange_byte(Write);	//send write signal to decoder
   	ssp0_exchange_byte(regAddr);
   	ssp0_exchange_byte(hi);
   	ssp0_exchange_byte(low);
   	XCS.setHigh();
}

bool decoderRegister::init(){
	setRegisterValue(Mode, 0x48, 0x42);
	setRegisterValue(Clockf, 0x60, 0x00);
	setRegisterValue(Vol, 0x40, 0x40);
	return true;
}