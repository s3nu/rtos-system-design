#include "Decoder.hpp"
#include "gpio.hpp"
#include "MP3.hpp"
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

enum decoderRegisterValues{
	//Write or read to/from registers
	Write		= 0x02,	
	Read 		= 0x03,
	//Register values
	Mode 		= 0x00,
	Status 		= 0x01,
	Bass 		= 0x02,
	Clockf 		= 0x03,
	DecodeTime 	= 0x04,
	Audata 		= 0x05,
	WRAM 		= 0x06,
	WRAMAddr 	= 0x07,
	Hdat0 		= 0x08,
	Hdat1 		= 0x09,
	AIAddr 		= 0x0A,
	Vol 		= 0x0B,
	AICtrl0 	= 0x0C,
	AICtrl1 	= 0x0D,
	AICtrl2 	= 0x0E,
	AICtrl3 	= 0x0F
};

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
	volume = 0x40;
	setRegisterValue(Mode, 0x48, 0x42);
	setRegisterValue(Clockf, 0x60, 0x00);
	setRegisterValue(Vol, volume, volume);
	return true;
}

bool decoderRegister::run(void *param){
	while(1){
	}
	return true;
}

void decoderRegister::lowerVolume(){
	if(volume>= 0x10){
		volume = volume - 0x10;
	}
	while(!dreqPin.read());

	XCS.setLow();    			//set pin XDCS low 
   	ssp0_exchange_byte(Read);	//send write signal to decoder
   	ssp0_exchange_byte(Vol);
   	ssp0_exchange_byte(volume);
   	ssp0_exchange_byte(volume);
   	XCS.setHigh();
}

void decoderRegister::raiseVolume(){
	if(volume<= 0xF0){
		volume = volume + 0x10;
	}
	while(!dreqPin.read());

	XCS.setLow();    			//set pin XDCS low 
   	ssp0_exchange_byte(Read);	//send write signal to decoder
   	ssp0_exchange_byte(Vol);
   	ssp0_exchange_byte(volume);
   	ssp0_exchange_byte(volume);
   	XCS.setHigh();
}
