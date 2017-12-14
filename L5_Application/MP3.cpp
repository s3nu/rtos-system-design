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



GPIO dreqPin(P1_19);
GPIO XCS(P1_22);
GPIO XDCS(P1_30);
GPIO decodeRST(P1_28);



bool MP3::init(){

	ssp0_init(0);
	XCS.setAsOutput();  			//set pin P1.22 as output -- XCS signal
    decodeRST.setAsOutput();  		//set pin P1.28 as output -- Decoder RESET signal
    XDCS.setAsOutput();			    //set pin P1.30 as output -- XDCS signal
    dreqPin.setAsInput();  			//set pin P1.19 as input -- DREQ signal


    decodeRST.setLow();
    decodeRST.setHigh();



	return true;
}
	
bool MP3::run(void *param){
	readAndSend();
	return true;
}

void MP3::readAndSend(){
	XCS.setLow();
	char data [512] = {0};
	int i = 0;
	while(1){
		Storage::read("1:Jaden Smith - Icon.mp3", data, sizeof(data)-1, i);
		i += 512;
		for(int j =0; j< 16; j++){
			while(!dreqPin.read());
			XDCS.setLow();
			for(int k=0; k<32; k++){
					ssp0_exchange_byte(data[k+(j*32)]);
				}
			XDCS.setHigh();
		}
	}
	XCS.setHigh();
}
