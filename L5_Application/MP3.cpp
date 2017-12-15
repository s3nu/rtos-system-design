// #include "MP3.hpp"
// #include "examples/examples.hpp"
// #include "stdio.h"
// #include "printf_lib.h"
// #include "i2c2.hpp"
// #include <stdint.h>
// #include "uart0_min.h"
// #include "event_groups.h"
// #include "io.hpp"
// #include "time.h"
// #include "storage.hpp"
// #include "command_handler.hpp"
// #include "ssp0.h"
// #include "Decoder.hpp"
// #include <iostream>
// #include <vector>
// #include <string.h>

// GPIO dreqPin(P1_19);
// GPIO XCS(P1_22);
// GPIO XDCS(P1_30);
// GPIO decodeRST(P1_28);
// GPIO playOrPausePin(P2_0);
// GPIO volumeUp(P2_1);
// GPIO volumeDown(P2_2);
// char* list_of_songs[100];

// bool MP3::init(){
	

// 	ssp0_init(0);
// 	XCS.setAsOutput();  			//set pin P1.22 as output -- XCS signal
//     decodeRST.setAsOutput();  		//set pin P1.28 as output -- Decoder RESET signal
//     XDCS.setAsOutput();			    //set pin P1.30 as output -- XDCS signal
//     dreqPin.setAsInput();  			//set pin P1.19 as input -- DREQ signal
//     playOrPausePin.setAsInput();
//     volumeUp.setAsInput();
//     volumeDown.setAsInput();

   


//     decodeRST.setLow();
//     decodeRST.setHigh();

//     int i =0;
//     //const char* list_of_songs[100];
	
// 	FILE *file = fopen("1:Music.txt", "r");
// 	if (file != NULL) {
// 		char line [64];
// 		while(fgets(line,sizeof (line)-1,file)!= NULL)  read a line from a file 
// 		{
// 			list_of_songs[i] = *line;
// 			u0_dbg_printf(" Song name %s \n", list_of_songs[i]);
// 			//fprintf(stdout,"%s",line); //print the file contents on stdout.
// 		}
// 		fclose(file);
// 		}
// 		else {
// 		//perror("Music.txt") 
//     	//print the error message on stderr.
// 		}
// 	//playOrPauseBool = true;
// 	return true;
// }
	
// bool MP3::run(void *param){
// 	decoderRegister decoder;
// 	eint3_enable_port2(1, eint_rising_edge, &decoder.raiseVolume);
// 	eint3_enable_port2(2, eint_rising_edge, &decoder.lowerVolume);
// 	//MP3 mp3Player;
// 	//eint3_enable_port2(0, eint_rising_edge, mp3Player.playOrPause(playOrPauseBool));
// 	readAndSend();
// 	return true;
// }

// // void_func_t MP3::playOrPause(bool playOrPauseBool){
// // 	if(playOrPauseBool){
// // 		playOrPauseBool = false;
// // 	}
// // 	else{
// // 		playOrPauseBool = true;
// // 	}
// // }


// void MP3::readAndSend(){
// 	XCS.setLow();
	
// 	char data [512] = {0};
// 	int i = 0;
// 	while(1){
// 		Storage::read(&list_of_songs[i], data, sizeof(data)-1, i);
// 		//if(playOrPauseBool){
// 			i += 512;
// 			for(int j =0; j< 16; j++){
// 				while(!dreqPin.read());
// 				XDCS.setLow();
// 				for(int k=0; k<32; k++){
// 						ssp0_exchange_byte(data[k+(j*32)]);
// 					}
// 				XDCS.setHigh();
// 			}
// 		//}
// 	}
// 	XCS.setHigh();
// }


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
#include "LCD.hpp"
#include "uart3.hpp"
#include "Decoder.hpp"


GPIO dreqPin(P1_19);
GPIO XCS(P1_22);
GPIO XDCS(P1_30);
GPIO decodeRST(P1_28);
GPIO playPin(P2_0);
GPIO pausePin(P2_1);
int counter = 0;
bool paused = false;

const char *songNames[]{
	"1:01 Keep the Family Close.mp3",
	"1:02 9.mp3",
	"1:03 U With Me.mp3",
	"1:04 Feel No Ways.mp3",
	"1:05 Hype.mp3",
	"1:06 Weston Road Flows.mp3",
	"1:07 Redemption.mp3",
	"1:08 With You.mp3",
	"1:09 Faithful.mp3",
	"1:10 Still Here.mp3",
	"1:11 Controlla.mp3",
	"1:12 One Dance.mp3",
	"1:13 Grammys.mp3",
	"1:14 Childs Play.mp3",
	"1:15 Pop Style.mp3",
	"1:16 Too Good.mp3",
	"1:17 Summers Over Interlude.mp3",
	"1:18 Fire & Desire.mp3",
	"1:19 Views.mp3",
	"1:20 Hotline Bling.mp3"
};


bool MP3::init(){

	ssp0_init(0);
	XCS.setAsOutput();  			//set pin P1.22 as output -- XCS signal
    decodeRST.setAsOutput();  		//set pin P1.28 as output -- Decoder RESET signal
    XDCS.setAsOutput();			    //set pin P1.30 as output -- XDCS signal
    dreqPin.setAsInput();  			//set pin P1.19 as input -- DREQ signal

    playPin.setAsInput();
	pausePin.setAsInput();
    decodeRST.setLow();
    decodeRST.setHigh();

    
 //    Uart3& u3 = Uart3::getInstance();
	// u3.init(38400,50,50);

	return true;
}
	
bool MP3::run(void *param){
	

	readAndSend();
	return true;
}

void MP3::readAndSend(){

		Uart3& u3 = Uart3::getInstance();
		u3.init(38400,50,50);
	
		decoderRegister decoderObject;
		XCS.setLow();
		char data [512] = {0};
		int i = 0;
		u3.putline(songNames[counter]);
		while(1){
				
				Storage::read(songNames[counter], data, sizeof(data)-1, i);

				i += 512;
				for(int j =0; j< 16; j++){
					while(!dreqPin.read());
					XDCS.setLow();
					for(int k=0; k<32; k++){
						if(pausePin.read()){
						while(!playPin.read());
					}
							ssp0_exchange_byte(data[k+(j*32)]);
						}
					XDCS.setHigh();
				}
			}
			XCS.setHigh();
		}
	
	
