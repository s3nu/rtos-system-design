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
#include "LCD.hpp"
#include "vector.hpp"





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
    //LCD* object;
    //FILE *fd = fopen("1:Music.txt", "r");
    //int size;
	/*char *array = new *song_list[128];
	if (fd)
		{
			fgets(song_list, sizeof(line)-1, fd);
			LCD::push_to_lcd(line);
			fclose(fd);
		}*/
	return true;
}
	
bool MP3::run(void *param)
{
	/*if(SW.getSwitch(1))
	{
		LE.on(1);
		vTaskDelay(3000);
		LD.clear();
	}
	else*/
	readAndSend();
	return true;
}

void MP3::readAndSend(){
	VECTOR<char> list_of_songs;

	FILE *file = fopen("1:Music.txt", "r");
	if (file != NULL) {
	  char line [1000];
	  while(fgets(line,sizeof line,file)!= NULL) /* read a line from a file */
	  {
		  //list_of_songs.push_back(line);
	      //fprintf(stdout,"%s",line); //print the file contents on stdout.
	    }
	    fclose(file);
	  }
	  else {
	    perror("Music.txt"); //print the error message on stderr.
	  }

	XCS.setLow();
	char data [512] = {0};
	int i = 0;
	while(1){
		Storage::read("1:01 Keep the Family Close.mp3", data, sizeof(data)-1, i);
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
void MP3::play(){




}

void MP3::pause(){



}





