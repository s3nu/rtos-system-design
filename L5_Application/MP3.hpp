

#include "tasks.hpp"
#include "gpio.hpp"
#include "ff.h"
#include "circular_buffer.hpp"



extern GPIO dreqPin;
extern GPIO XCS;
extern GPIO XDCS;
extern GPIO decodeRST;
extern GPIO playOrPausePin;
extern GPIO nextSong;
extern int counter;
extern bool notPause;

class MP3: public scheduler_task {
public:
	MP3() :
		scheduler_task("MP3", 2048, PRIORITY_HIGH) {}
	bool run(void *param);
	bool init();
	void readAndSend();
};
