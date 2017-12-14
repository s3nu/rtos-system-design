
#include "tasks.hpp"
#include "gpio.hpp"



extern GPIO dreqPin;
extern GPIO XCS;
extern GPIO XDCS;
extern GPIO decodeRST;


class MP3: public scheduler_task {
public:
	MP3():
		scheduler_task("MP3", 2048, PRIORITY_HIGH) {}
	bool run(void *param);
	bool init();
	void readAndSend();
	void play();
	void pause();
};
