#include "tasks.hpp"

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


class decoderRegister: public scheduler_task {
public:
	decoderRegister(): 
		scheduler_task("Decoder", 2048, PRIORITY_HIGH) {
	}

	bool init();
	bool run(void *param);
	void setRegisterValue(char regAddr, char hi, char low);
	char regAddress;
	char hiByte;
	char lowByte;
};
