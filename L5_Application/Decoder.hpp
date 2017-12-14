#include "tasks.hpp"



class decoderRegister: public scheduler_task {
	public:
	decoderRegister(): 
		scheduler_task("Decoder", 2048, PRIORITY_HIGH) {}

		char volume;
	
		char regAddress;
		char hiByte;
		char lowByte;
		bool init();
		bool run(void *param);
		void setRegisterValue(char regAddr, char hi, char low);
		void lowerVolume();
		void raiseVolume();

};
