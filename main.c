#include "./DmxSlave.h"

int main(void) {
    dmxSlaveInit(11);

    while (1) {
		dmxSlaveUpdateStatus();
        dmxSlaveProcessData();
		#warning debug
		PORTD ^= 0x02;
    }
	
	return 0;
}
