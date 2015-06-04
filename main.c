#include "./DmxSlave.h"

int main(void) {
    dmxSlaveInit(11);

    while (1) {
		dmxSlaveUpdateStatus();
        dmxSlaveProcessData();
    }
	
	return 0;
}
