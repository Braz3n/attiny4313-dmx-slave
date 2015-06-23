#include "./DmxSlave.h"

int main(void) {
    dmxSlaveInit();

    while (1) {
		dmxSlaveUpdateStatus();
        dmxSlaveProcessData();
    }

	return 0;
}
