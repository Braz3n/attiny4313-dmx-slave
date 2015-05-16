#include "./DmxSlave.h"

void main(void) {
    dmxSlaveInit(4);

    while (1) {
        dmxSlaveProcessData();
    }
}
