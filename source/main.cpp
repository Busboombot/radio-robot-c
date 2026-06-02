#include "MicroBit.h"
#include "Robot.h"

// MicroBit uBit singleton lives here as the file-scope owner.
// uBit MUST be constructed before Robot so all CODAL peripherals are
// ready when Robot stores references to them.
static MicroBit uBit;

int main() {
    uBit.init();
    static Robot robot(uBit.i2c, uBit.serial, uBit.radio, uBit.io, uBit.messageBus, uBit);
    robot.run();
    return 0;
}
