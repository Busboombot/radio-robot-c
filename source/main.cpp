#include "Robot.h"

// MicroBit uBit singleton lives inside Robot as its first member.
// There is no global uBit here — it is encapsulated in the Robot class
// to guarantee correct construction order for all subsystems.
static Robot robot;

int main() {
    robot.run();
    return 0;
}
