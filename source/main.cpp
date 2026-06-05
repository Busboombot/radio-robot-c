#include "MicroBit.h"
#include "Robot.h"
#include "CommandProcessor.h"
#include "LoopScheduler.h"
#include "SerialPort.h"
#include "Icons.h"

// ---------------------------------------------------------------------------
// MicroBit uBit singleton — must be file-scope so CODAL peripherals are
// fully initialised before Robot is constructed in main().
// ---------------------------------------------------------------------------
static MicroBit uBit;

// ---------------------------------------------------------------------------
// serialReply — thin adapter used for the boot HELLO banner.
// ---------------------------------------------------------------------------
static void serialReply(const char* msg, void* ctx)
{
    static_cast<SerialPort*>(ctx)->send(msg);
}

// ---------------------------------------------------------------------------
// main — constructs the robot and runs the single cooperative main loop.
//
// Single cooperative main loop architecture (014-006/007):
//
//   LoopScheduler::run() (never returns):
//     1. HARD TASK: split-phase encoder COLLECT → velocity (ZOH) → PID → PWM.
//     2. LOW-PRIORITY SWEEP: comms-in, drive-advance, odometry-predict,
//        otos-correct, line-read, color-read, ports-read, telemetry-emit.
//        Round-robin, persistent cursor, budget-gated against controlDeadline.
//     3. ENCODER REQUEST: fire next wheel request (last I2C before idle).
//     4. IDLE SLEEP: sleep until controlDeadline.
//
// No CODAL fibers. All I/O inline. All task entry points on Robot.
// ---------------------------------------------------------------------------

int main() {
    uBit.init();

    // Force the I2C bus to 100 kHz (matches the known-good MakeCode firmware).
    // The CODAL default can be faster; at higher speed the OTOS (0x17) read
    // wedges the shared bus once a second sensor (color 0x43) is present —
    // writes survive, reads return 0. 100 kHz gives the margin the loaded bus
    // needs. (Sprint 014 color/OTOS bus-conflict fix.)
    uBit.i2c.setFrequency(100000);

    // Show a heart on the 5x5 LED matrix as a "powered and ready" indicator.
    uBit.display.printAsync(icons::boot()); // delay=0 → show forever, non-blocking

    static Robot            robot(uBit.i2c, uBit.serial, uBit.radio,
                                  uBit.io, uBit.messageBus, uBit);
    static CommandProcessor cmd(robot);

    // Emit DEVICE: identification banner once at boot over serial.
    cmd.process("HELLO", serialReply, &robot.serialPort());

    // Run the cooperative main loop — never returns.
    // run_tasks() = production priority-task loop; run_all() = explicit testing loop.
    static LoopScheduler sched(robot, cmd, uBit);
    cmd.setScheduler(&sched);   // enable DBG LOOP <x> <state> task toggling
    sched.run_all();            // testing loop (per-task toggles + timing)

    return 0;
}
