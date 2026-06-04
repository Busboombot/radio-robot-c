#include "MicroBit.h"
#include "Robot.h"
#include "CommandProcessor.h"
#include "SerialPort.h"
#include "Radio.h"
#include "Icons.h"

// ---------------------------------------------------------------------------
// MicroBit uBit singleton — must be file-scope so CODAL peripherals are
// fully initialised before Robot is constructed in main().
// ---------------------------------------------------------------------------
static MicroBit uBit;

// ---------------------------------------------------------------------------
// Reply sinks — thin adapters from the (const char*, void*) ReplyFn
// signature to the HAL send() methods.
// ---------------------------------------------------------------------------

static void serialReply(const char* msg, void* ctx) {
    static_cast<SerialPort*>(ctx)->send(msg);
}

static void radioReply(const char* msg, void* ctx) {
    static_cast<Radio*>(ctx)->send(msg);
}

// ---------------------------------------------------------------------------
// Shared pointers to the two application singletons — needed by the control
// fiber trampoline which has only a void* arg.
// ---------------------------------------------------------------------------
static Robot*            gRobot = nullptr;

// ---------------------------------------------------------------------------
// controlFiberFn — runs on a dedicated CODAL fiber.
//
// This is the HIGH-PRIORITY deterministic path:
//   encoder reads (I2C, now busy-wait) → PID → Motor::setSpeed → odometry
//
// It does NOT touch serial or radio.  All EVT completions (done, safety_stop)
// are enqueued into DriveController's ring buffer; the comms fiber drains
// them via telemetryTick().
//
// Sleep duration: RobotConfig::controlPeriodMs (default 10 ms).
// Actual period = controlPeriodMs + I2C busy-wait cost (~16 ms for two
// encoder reads at 4+4 ms each), so effective rate ≈ ~40 Hz at default.
// ---------------------------------------------------------------------------
static void controlFiberFn(void* /*arg*/)
{
    while (true) {
        gRobot->controlTick(uBit.systemTime());
        uBit.sleep((uint32_t)gRobot->config().controlPeriodMs);
    }
}

// ---------------------------------------------------------------------------
// main — constructs the robot, spawns the control fiber, then runs the
// comms+telemetry loop on the main fiber.
//
// Two-fiber architecture (013-010):
//
//   Control fiber (spawned via create_fiber):
//     Runs controlFiberFn() in a tight loop:
//       robot.controlTick() → uBit.sleep(controlPeriodMs)
//     Executes: encoder I2C → PID → setSpeed + drive-mode logic.
//     No serial/radio, no snprintf, no telemetry.
//
//   Comms+telemetry fiber (this loop — the original main loop):
//     Drains serial and radio command queues → dispatches commands.
//     Calls robot.telemetryTick() which:
//       (a) drains DriveController EVT ring (emits done/safety_stop);
//       (b) assembles + sends TLM frame when tlmPeriodMs elapses.
//     Sleeps commsPeriodMs (5 ms) per iteration.
//
// Reply-sink routing: activeFn/activeCtx track whichever channel delivered
// the most recent command so that telemetry and EVT completions are returned
// over the same channel that originated the command.
// ---------------------------------------------------------------------------

int main() {
    uBit.init();

    // Show a heart on the 5x5 LED matrix as a "powered and ready" indicator.
    uBit.display.printAsync(icons::boot()); // delay=0 → show forever, non-blocking

    static Robot            robot(uBit.i2c, uBit.serial, uBit.radio,
                                  uBit.io, uBit.messageBus, uBit);
    static CommandProcessor cmd(robot);

    // Publish the robot pointer for the control fiber trampoline.
    gRobot = &robot;

    // Alias the HAL objects out of Robot for the reply-sink ctxs.
    SerialPort& serial = robot.serialPort();
    Radio&      radio  = robot.radioPort();

    // Active reply sink — initialised to serial; updated each time a command
    // is dispatched so telemetryTick() sends completions to the right channel.
    ReplyFn activeFn  = serialReply;
    void*   activeCtx = &serial;

    // Emit DEVICE: identification banner once at boot.
    cmd.process("HELLO", serialReply, &serial);

    // Spawn the control fiber — runs at fixed period (controlPeriodMs).
    // CODAL create_fiber() takes a void(*)(void*) trampoline and a void* arg.
    create_fiber(controlFiberFn, (void*)nullptr);

    char buf[512];

    // Comms period: how often the main fiber wakes to drain serial/radio and
    // emit telemetry.  5 ms gives ~200 Hz max command throughput; lateness is
    // acceptable (best-effort path).
    static constexpr uint32_t kCommsPeriodMs = 5;

    while (true) {
        // Drain serial — commands arrive directly from a USB/UART host.
        while (serial.readLine(buf, sizeof(buf))) {
            activeFn  = serialReply;
            activeCtx = &serial;
            cmd.process(buf, serialReply, &serial);
        }

        // Drain radio — commands arrive via the RadioRelay; replies must
        // go back over radio so the relay can forward them to the host.
        while (radio.poll(buf, sizeof(buf))) {
            activeFn  = radioReply;
            activeCtx = &radio;
            cmd.process(buf, radioReply, &radio);
        }

        // Drain EVT completions and emit periodic TLM (reads cached data
        // published by the control fiber — no direct motor I2C here).
        robot.telemetryTick(uBit.systemTime(), activeFn, activeCtx);

        uBit.sleep(kCommsPeriodMs);
    }

    return 0;
}
