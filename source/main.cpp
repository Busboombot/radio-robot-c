#include "MicroBit.h"
#include "Robot.h"
#include "CommandProcessor.h"
#include "SerialPort.h"
#include "Radio.h"

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
// main — constructs the robot, then runs the visible main loop.
//
// Reply-sink routing (fixes the async-completion channel bug):
//   activeFn / activeCtx are updated to whichever channel (serial or radio)
//   delivered the most recent command.  robot.tick() is then called with
//   that active sink, so async completions (T+DONE, D+DONE, G+DONE,
//   SAFETY_STOP) and encoder streaming are returned over the SAME channel
//   the originating command arrived on — not hardwired to serial.
// ---------------------------------------------------------------------------

int main() {
    uBit.init();

    static Robot            robot(uBit.i2c, uBit.serial, uBit.radio,
                                  uBit.io, uBit.messageBus, uBit);
    static CommandProcessor cmd(robot);

    // Alias the HAL objects out of Robot for the reply-sink ctxs.
    SerialPort& serial = robot.serialPort();
    Radio&      radio  = robot.radioPort();

    // Active reply sink — initialised to serial; updated each time a command
    // is dispatched so robot.tick() sends completions to the right channel.
    ReplyFn activeFn  = serialReply;
    void*   activeCtx = &serial;

    char buf[512];

    while (true) {
        // Drain serial — commands arrive directly from a USB/UART host.
        while (serial.readLine(buf, sizeof(buf))) {
            activeFn  = serialReply;
            activeCtx = &serial;
            if (!robot.announcer().handle(buf, serialReply, &serial))
                cmd.process(buf, serialReply, &serial);
        }

        // Drain radio — commands arrive via the RadioRelay; replies must
        // go back over radio so the relay can forward them to the host.
        while (radio.poll(buf, sizeof(buf))) {
            activeFn  = radioReply;
            activeCtx = &radio;
            if (!robot.announcer().handle(buf, radioReply, &radio))
                cmd.process(buf, radioReply, &radio);
        }

        // Advance drive state machines; completions go to the active sink.
        robot.tick(uBit.systemTime(), activeFn, activeCtx);

        uBit.sleep((uint32_t)robot.config().tickMs);
    }

    return 0;
}
