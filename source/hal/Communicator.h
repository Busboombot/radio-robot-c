#pragma once
#include "MicroBit.h"
#include "SerialPort.h"
#include "Radio.h"

/**
 * Communicator — owns both communication wrappers (serial + radio).
 *
 * Constructed in main() with the CODAL peripherals; Robot and LoopScheduler
 * receive a reference so they never see raw NRF52Serial / MicroBitRadio.
 *
 * begin() enables both channels.  Call it once in main() before starting the
 * cooperative loop.
 *
 * Usage (main.cpp):
 *   static Communicator comm(uBit.serial, uBit.radio, uBit.messageBus);
 *   comm.begin();
 */
class Communicator {
public:
    Communicator(NRF52Serial& serial, MicroBitRadio& radio, MessageBus& bus);

    // Call once in main() after uBit.init(): enables serial + radio.
    void begin();

    // Accessors for the two comms HAL objects.
    SerialPort& serial() { return _serial; }
    Radio&      radio()  { return _radio;  }

private:
    SerialPort _serial;
    Radio      _radio;
};
