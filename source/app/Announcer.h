#pragma once
#include "MicroBit.h"
#include "SerialPort.h"
#include "Radio.h"
#include "Protocol.h"

/**
 * Announcer — builds and emits the DEVICE: announcement string.
 *
 * Announcement format: DEVICE:Nezha2:<name>:microbit:<serial>
 *   name   = uBit.getName()   — 5-letter codename from nRF52 FICR
 *   serial = uBit.getSerial() — unique serial number as decimal string
 *
 * The announcement string is built once in the constructor and stored in
 * _announcement[96]. announce() and handle() reuse that buffer without
 * reformatting.
 */
class Announcer {
public:
    Announcer(MicroBit& uBit, SerialPort& serial, Radio& radio);

    // Emit the DEVICE: announcement over serial (boot banner).
    void announce();

    // If line == "HELLO", re-emit the announcement via replyFn (so a HELLO
    // arriving over the radio relay is answered over the radio, and a serial
    // HELLO over serial) and return true. Otherwise return false (caller
    // processes the line normally).
    bool handle(const char* line, ReplyFn replyFn, void* ctx);

private:
    SerialPort& _serial;
    Radio&      _radio;
    char        _announcement[96];
};
