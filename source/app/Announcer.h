#pragma once
#include "MicroBit.h"
#include "hal/SerialPort.h"
#include "hal/Radio.h"

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

    // Emit the DEVICE: announcement over serial.
    void announce();

    // If line == "HELLO", re-emit announcement and return true.
    // Otherwise return false (caller processes the line normally).
    bool handle(const char* line);

private:
    SerialPort& _serial;
    Radio&      _radio;
    char        _announcement[96];
};
