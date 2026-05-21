#include "app/Announcer.h"
#include <string.h>
#include <stdio.h>

Announcer::Announcer(MicroBit& uBit, SerialPort& serial, Radio& radio)
    : _serial(serial), _radio(radio)
{
    // Build announcement once. ManagedString::toCharArray() returns a const char*
    // valid for the duration of the expression — used immediately inside snprintf().
    snprintf(_announcement, sizeof(_announcement),
             "DEVICE:Nezha2:%s:microbit:%s",
             uBit.getName().toCharArray(),
             uBit.getSerial().toCharArray());
}

void Announcer::announce() {
    _serial.send(_announcement);
}

bool Announcer::handle(const char* line) {
    if (strcmp(line, "HELLO") == 0) {
        announce();
        return true;
    }
    return false;
}
