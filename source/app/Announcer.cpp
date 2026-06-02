#include "Announcer.h"
#include <string.h>
#include <stdio.h>

Announcer::Announcer(MicroBit& uBit, SerialPort& serial, Radio& radio)
    : _serial(serial), _radio(radio)
{
    // Build announcement once.
    // microbit_friendly_name() returns a static char* (5-letter codename from FICR).
    // MicroBit::getSerial() returns a ManagedString; toCharArray() gives const char*.
    snprintf(_announcement, sizeof(_announcement),
             "DEVICE:Nezha2:%s:microbit:%s",
             microbit_friendly_name(),
             uBit.getSerial().toCharArray());
}

void Announcer::announce() {
    _serial.send(_announcement);
}

bool Announcer::handle(const char* line, ReplyFn replyFn, void* ctx) {
    if (strcmp(line, "HELLO") == 0) {
        replyFn(_announcement, ctx);
        return true;
    }
    return false;
}
