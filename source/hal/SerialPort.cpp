#include "SerialPort.h"
#include <string.h>
#include <stdio.h>

SerialPort::SerialPort(NRF52Serial& serial)
    : _serial(serial), _rxLen(0)
{
    memset(_rxBuf, 0, sizeof(_rxBuf));
}

void SerialPort::begin() {
    _serial.setRxBufferSize(255);
    // TX buffer size is a uint8_t in CODAL — 255 is the max (1024 wraps to 0!).
    // Bursts must therefore fit in 255 bytes: keep replies to a single line and
    // don't fire many sends back-to-back (the firmware blocks/loses output when
    // the TX buffer can't absorb a burst).
    _serial.setTxBufferSize(255);
    _serial.setBaud(115200);
}

bool SerialPort::readLine(char* buf, uint16_t len) {
    int c;
    while ((c = _serial.read(ASYNC)) != MICROBIT_NO_DATA) {
        if (c == '\r') continue;
        if (c == '\n') {
            _rxBuf[_rxLen] = '\0';
            uint16_t copy = (_rxLen < len - 1) ? _rxLen : (len - 1);
            memcpy(buf, _rxBuf, copy);
            buf[copy] = '\0';
            _rxLen = 0;
            return true;
        }
        if (_rxLen < sizeof(_rxBuf) - 1)
            _rxBuf[_rxLen++] = (char)c;
    }
    return false;
}

void SerialPort::send(const char* msg) {
    // ASYNC: queue what fits in the TX buffer and return IMMEDIATELY. Never block
    // the cooperative loop waiting for the host to drain the buffer. The default
    // (SYNC_SLEEP) fiber-sleeps until the buffer has room — under a telemetry
    // flood (host not reading fast enough) that stalls the whole loop, which
    // starves the S-keepalive (→ watchdog safety_stop = the velocity "notches" /
    // momentary stops) and, if the reader stops entirely, hard-hangs the firmware.
    // Trade-off: under flood a telemetry frame may be dropped (host just sees a
    // truncated line and skips it) — acceptable; a stalled control loop is not.
    _serial.send(ManagedString(msg) + ManagedString("\r\n"), ASYNC);
}

void SerialPort::sendf(const char* fmt, ...) {
    char tmp[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);
    send(tmp);
}
