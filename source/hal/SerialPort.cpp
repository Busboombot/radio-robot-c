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
    _serial.send(ManagedString(msg));
}

void SerialPort::sendf(const char* fmt, ...) {
    char tmp[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);
    send(tmp);
}
