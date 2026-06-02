#pragma once
#include "MicroBit.h"
#include <stdarg.h>

/**
 * SerialPort — line-buffered 115200-baud serial over USB.
 *
 * Non-blocking: readLine() drains the CODAL ASYNC receive buffer each call
 * and returns true only when a complete '\n'-terminated line is ready.
 * Never calls uBit.sleep() or any blocking CODAL primitive.
 */
class SerialPort {
public:
    explicit SerialPort(NRF52Serial& serial);

    // Configure buffers (256 bytes each) and set baud rate to 115200.
    // Must be called once before readLine() / send() / sendf().
    void begin();

    // Non-blocking. Accumulates bytes from ASYNC read; returns true when
    // a complete '\n'-terminated line is ready. buf is null-terminated;
    // newline stripped. len includes the NUL terminator.
    bool readLine(char* buf, uint16_t len);

    void send(const char* msg);
    void sendf(const char* fmt, ...);  // snprintf into 256-byte stack buffer

private:
    NRF52Serial& _serial;
    char     _rxBuf[256];   // holds up to a 250-byte line (RAW250 message size)
    uint16_t _rxLen;
};
