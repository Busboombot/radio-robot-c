---
id: '003'
title: Implement SerialPort and Radio HAL drivers
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on:
- '001'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Implement SerialPort and Radio HAL drivers

## Description

Implement two HAL drivers that handle all host communication:

- `SerialPort` — line-buffered 115200-baud serial over USB; non-blocking `readLine()`
- `Radio` — micro:bit radio with a 4-slot ring buffer; ISR-driven receive; relay support

These are required (non-optional) subsystems. Both are used by `Announcer` and
the tick loop in `Robot`. They can be implemented in parallel with ticket 002.

## Files to Create

- `source/hal/SerialPort.h`
- `source/hal/SerialPort.cpp`
- `source/hal/Radio.h`
- `source/hal/Radio.cpp`

## SerialPort Interface and Implementation Notes

```cpp
// SerialPort.h
#pragma once
#include "MicroBit.h"
#include <stdarg.h>

class SerialPort {
public:
    explicit SerialPort(MicroBitSerial& serial);
    void begin();  // setRxBufferSize(256), setTxBufferSize(256), init(115200)

    // Non-blocking. Accumulates bytes from ASYNC read; returns true when
    // a complete '\n'-terminated line is ready. buf is null-terminated;
    // newline stripped. len includes the NUL terminator.
    bool readLine(char* buf, uint16_t len);

    void send(const char* msg);
    void sendf(const char* fmt, ...);  // snprintf into 128-byte stack buffer

private:
    MicroBitSerial& _serial;
    char     _rxBuf[128];
    uint16_t _rxLen;
};
```

CODAL serial API:
- `_serial.setRxBufferSize(N)` and `_serial.setTxBufferSize(N)` — before `init()`
- `_serial.init(115200)` — sets baud rate (no return value)
- `_serial.read(ASYNC)` — returns next byte (int) or `MICROBIT_NO_DATA` (-1) without sleeping
- `_serial.send(ManagedString(buf))` — sends a null-terminated C string

`readLine()` implementation sketch:
```cpp
bool SerialPort::readLine(char* buf, uint16_t len) {
    int c;
    while ((c = _serial.read(ASYNC)) != MICROBIT_NO_DATA) {
        if (c == '\r') continue;  // ignore CR
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
```

`sendf()` implementation:
```cpp
void SerialPort::sendf(const char* fmt, ...) {
    char tmp[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);
    send(tmp);
}
```

Confirm `MICROBIT_NO_DATA` constant value from CODAL headers after `python build.py`
fetches dependencies. It should be defined in `codal-core/inc/core/ErrorNo.h` or similar.

## Radio Interface and Implementation Notes

```cpp
// Radio.h
#pragma once
#include "MicroBit.h"

class Radio {
public:
    explicit Radio(MicroBitRadio& radio, MicroBitMessageBus& bus);

    // setGroup(10), enable(), setTransmitPower(7), register ISR
    void begin();

    // Non-blocking. Returns true and fills buf if a packet is available.
    // Sets isRelayed=true if the original message started with '>'.
    bool poll(char* buf, uint16_t len, bool& isRelayed);

    // Send msg. If relay=true, prepends '<'.
    void send(const char* msg, bool relay = false);

private:
    MicroBitRadio&      _radio;
    MicroBitMessageBus& _bus;

    static constexpr int SLOTS    = 4;
    static constexpr int SLOT_LEN = 64;
    char    _ring[SLOTS][SLOT_LEN];
    uint8_t _head;   // next slot to write (ISR)
    uint8_t _tail;   // next slot to read (poll)

    static void onData(MicroBitEvent);
    static Radio* _instance;  // set in begin(); allows ISR to reach ring
};
```

CODAL radio API:
```cpp
// In begin():
_radio.setGroup(10);
_radio.enable();                // returns MICROBIT_OK on success
_radio.setTransmitPower(7);
_bus.listen(DEVICE_ID_RADIO, MICROBIT_RADIO_EVT_DATAGRAM, onData);

// In onData ISR:
PacketBuffer pkt = Radio::_instance->_radio.datagram.recv();
// pkt.length() bytes, pkt[i] to access

// In send():
_radio.datagram.send((uint8_t*)outbuf, strlen(outbuf));
// or: _radio.datagram.send(ManagedString(msg));
```

Ring buffer: `_head` and `_tail` are indices mod SLOTS. ISR writes at `_head`,
increments `_head`. `poll()` reads at `_tail`, increments `_tail`. Both wrap at
SLOTS. If the ring is full (ISR check: next_head == _tail), drop the packet.

Relay protocol:
- Inbound: if `pkt[0] == '>'`, strip the `>` and set `isRelayed = true`
- Outbound: if `relay == true`, prefix the send buffer with `<`

`_instance` is set in `begin()`. Only one `Radio` object may call `begin()`.

## Acceptance Criteria

- [x] `source/hal/SerialPort.h` and `.cpp` exist
- [x] `source/hal/Radio.h` and `.cpp` exist
- [x] `SerialPort::readLine()` is non-blocking (never calls `uBit.sleep()`)
- [x] `SerialPort::sendf()` uses a stack-local 128-byte buffer; no heap
- [x] `Radio::begin()` configures group 10, power 7, and registers the ISR
- [x] `Radio` ring buffer holds 4 slots of 64 bytes; drops packets when full
- [x] Relay prefix: inbound `>` stripped; outbound relay prepends `<`
- [x] No global variables except `Radio::_instance` (static class member)
- [ ] `python build.py` compiles with no errors or warnings

## Testing

Hardware-in-the-loop only.

- **Verification**: `python build.py` succeeds. Functional testing deferred to
  ticket 005 (Announcer + Robot) where HELLO via serial is exercised end-to-end.
