#pragma once
#include "MicroBit.h"

/**
 * Radio — micro:bit radio driver with ISR-driven receive ring buffer.
 *
 * Configured for group 10, transmit power 7. Receive packets are stored in a
 * 4-slot ring buffer written by the CODAL event ISR and drained by poll().
 *
 * Relay protocol:
 *   - Inbound: message starting with '>' is a relay; the '>' is stripped and
 *     isRelayed is set to true in poll().
 *   - Outbound: if relay=true in send(), the buffer is prefixed with '<'.
 *
 * Only one Radio instance may call begin(). _instance is a static singleton
 * pointer used by the static ISR callback.
 */
class Radio {
public:
    explicit Radio(MicroBitRadio& radio, MessageBus& bus);

    // setGroup(10), enable(), setTransmitPower(7), register ISR.
    void begin();

    // Non-blocking. Returns true and fills buf if a packet is available.
    // Sets isRelayed=true if the original message started with '>'.
    bool poll(char* buf, uint16_t len, bool& isRelayed);

    // Send msg. If relay=true, prepends '<'.
    void send(const char* msg, bool relay = false);

private:
    MicroBitRadio&      _radio;
    MessageBus& _bus;

    static constexpr int SLOTS    = 4;
    static constexpr int SLOT_LEN = 64;
    char    _ring[SLOTS][SLOT_LEN];
    uint8_t _head;   // next slot to write (ISR)
    uint8_t _tail;   // next slot to read (poll)

    static void onData(MicroBitEvent);
    static Radio* _instance;
};
