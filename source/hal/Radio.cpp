#include "Radio.h"
#include <string.h>

Radio* Radio::_instance = nullptr;

Radio::Radio(MicroBitRadio& radio, MessageBus& bus)
    : _radio(radio), _bus(bus), _head(0), _tail(0)
{
    memset(_ring, 0, sizeof(_ring));
}

void Radio::begin() {
    _instance = this;
    _radio.setGroup(10);
    _radio.enable();
    _radio.setTransmitPower(7);
    _bus.listen(DEVICE_ID_RADIO, MICROBIT_RADIO_EVT_DATAGRAM, onData);
}

void Radio::onData(MicroBitEvent) {
    if (!_instance) return;
    PacketBuffer pkt = _instance->_radio.datagram.recv();
    uint8_t next = (_instance->_head + 1) % SLOTS;
    if (next == _instance->_tail) return;  // ring full, drop
    int len = pkt.length();
    if (len >= SLOT_LEN) len = SLOT_LEN - 1;
    for (int i = 0; i < len; i++)
        _instance->_ring[_instance->_head][i] = pkt[i];
    _instance->_ring[_instance->_head][len] = '\0';
    _instance->_head = next;
}

bool Radio::poll(char* buf, uint16_t len, bool& isRelayed) {
    if (_tail == _head) return false;
    const char* slot = _ring[_tail];
    isRelayed = (slot[0] == '>');
    const char* src = isRelayed ? slot + 1 : slot;
    uint16_t copy = (uint16_t)strlen(src);
    if (copy >= len) copy = len - 1;
    memcpy(buf, src, copy);
    buf[copy] = '\0';
    _tail = (_tail + 1) % SLOTS;
    return true;
}

void Radio::send(const char* msg, bool relay) {
    char outbuf[SLOT_LEN];
    if (relay) {
        outbuf[0] = '<';
        strncpy(outbuf + 1, msg, SLOT_LEN - 2);
        outbuf[SLOT_LEN - 1] = '\0';
    } else {
        strncpy(outbuf, msg, SLOT_LEN - 1);
        outbuf[SLOT_LEN - 1] = '\0';
    }
    _radio.datagram.send((uint8_t*)outbuf, strlen(outbuf));
}
