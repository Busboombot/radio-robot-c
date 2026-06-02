#pragma once
#include "MicroBit.h"

/**
 * Radio — micro:bit radio driver speaking the RadioRelay RAW250 framing.
 *
 * Configured for channel 0, group 10, transmit power 7, to match the
 * micro:bit RadioRelay's defaults (see microbit-radio-relay, RAW250 mode).
 * The firmware MUST be built with MICROBIT_RADIO_MAX_PACKET_SIZE=250
 * (set in codal.json) so the on-air nRF MAXLEN matches the relay; otherwise
 * the relay's larger frames are dropped on receive.
 *
 * Wire framing (RadioRelay §5): every on-air packet is a fragment
 *     [SEQ:1][FLAGS:1][LEN:1][payload:LEN]
 * carried as the CODAL datagram payload (no MakeCode/PXT header in RAW250).
 * FLAGS: START=0x01, MORE=0x02, END=0x04, ACK=0x10. A message is split into
 * fragments of up to MTU (=247) payload bytes; the receiver reassembles from
 * START through END. A single-fragment message is flagged START|END (0x05).
 *
 * Receive: the CODAL datagram ISR reassembles fragments in place and, on END,
 * publishes the complete message; poll() (main loop) hands it to the caller.
 * Only one completed message is buffered — if a second completes before poll()
 * consumes the first, the newer one is dropped (commands are processed far
 * faster than they arrive). Send: send() fragments the message and transmits
 * each frame. The relay forwards both directions transparently, so a
 * reassembled message is a host command line and send() output reaches the host.
 *
 * Only one Radio instance may call begin(). _instance is a static singleton
 * pointer used by the static ISR callback.
 */
class Radio {
public:
    explicit Radio(MicroBitRadio& radio, MessageBus& bus);

    // enable(), setFrequencyBand(0), setGroup(10), setTransmitPower(7),
    // register the datagram ISR.
    void begin();

    // Non-blocking. Returns true and fills buf (NUL-terminated) when a complete
    // reassembled message is ready.
    bool poll(char* buf, uint16_t len);

    // Fragment msg into RAW250 frames and transmit each one.
    void send(const char* msg);

private:
    MicroBitRadio& _radio;
    MessageBus&    _bus;

    // RadioRelay §5 fragment framing.
    static constexpr uint8_t FLAG_START = 0x01;
    static constexpr uint8_t FLAG_MORE  = 0x02;
    static constexpr uint8_t FLAG_END   = 0x04;
    static constexpr uint8_t FLAG_ACK   = 0x10;
    static constexpr int FRAME_HEADER = 3;
    static constexpr int MAX_FRAME    = MICROBIT_RADIO_MAX_PACKET_SIZE; // 250
    static constexpr int MTU          = MAX_FRAME - FRAME_HEADER;       // 247
    static constexpr int REASM_MAX    = 512;   // v2 GET dump can reach ~290 bytes

    // Reassembly accumulator (ISR-owned).
    char _reasm[REASM_MAX];
    int  _reasmLen;
    bool _reasmActive;

    // Completed message published to poll(). _msgReady gates the handoff and is
    // the single synchronization point between the ISR and the main loop.
    char          _msg[REASM_MAX];
    volatile bool _msgReady;

    uint8_t _txSeq;           // rolling §5 sequence number

    static void onData(MicroBitEvent);
    static Radio* _instance;
};
