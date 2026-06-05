#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// Protocol v2 response tag constants
// ---------------------------------------------------------------------------
constexpr const char* PROTO_TAG_OK  = "OK";
constexpr const char* PROTO_TAG_ERR = "ERR";
constexpr const char* PROTO_TAG_EVT = "EVT";
constexpr const char* PROTO_TAG_TLM = "TLM";
constexpr const char* PROTO_TAG_CFG = "CFG";
constexpr const char* PROTO_TAG_ID  = "ID";

// ---------------------------------------------------------------------------
// Protocol version and firmware version string
// ---------------------------------------------------------------------------
constexpr int         PROTO_VERSION    = 2;
constexpr const char* FIRMWARE_VERSION = "0.20260605.5";

using ReplyFn = void(*)(const char* msg, void* ctx);

struct ReplyCtx {
    bool viaSerial;
    bool viaRadio;
    bool relay;
};
