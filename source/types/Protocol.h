#pragma once
#include <stdint.h>

constexpr const char* PROTO_CMD_HELLO    = "HELLO";

constexpr const char* PROTO_REPLY_DEVICE = "DEVICE:";
constexpr const char* PROTO_REPLY_LOG    = "LOG:";
constexpr const char* PROTO_REPLY_OK     = "OK";
constexpr const char* PROTO_REPLY_ERR    = "ERR:";

using ReplyFn = void(*)(const char* msg, void* ctx);

struct ReplyCtx {
    bool viaSerial;
    bool viaRadio;
    bool relay;
};
