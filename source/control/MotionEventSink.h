#pragma once

// ---------------------------------------------------------------------------
// MotionEventSink — narrow interface for MotionController to report motion
// completion events and safety events to the app layer without any
// protocol-layer knowledge.
//
// Sprint 026, Ticket 002: eliminates the MotionController → CommandProcessor
// dependency (layering inversion fix).
//
// The app layer sets emitFn to a static function that formats and calls
// CommandProcessor::replyEvt (or directly appends the EVT line to the reply
// context). source/control/ sees only this header, which has no app-layer
// includes.
// ---------------------------------------------------------------------------
struct MotionEventSink {
    void (*emitFn)(const char* evtLine, const char* corrId, void* ctx);
    void* ctx;
};
