#pragma once

// Self-contained encoder-wedge bench harness, invoked via
//   DBG WEDGE [rateHz] [writeMs] [busKHz] [dither]
//
// runWedgeTest() takes over the robot and never returns until a wedge is
// detected or a serial byte arrives. It is built to MIRROR the production motor
// path so it reproduces the real wedge:
//   - reads BOTH encoders every tick (motor 1 first), 4 ms settle
//   - drives the motors with a per-tick DITHERING pwm (mimics the PID emitting a
//     fresh value every tick), written subject to a write-on-change + min-
//     interval rate limit exactly like Motor::setSpeed()
//   - runs the bus at busKHz (production = 400 kHz)
// and prints the measured loop rate AND motor-write rate once a second.
//
// Params (all optional, production-faithful defaults):
//   rateHz  loop/read rate              (default 50)
//   writeMs min ms between motor writes (default 40 — matches setSpeed; 0 = every tick)
//   busKHz  I2C bus speed in kHz        (default 400 — production)
//   dither  per-tick pwm dither ±units  (default 3 — forces a write every tick)
//   reg     encoder read register       (default 0x46 angle/pos; 0x47 = speed)
//   sensors 1 = also hammer OTOS/colour/line on the shared bus (production load)
//   realCtrl 1 = drive through the REAL production control path (set mm/s target
//            → MotorController velocity PID → Motor::setSpeed/I2CBus →
//            controlCollectSplitPhase read-both), instead of the raw fixed-PWM
//            uBit.i2c path. THIS is the suspected toggle: the raw path never
//            wedged; the real PID (high spin-up PWM, write-every-tick via the
//            I2CBus wrapper) is what production does. Phase l/r are mm/s in this
//            mode, raw PWM otherwise. Requires a non-null Robot*.
//
// A mode-independent wedge check reads 0x46 position once a second and trips
// WEDGE-POS-FROZEN if the robot was driven but position did not advance — so it
// works whether the per-tick reads are 0x46 or 0x47.

#include "MicroBit.h"

class Robot;   // forward decl — real-control mode drives through it

void runWedgeTest(MicroBit& uBit, int rateHz = 50, int writeMs = 40,
                  int busKHz = 400, int dither = 3, int reg = 0x46, int sensors = 0,
                  int realCtrl = 0, Robot* robot = nullptr);
