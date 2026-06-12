#include "NezhaHAL.h"

NezhaHAL::NezhaHAL(MicroBitI2C& i2c, MicroBitIO& io, const RobotConfig& cfg)
    : _bus(i2c),
      _motorL(_bus, 2, cfg.fwdSignL),   // M2 left
      _motorR(_bus, 1, cfg.fwdSignR),   // M1 right
      _otos(_bus, cfg),
      _benchOtos(),
      _line(_bus),
      _color(_bus),
      _portio(io),
      _gripper(io.P1),
      _otosActive(&_otos)               // default: real sensor
{
}

void NezhaHAL::begin()
{
    _otos.begin();
    _benchOtos.begin();   // no-op: sets _initialized = true
    _line.begin();
    _color.begin();
}
