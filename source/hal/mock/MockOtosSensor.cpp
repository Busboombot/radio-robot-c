#include "MockOtosSensor.h"
#include "types/Config.h"

OtosPose MockOtosSensor::readTransformed(const RobotConfig& /*cfg*/) const {
    OtosPose pose;
    pose.x = _injectedX;
    pose.y = _injectedY;
    pose.h = _injectedH;
    return pose;
}

void MockOtosSensor::getPositionRaw(int16_t& x, int16_t& y, int16_t& h) const {
    x = _rawX;
    y = _rawY;
    h = _rawH;
}

void MockOtosSensor::setPositionRaw(int16_t x, int16_t y, int16_t h) {
    _rawX = x;
    _rawY = y;
    _rawH = h;
}

void MockOtosSensor::setInjectedPose(float x, float y, float h) {
    _injectedX = x;
    _injectedY = y;
    _injectedH = h;
}
