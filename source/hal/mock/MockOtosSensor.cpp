#include "MockOtosSensor.h"
#include "types/Config.h"
#include <cmath>

#ifdef HOST_BUILD
#include <random>

static float otosGaussian(std::mt19937& rng, float sigma) {
    if (sigma <= 0.0f) return 0.0f;
    std::normal_distribution<float> dist(0.0f, sigma);
    return dist(rng);
}
#endif

bool MockOtosSensor::readTransformed(const RobotConfig& /*cfg*/, OtosPose& poseOut,
                                      float /*headingRad*/) const {
    if (_readFailure) {
        poseOut = {0.0f, 0.0f, 0.0f};
        return false;
    }
    if (_useSimModel) {
        poseOut.x = _odomX;
        poseOut.y = _odomY;
        poseOut.h = _odomH;
    } else {
        poseOut.x = _injectedX;
        poseOut.y = _injectedY;
        poseOut.h = _injectedH;
    }
    return true;
}

bool MockOtosSensor::readVelocityTransformed(const RobotConfig& /*cfg*/, OtosVelocity& velOut,
                                              float /*headingRad*/) const {
    if (_readFailure) {
        velOut = {0.0f, 0.0f};
        return false;
    }
    velOut = {_velV, _velOmega};
    return true;
}

OtosAccel MockOtosSensor::readAccelTransformed(const RobotConfig& /*cfg*/) const {
    return {_accAx, _accAy};
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
    // Also reset the odometry accumulator so camera fixes reset the OTOS model.
    _odomX = x;
    _odomY = y;
    _odomH = h;
}

void MockOtosSensor::tick(float velL, float velR, float tw, uint32_t dt_ms) {
#ifdef HOST_BUILD
    if (!_useSimModel || tw <= 0.0f) return;
    float dt_s    = static_cast<float>(dt_ms) / 1000.0f;
    float dC      = (velL + velR) * 0.5f * dt_s;
    float dTh     = (velR - velL) / tw * dt_s;
    float noisyDC  = dC  * (1.0f + otosGaussian(_rng, _linearNoiseSigma));
    float noisyDTh = dTh * (1.0f + otosGaussian(_rng, _yawNoiseSigma));
    float hMid    = _odomH + noisyDTh * 0.5f;
    _odomX += noisyDC * cosf(hMid);
    _odomY += noisyDC * sinf(hMid);
    _odomH += noisyDTh;
    // Wrap heading to [-pi, pi]
    while (_odomH >  static_cast<float>(M_PI)) _odomH -= 2.0f * static_cast<float>(M_PI);
    while (_odomH < -static_cast<float>(M_PI)) _odomH += 2.0f * static_cast<float>(M_PI);

    // Body-frame velocity/accel from the same noisy arc segment (consistent
    // with the position channel). v = arc-distance / dt, omega = dTheta / dt.
    if (dt_s > 0.0f) {
        float newV     = noisyDC  / dt_s;
        float newOmega = noisyDTh / dt_s;
        _accAx    = (newV - _prevVelV) / dt_s;
        _accAy    = 0.0f;
        _prevVelV = newV;
        _velV     = newV;
        _velOmega = newOmega;
    }
#else
    (void)velL; (void)velR; (void)tw; (void)dt_ms;
#endif
}
