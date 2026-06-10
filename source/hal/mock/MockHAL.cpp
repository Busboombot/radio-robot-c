#include "MockHAL.h"
#include <cmath>

void MockHAL::tick(uint32_t now_ms) {
    int32_t dt = static_cast<int32_t>(now_ms - _lastTickMs);
    if (dt > 0) {
        uint32_t udt = static_cast<uint32_t>(dt);

        // Compute turn rate from current motor commands and feed to each motor
        // before ticking so the slip model sees the correct turn intensity.
        float aL = fabsf(static_cast<float>(_motorL.cmdSpeed()));
        float aR = fabsf(static_cast<float>(_motorR.cmdSpeed()));
        float turnRate = (aL + aR > 0.5f)
            ? fabsf(static_cast<float>(_motorR.cmdSpeed() - _motorL.cmdSpeed())) / (aL + aR)
            : 0.0f;
        _motorL.setTurnRate(turnRate);
        _motorR.setTurnRate(turnRate);

        _motorL.tick(udt);
        _motorR.tick(udt);

        // Update oracle ground-truth pose from pre-slip true velocities.
        if (_trackwidthMm > 0.0f) {
            _exactPose.update(
                _motorL.trueVelocityMms(),
                _motorR.trueVelocityMms(),
                _trackwidthMm,
                udt);
        }

        _otos.tick(_motorL.trueVelocityMms(), _motorR.trueVelocityMms(), _trackwidthMm, udt);

        _line.tick(udt);
        _color.tick(udt);
    }
    _lastTickMs = now_ms;
}
