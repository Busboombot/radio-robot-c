#include "MockHAL.h"
#include "control/RobotState.h"   // MotorCommands (commanded wheel velocity)
#include <cmath>

// tick(now) — plant integration with no actuator-command input (the bench
// sensor needs commands, so it is not driven on this path).
void MockHAL::tick(uint32_t now_ms) {
    advance(now_ms, nullptr);
}

// tick(now, cmds) — the firmware loop's actuator-state tick.  Drives the same
// plant AND, when bench mode is active, feeds the BenchOtosSensor the commanded
// wheel velocity (mirrors NezhaHAL::tick).
void MockHAL::tick(uint32_t now_ms, const MotorCommands& cmds) {
    advance(now_ms, &cmds);
}

void MockHAL::advance(uint32_t now_ms, const MotorCommands* cmds) {
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

        // Bench OTOS: when active, integrate the COMMANDED wheel velocity into
        // the synthetic pose — the same device + input the firmware uses on
        // hardware.  otos() returns this sensor while bench mode is on, so
        // Robot::otosCorrect() fuses it into the EKF exactly as on the bench.
        if (cmds != nullptr && _trackwidthMm > 0.0f &&
                _otosActive == static_cast<IOtosSensor*>(&_benchOtos)) {
            _benchOtos.tick(cmds->tgtLMms, cmds->tgtRMms, _trackwidthMm, udt);
        }

        _line.tick(udt);
        _color.tick(udt);
    }
    _lastTickMs = now_ms;
}
