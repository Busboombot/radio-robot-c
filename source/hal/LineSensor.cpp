#include "LineSensor.h"

LineSensor::LineSensor(MicroBitI2C& i2c)
    : _i2c(i2c)
    , _alpha(0.0f)
{
    // Initialize calibration defaults: min=0, max=255 per channel.
    for (uint8_t ch = 0; ch < 4; ch++) {
        _calMin[ch] = 0;
        _calMax[ch] = 255;
        _emaState[ch] = 0.0f;
    }
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

bool LineSensor::begin()
{
    // Probe with retry: a successful 4-channel raw read means present.  Retry
    // with a settle pause so a sensor still powering up after a cold boot is
    // caught once it answers (the old firmware read it lazily at runtime).
    for (int i = 0; i < 20; i++) {
        if (readRaw(nullptr)) {
            _initialized = true;
            return true;
        }
        fiber_sleep(50);
    }
    _initialized = false;
    return false;
}

bool LineSensor::readValues(uint16_t out[4]) const
{
    if (!is_initialized()) return false;
    return readRaw(out);
}

bool LineSensor::readRaw(uint16_t out[4]) const
{
    for (uint8_t ch = 0; ch < 4; ch++) {
        // Write the channel index byte.
        uint8_t chByte = ch;
        int rc = _i2c.write((uint16_t)(ADDR << 1), (uint8_t*)&chByte, 1, false);
        if (rc != MICROBIT_OK) return false;

        // Read 1 byte of grayscale data.
        uint8_t val = 0;
        rc = _i2c.read((uint16_t)(ADDR << 1), (uint8_t*)&val, 1, false);
        if (rc != MICROBIT_OK) return false;

        if (out) out[ch] = val;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------

bool LineSensor::captureCalibMin()
{
    uint16_t raw[4] = {};
    if (!readValues(raw)) return false;
    for (uint8_t ch = 0; ch < 4; ch++) {
        _calMin[ch] = raw[ch];
    }
    return true;
}

bool LineSensor::captureCalibMax()
{
    uint16_t raw[4] = {};
    if (!readValues(raw)) return false;
    for (uint8_t ch = 0; ch < 4; ch++) {
        _calMax[ch] = raw[ch];
    }
    return true;
}

bool LineSensor::readNormalized(uint16_t out[4])
{
    uint16_t raw[4] = {};
    if (!readValues(raw)) return false;

    for (uint8_t ch = 0; ch < 4; ch++) {
        uint16_t mn = _calMin[ch];
        uint16_t mx = _calMax[ch];
        uint16_t span = (mx > mn) ? (mx - mn) : 255u;

        // Compute normalized value in [0, 1000].
        int32_t norm;
        if (raw[ch] <= mn) {
            norm = 0;
        } else if (raw[ch] >= mx) {
            norm = 1000;
        } else {
            norm = ((int32_t)(raw[ch] - mn) * 1000) / (int32_t)span;
        }

        // Clamp to [0, 1000].
        if (norm < 0) norm = 0;
        if (norm > 1000) norm = 1000;

        // Apply EMA smoothing if enabled.
        if (_alpha > 0.0f) {
            _emaState[ch] = _alpha * _emaState[ch] + (1.0f - _alpha) * (float)norm;
            norm = (int32_t)_emaState[ch];
            // Re-clamp after smoothing.
            if (norm < 0) norm = 0;
            if (norm > 1000) norm = 1000;
        }

        if (out) out[ch] = (uint16_t)norm;
    }
    return true;
}

void LineSensor::setSmoothingAlpha(float alpha)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha >= 1.0f) alpha = 0.99f;  // Clamp to keep system stable.
    _alpha = alpha;
}
