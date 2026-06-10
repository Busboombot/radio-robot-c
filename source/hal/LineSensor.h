#pragma once
#include "MicroBit.h"
#include "I2CBus.h"
#include "ILineSensor.h"
#include <stdint.h>

/**
 * LineSensor — I2C driver for the PlanetX line sensor.
 *
 * I2C address: 0x1A (7-bit).
 *
 * Protocol: write 1-byte channel index (0-3), then read 1 byte of grayscale
 * data (0 = white, 255 = black approximately).
 *
 * Calibration workflow:
 *   1. Place robot over white surface; call captureCalibMin().
 *   2. Place robot over black surface; call captureCalibMax().
 *   3. Call readNormalized() to get 0–1000 scaled values per channel
 *      (0 = white, 1000 = black). Values are clamped to [0, 1000].
 *
 * Optional EMA smoothing (applied in readNormalized only):
 *   Call setSmoothingAlpha(alpha) with alpha in [0.0, 1.0).
 *   alpha = 0.0 means no smoothing (default).
 *   Higher alpha means more smoothing (output lags behind input more).
 *   EMA formula: ema = alpha * ema_prev + (1 - alpha) * new_sample
 */
class LineSensor : public ILineSensor {
public:
    explicit LineSensor(I2CBus& i2c);

    // Probe the sensor (read all 4 channels); set _initialized to the result.
    // Returns _initialized.
    bool begin() override;  // ILineSensor (via Sensor)

    // Fills out[0..3] with raw grayscale values (0=white, 255=black approx).
    // Returns false on I2C error. out may be nullptr (probe use).
    bool readValues(uint16_t out[4]) const override;

    // Snapshot current raw readings into the calibration minimum array.
    // Call this while the sensor is over a white surface.
    bool captureCalibMin();

    // Snapshot current raw readings into the calibration maximum array.
    // Call this while the sensor is over a black surface.
    bool captureCalibMax();

    // Fills out[0..3] with normalized values in [0, 1000] per channel.
    // 0 = white (min calibration), 1000 = black (max calibration).
    // If min == max for a channel, span defaults to 255.
    // Applies EMA smoothing if _alpha > 0.0f.
    // Returns false on I2C error.
    bool readNormalized(uint16_t out[4]) override;

    // Set EMA smoothing coefficient for readNormalized.
    // alpha = 0.0 means no smoothing (default).
    // alpha in (0.0, 1.0) applies exponential moving average.
    void setSmoothingAlpha(float alpha);

private:
    I2CBus& _i2c;
    static constexpr uint8_t ADDR = 0x1A;

    // Low-level 4-channel raw read (ungated); used by begin()'s probe and by
    // the gated readValues().  out may be nullptr (probe use).
    bool readRaw(uint16_t out[4]) const;

    // Per-channel calibration bounds. Defaults: min=0, max=255.
    uint16_t _calMin[4];
    uint16_t _calMax[4];

    // EMA smoothing state (normalized, float 0.0–1000.0).
    float _alpha;
    float _emaState[4];
};
