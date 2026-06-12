#pragma once
#include <stdint.h>
#include "../IColorSensor.h"

/**
 * MockColorSensor — host-compilable IColorSensor implementation for unit tests.
 *
 * Cycles through a preset schedule of RGBC rows on each tick(dt_ms).
 * Default schedule: all-zero RGBC (sensor not over a colored surface).
 */
class MockColorSensor : public IColorSensor {
public:
    static constexpr int      kScheduleRows  = 4;
    static constexpr uint32_t kRowDurationMs = 250;  // 4 rows * 250 ms = 1000 ms total

    // IColorSensor interface -------------------------------------------------
    bool readRGBC(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c) override;
    bool pollRGBC(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c) override;

    // Sensor interface -------------------------------------------------------
    bool begin() override { _initialized = true; return true; }

    // Simulation control -----------------------------------------------------
    void tick(uint32_t dt_ms);

    // Override the schedule (rows * 4 values: r, g, b, c).
    void setSchedule(const uint16_t table[][4], int rows);

    void reset() { _elapsedMs = 0; }

    // N8 (030-008): freeze the sensor so pollRGBC() returns false, simulating
    // a wedged sensor.  When frozen, Robot::colorRead() never updates lastUpdMs,
    // so the TLM freshness gate will drop the field after ~2×lagMs.
    void setFrozen(bool frozen) { _frozen = frozen; }

private:
    bool     _frozen       = false;   // N8: frozen=true → pollRGBC returns false

    uint16_t _table[kScheduleRows][4] = {
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
    };
    int      _scheduleRows = kScheduleRows;
    uint32_t _elapsedMs    = 0;

    int currentRow() const;
    void fillOutputs(int row, uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c) const;
};
