#pragma once
#include <stdint.h>
#include "../ILineSensor.h"

/**
 * MockLineSensor — host-compilable ILineSensor implementation for unit tests.
 *
 * Cycles through a preset schedule of 4-channel uint16_t rows on each
 * tick(dt_ms). The default schedule simulates a simple line crossing:
 *   row 0: all bright   (1000, 1000, 1000, 1000)
 *   row 1: ch0 dark     (   0, 1000, 1000, 1000)
 *   row 2: all dark     (   0,    0,    0,    0)
 *   row 3: ch3 dark     (1000, 1000, 1000,    0)
 *   row 4: all bright   (1000, 1000, 1000, 1000)
 *
 * The schedule advances to the next row every kRowDurationMs milliseconds.
 */
class MockLineSensor : public ILineSensor {
public:
    static constexpr int      kScheduleRows   = 5;
    static constexpr uint32_t kRowDurationMs  = 600;  // 5 rows * 600 ms = 3000 ms total

    // ILineSensor interface --------------------------------------------------
    bool readValues(uint16_t out[4]) const override;
    bool readNormalized(uint16_t out[4]) override;

    // Sensor interface -------------------------------------------------------
    bool begin() override { _initialized = true; return true; }

    // Simulation control -----------------------------------------------------
    void tick(uint32_t dt_ms);

    // Override the entire schedule (rows * 4 values).
    void setSchedule(const uint16_t table[][4], int rows);

    // Reset schedule position.
    void reset() { _elapsedMs = 0; }

private:
    // Default schedule — normalized values in [0, 1000].
    uint16_t _table[kScheduleRows][4] = {
        {1000, 1000, 1000, 1000},
        {   0, 1000, 1000, 1000},
        {   0,    0,    0,    0},
        {1000, 1000, 1000,    0},
        {1000, 1000, 1000, 1000},
    };
    int      _scheduleRows = kScheduleRows;
    uint32_t _elapsedMs    = 0;

    int currentRow() const;
};
