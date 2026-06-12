#include "MockColorSensor.h"

int MockColorSensor::currentRow() const {
    if (_scheduleRows <= 0) return 0;
    uint32_t period = static_cast<uint32_t>(_scheduleRows) * kRowDurationMs;
    uint32_t t = (period > 0) ? (_elapsedMs % period) : 0;
    return static_cast<int>(t / kRowDurationMs);
}

void MockColorSensor::fillOutputs(int row, uint16_t& r, uint16_t& g,
                                  uint16_t& b, uint16_t& c) const {
    r = _table[row][0];
    g = _table[row][1];
    b = _table[row][2];
    c = _table[row][3];
}

bool MockColorSensor::readRGBC(uint16_t& r, uint16_t& g,
                                uint16_t& b, uint16_t& c) {
    // N8 (030-008): when frozen, return false so colorRead() skips lastUpdMs.
    if (_frozen) return false;
    fillOutputs(currentRow(), r, g, b, c);
    return true;
}

bool MockColorSensor::pollRGBC(uint16_t& r, uint16_t& g,
                                uint16_t& b, uint16_t& c) {
    // N8 (030-008): when frozen, return false so colorRead() skips lastUpdMs.
    if (_frozen) return false;
    fillOutputs(currentRow(), r, g, b, c);
    return true;
}

void MockColorSensor::tick(uint32_t dt_ms) {
    _elapsedMs += dt_ms;
}

void MockColorSensor::setSchedule(const uint16_t table[][4], int rows) {
    int n = rows < kScheduleRows ? rows : kScheduleRows;
    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < 4; ++c) {
            _table[r][c] = table[r][c];
        }
    }
    _scheduleRows = n;
    _elapsedMs    = 0;
}
