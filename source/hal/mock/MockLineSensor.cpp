#include "MockLineSensor.h"
#include <string.h>

int MockLineSensor::currentRow() const {
    if (_scheduleRows <= 0) return 0;
    uint32_t period = static_cast<uint32_t>(_scheduleRows) * kRowDurationMs;
    uint32_t t = (period > 0) ? (_elapsedMs % period) : 0;
    return static_cast<int>(t / kRowDurationMs);
}

bool MockLineSensor::readValues(uint16_t out[4]) const {
    // N8 (030-008): when frozen, return false (no new data) so lineRead()
    // skips the lastUpdMs update — the TLM freshness gate then drops the field.
    if (_frozen) return false;
    int row = currentRow();
    for (int i = 0; i < 4; ++i) {
        out[i] = _table[row][i];
    }
    return true;
}

bool MockLineSensor::readNormalized(uint16_t out[4]) {
    return readValues(out);
}

void MockLineSensor::tick(uint32_t dt_ms) {
    _elapsedMs += dt_ms;
}

void MockLineSensor::setSchedule(const uint16_t table[][4], int rows) {
    int n = rows < kScheduleRows ? rows : kScheduleRows;
    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < 4; ++c) {
            _table[r][c] = table[r][c];
        }
    }
    _scheduleRows = n;
    _elapsedMs    = 0;
}
