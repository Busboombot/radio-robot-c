#pragma once
#include "MicroBit.h"

// ---------------------------------------------------------------------------
// icons — named MicroBitImage constructors for the 5x5 LED matrix.
//
// Each function returns a fresh MicroBitImage by value.  Call as:
//   uBit.display.printAsync(icons::boot());
//
// Add new icons here as additional inline functions in the same namespace.
// ---------------------------------------------------------------------------
namespace icons {

    /// Classic micro:bit 5x5 heart — used as a "powered and ready" boot cue.
    inline MicroBitImage boot() {
        static const uint8_t px[25] = {
            0, 1, 0, 1, 0,
            1, 1, 1, 1, 1,
            1, 1, 1, 1, 1,
            0, 1, 1, 1, 0,
            0, 0, 1, 0, 0,
        };
        return MicroBitImage(5, 5, px);
    }

    // Add more named icons here as needed, e.g.:
    //   inline MicroBitImage sad() { ... }
    //   inline MicroBitImage tick() { ... }

} // namespace icons
