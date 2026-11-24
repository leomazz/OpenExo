/**
 * @file SdOnce.h
 * @brief Initialise the SD card only once. The original firmware called
 *        SD.begin() every time controller parameters were read; re-initialising
 *        the card while the data logger has a file open could corrupt it.
 */
#ifndef SD_ONCE_H
#define SD_ONCE_H

#include <SD.h>

inline bool oex_sd_begin(uint8_t cs_pin)
{
    static bool ok = false;
    if (!ok)
    {
        ok = SD.begin(cs_pin);
    }
    return ok;
}

#endif
