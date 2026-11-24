/**
 * @file DataLoggerSD.h
 * @brief Binary data logger on the Teensy 4.1 built-in SD card.
 *
 * One record per control cycle (or every DATALOG_DECIMATION cycles) while the
 * exo is in trial_on / fsr_calibration / fsr_refinement. A new LOGnnn.BIN file
 * is opened when that state starts and closed when it ends.
 *
 * File layout:
 *   text header, ending with the line "END\n":
 *     OEXLOG1
 *     fields:<comma-separated names>
 *     <build information>
 *     END
 *   binary records, little-endian: uint32 t_us, uint32 cycle, float32 x N
 *
 * Writing uses a RAM ring buffer and 512-byte sector writes only when the
 * card is not busy, so the control loop is not blocked by the SD card.
 * Convert with tools/oexlog_to_csv.py.
 */
#ifndef DATALOGGER_SD_H
#define DATALOGGER_SD_H

#if defined(ARDUINO_TEENSY41)

#include "Arduino.h"
#include "ExoData.h"
#include "FsrFreeConfig.h"

namespace datalogger
{
    /** Call once per control cycle, after the sides have run. */
    void run(ExoData* data, uint32_t loop_dt_us);

    /** Number of records lost because the ring buffer was full (current file). */
    uint32_t dropped();
}

#endif
#endif
