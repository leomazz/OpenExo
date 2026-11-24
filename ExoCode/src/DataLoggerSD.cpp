#include "DataLoggerSD.h"

#if defined(ARDUINO_TEENSY41)

#include <SD.h>
#include <RingBuf.h>
#include "SdOnce.h"
#include "StatusDefs.h"
#include "Config.h"
#include "PelvisImu.h"
#include "ParamsFromSD.h"   // SD_SELECT

#ifndef SD_SELECT
#define SD_SELECT BUILTIN_SDCARD
#endif

/* ------------------------------------------------------------------------
 * Logged fields. X(name, expression) with `d` = ExoData*, `S` = SideData&.
 * To add a field, add one line: header and record stay consistent.
 * ------------------------------------------------------------------------ */
#define OEX_GLOBAL_FIELDS(X) \
    X(status,              (float)d->get_status()) \
    X(sync_led,            (float)d->sync_led_state) \
    X(loop_dt_us,          (float)loop_dt_us) \
    X(imu_ax_g,            pelvis_imu::raw.acc_g[0]) \
    X(imu_ay_g,            pelvis_imu::raw.acc_g[1]) \
    X(imu_az_g,            pelvis_imu::raw.acc_g[2]) \
    X(imu_gx_dps,          pelvis_imu::raw.gyro_dps[0]) \
    X(imu_gy_dps,          pelvis_imu::raw.gyro_dps[1]) \
    X(imu_gz_dps,          pelvis_imu::raw.gyro_dps[2]) \
    X(imu_seq,             pelvis_imu::raw.seq) \
    X(imu_nano_ms,         pelvis_imu::raw.nano_ms) \
    X(imu_count,           (float)pelvis_imu::raw.count) \
    X(imu_age_us,          (float)(pelvis_imu::state.age_us > 1e7 ? 1e7 : pelvis_imu::state.age_us)) \
    X(imu_ok,              (float)pelvis_imu::state.ok) \
    X(pelvis_tilt_deg,     pelvis_imu::state.tilt_deg) \
    X(pelvis_tilt_acc_deg, pelvis_imu::state.tilt_acc_deg) \
    X(pelvis_rate_dps,     pelvis_imu::state.tilt_rate_dps)

#define OEX_SIDE_FIELDS(X, P) \
    X(P##heel_fsr,            S.heel_fsr) \
    X(P##toe_fsr,             S.toe_fsr) \
    X(P##fsr_hs,              (float)S.fsr_ground_strike) \
    X(P##fsr_to,              (float)S.fsr_toe_off) \
    X(P##fsr_pct_gait,        S.fsr_percent_gait) \
    X(P##fsr_pct_stance,      S.fsr_percent_stance) \
    X(P##fsr_step_ms,         S.fsr_expected_step_duration) \
    X(P##used_pct_gait,       S.percent_gait) \
    X(P##used_pct_stance,     S.percent_stance) \
    X(P##used_step_ms,        S.expected_step_duration) \
    X(P##hip_pos_rad,         S.hip.position) \
    X(P##hip_vel_rads,        S.hip.velocity) \
    X(P##mot_p,               S.hip.motor.p) \
    X(P##mot_v,               S.hip.motor.v) \
    X(P##mot_i,               S.hip.motor.i) \
    X(P##mot_t_ff,            S.hip.motor.t_ff) \
    X(P##mot_enabled,         (float)S.hip.motor.enabled) \
    X(P##mot_timeouts,        (float)S.hip.motor.timeout_count) \
    X(P##can_len,             (float)S.hip.motor.can_len) \
    X(P##can_b6,              (float)S.hip.motor.can_b6) \
    X(P##can_b7,              (float)S.hip.motor.can_b7) \
    X(P##can_rx_count,        (float)S.hip.motor.can_rx_count) \
    X(P##setpoint,            S.hip.controller.setpoint) \
    X(P##ff_setpoint,         S.hip.controller.ff_setpoint) \
    X(P##torque_reading,      S.hip.torque_reading) \
    X(P##pp_enc_in_deg,       S.pp_enc_input_deg) \
    X(P##pp_enc_phase,        S.pp_enc_phase) \
    X(P##pp_enc_stride_ms,    S.pp_enc_stride_ms) \
    X(P##pp_enc_valid,        (float)S.pp_enc_valid) \
    X(P##pp_enc_hs,           (float)S.pp_enc_hs) \
    X(P##pp_imu_in_deg,       S.pp_imu_input_deg) \
    X(P##pp_imu_phase,        S.pp_imu_phase) \
    X(P##pp_imu_stride_ms,    S.pp_imu_stride_ms) \
    X(P##pp_imu_valid,        (float)S.pp_imu_valid) \
    X(P##pp_imu_hs,           (float)S.pp_imu_hs) \
    X(P##phase_source,        (float)S.phase_source) \
    X(P##inhibit,             (float)S.hip.phase_inhibit) \
    X(P##est_exec_us,         S.estimator_exec_us)

#define OEX_COUNT(name, expr) +1
static const int N_GLOBAL = 0 OEX_GLOBAL_FIELDS(OEX_COUNT);
static const int N_SIDE   = 0 OEX_SIDE_FIELDS(OEX_COUNT, x_);
static const int N_FIELDS = N_GLOBAL + 2 * N_SIDE;

namespace datalogger
{
    static FsFile _file;
    static RingBuf<FsFile, DATALOG_RING_BYTES> _rb;
    static bool _open = false;
    static uint32_t _cycle = 0;
    static uint32_t _dropped = 0;
    static uint32_t _records = 0;

    uint32_t dropped() { return _dropped; }

    static void _write_header()
    {
        _rb.print("OEXLOG1\nfields:t_us,cycle");
        #define OEX_NAME(name, expr) _rb.print("," #name);
        OEX_GLOBAL_FIELDS(OEX_NAME)
        OEX_SIDE_FIELDS(OEX_NAME, L_)
        OEX_SIDE_FIELDS(OEX_NAME, R_)
        #undef OEX_NAME
        _rb.print("\nbuild:");
        _rb.print("PHASE_SOURCE="); _rb.print(PHASE_SOURCE);
        _rb.print(";PP_ALPHA="); _rb.print(PP_ALPHA);
        _rb.print(";PP_FILTER_ORDER="); _rb.print(PP_FILTER_ORDER);
        _rb.print(";PP_NORM_POLE_1KHZ="); _rb.print(PP_NORM_POLE_1KHZ);
        _rb.print(";PP_PHASE_OFFSET_PCT="); _rb.print(PP_PHASE_OFFSET_PCT);
        _rb.print(";PP_TOE_OFF_PCT="); _rb.print(PP_TOE_OFF_PCT);
        _rb.print(";HIP_SIGN_L="); _rb.print(HIP_FLEXION_SIGN_LEFT);
        _rb.print(";HIP_SIGN_R="); _rb.print(HIP_FLEXION_SIGN_RIGHT);
        _rb.print(";PELVIS_AXES="); _rb.print(PELVIS_AXIS_VERTICAL); _rb.print(PELVIS_AXIS_FORWARD); _rb.print(PELVIS_AXIS_LATERAL);
        _rb.print(";LOOP_FREQ_HZ="); _rb.print(LOOP_FREQ_HZ);
        _rb.print(";DECIMATION="); _rb.print(DATALOG_DECIMATION);
        _rb.print(";N_FIELDS="); _rb.print(N_FIELDS);
        _rb.print("\nEND\n");
    }

    static bool _open_file()
    {
        if (!oex_sd_begin(SD_SELECT)) return false;

        char name[16];
        for (int i = 0; i < 1000; i++)
        {
            snprintf(name, sizeof(name), "LOG%03d.BIN", i);
            if (!SD.sdfs.exists(name)) break;
        }
        if (!_file.open(&SD.sdfs, name, O_RDWR | O_CREAT | O_TRUNC)) return false;
        _file.preAllocate(DATALOG_PREALLOC_BYTES);   // avoids long busy periods
        _rb.begin(&_file);
        _dropped = 0;
        _records = 0;
        _write_header();
        Serial.print("\nDataLogger: opened "); Serial.println(name);
        return true;
    }

    static void _close_file()
    {
        _rb.sync();            // flush what is left in RAM
        _file.truncate();      // drop the unused preallocated space
        _file.close();
        Serial.print("\nDataLogger: closed, records = "); Serial.print(_records);
        Serial.print(", dropped = "); Serial.println(_dropped);
    }

    void run(ExoData* d, uint32_t loop_dt_us)
    {
#if DATALOG_ENABLE
        const uint16_t st = d->get_status();
        const bool logging_state = (st == status_defs::messages::trial_on) ||
                                   (st == status_defs::messages::fsr_calibration) ||
                                   (st == status_defs::messages::fsr_refinement);
        _cycle++;

        if (logging_state && !_open)
        {
            _open = _open_file();
            if (!_open) return;
        }
        else if (!logging_state && _open)
        {
            _close_file();
            _open = false;
            return;
        }
        if (!_open) return;

        if ((_cycle % DATALOG_DECIMATION) == 0)
        {
            const size_t rec_bytes = 8 + 4 * N_FIELDS;
            if (_rb.bytesFree() >= rec_bytes)
            {
                const uint32_t t = micros();
                _rb.write((const uint8_t*)&t, 4);
                _rb.write((const uint8_t*)&_cycle, 4);
                float v;
                #define OEX_WRITE(name, expr) v = (float)(expr); _rb.write((const uint8_t*)&v, 4);
                OEX_GLOBAL_FIELDS(OEX_WRITE)
                {
                    SideData& S = d->left_side;
                    OEX_SIDE_FIELDS(OEX_WRITE, L_)
                }
                {
                    SideData& S = d->right_side;
                    OEX_SIDE_FIELDS(OEX_WRITE, R_)
                }
                #undef OEX_WRITE
                _records++;
            }
            else
            {
                _dropped++;
            }
        }

        // Move one sector to the card when the card is free (non-blocking).
        if (_rb.bytesUsed() >= 512 && !_file.isBusy())
        {
            _rb.writeOut(512);
        }
#endif
    }
}

#endif
