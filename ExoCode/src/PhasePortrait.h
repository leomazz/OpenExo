/**
 * @file PhasePortrait.h
 * @brief Speed-adaptive phase-portrait gait phase estimator, after
 *        Choi, Ko & Kong, "Walking-Speed-Adaptive Gait Phase Estimation for
 *        Wearable Robots", Sensors 2023, 23(19):8276.
 *
 * Input : one sagittal angle per control cycle (hip or thigh, deg, flexion +).
 * Output: continuous gait phase in % (0 % = heel strike), stride period,
 *         virtual heel-strike / toe-off events, validity flag.
 *
 * Steps (numbers refer to Choi 2023):
 *  1. Gait-frequency adaptive low-pass filter, Eqs. (4)-(7):
 *     n cascaded first-order sections with cutoff fc = ALPHA / T_stride.
 *     For this filter the delay at the gait frequency is exactly
 *     n*atan(1/ALPHA) (Eq. 8), compensated in step 4 (Eq. 9).
 *  2. Angular velocity by numerical differentiation of the filtered angle.
 *  3. Normalization, Eqs. (2)-(3), (10)-(13): centre = (max+min)/2 and
 *     amplitude = (max-min)/2 of the previous stride, updated at maximum
 *     flexion and smoothed by H(z) = (1-a)/(1-a z^-1), Eq. (14).
 *  4. phase = mod(-atan2(dtheta_n, theta_n) + n*atan(1/ALPHA) - phi_off, 2*pi),
 *     where -atan2 equals the piecewise Eq. (1) and phi_off moves the origin
 *     from maximum flexion to heel strike.
 *  Difference from Choi: the stride period is measured between consecutive
 *  maxima of the angle, so no foot sensor is needed.
 *
 * Header-only and free of Arduino calls so that it can be unit-tested on a PC.
 */
#ifndef PHASE_PORTRAIT_H
#define PHASE_PORTRAIT_H

#include <math.h>
#include <stdint.h>

#ifndef PP_TWO_PI
#define PP_TWO_PI 6.28318530717958647692f
#endif

struct PhasePortraitParams
{
    float alpha = 2.5f;
    int   order = 2;
    float norm_pole_1khz = 0.98f;
    float phase_offset_pct = 17.0f;
    float toe_off_pct = 60.0f;
    float initial_stride_ms = 1100.0f;
    float min_stride_ms = 600.0f;
    float max_stride_ms = 2000.0f;
    float min_amplitude_deg = 5.0f;
    int   strides_to_valid = 2;
};

class PhasePortrait
{
public:
    static const int MAX_ORDER = 4;

    explicit PhasePortrait(const PhasePortraitParams& p = PhasePortraitParams()) : _p(p)
    {
        if (_p.order < 1) _p.order = 1;
        if (_p.order > MAX_ORDER) _p.order = MAX_ORDER;
        reset();
    }

    void reset()
    {
        _first = true;
        _stride_ms = _p.initial_stride_ms;
        for (int i = 0; i < MAX_ORDER; i++) { _x1[i] = 0; _y1[i] = 0; }
        _theta_f = _theta_f_prev = 0; _dtheta = 0; _dtheta_prev = 0;
        _have_last_max = false; _last_max_us = 0;
        _strides = 0;
        _norm_init = false;
        _c_th = _c_dth = 0; _a_th = _a_dth = 1;
        _tc_th = _tc_dth = 0; _ta_th = _ta_dth = 1;
        _reset_extrema();
        _phase_pct = -1; _prev_phase_pct = -1;
        _valid = false; _heel_strike = false; _toe_off = false; _stance = false;
        _fs_hz = 500.0f;
        _update_filter();
    }

    /**
     * @param t_us      timestamp in microseconds (wrap-safe)
     * @param theta_deg angle, flexion positive
     * @param input_ok  false if the input is stale (e.g. IMU timeout)
     */
    void update(uint32_t t_us, float theta_deg, bool input_ok = true)
    {
        _heel_strike = false;
        _toe_off = false;

        if (_first)
        {
            _first = false;
            _t_prev_us = t_us;
            for (int i = 0; i < _p.order; i++) { _x1[i] = theta_deg; _y1[i] = theta_deg; }
            _theta_f = _theta_f_prev = theta_deg;
            _phase_pct = -1;
            _valid = false;
            return;
        }

        float dt = (float)(uint32_t)(t_us - _t_prev_us) * 1e-6f;
        _t_prev_us = t_us;
        if (dt <= 0.0f || dt > 0.1f) { dt = 1.0f / _fs_hz; }

        // Track the sampling rate slowly and refresh the filter if it drifts.
        const float fs_now = 1.0f / dt;
        _fs_hz += 0.01f * (fs_now - _fs_hz);
        if (fabsf(_fs_hz - _fs_used) > 0.05f * _fs_used) _update_filter();

        // 1. Adaptive low-pass filter (n cascaded first-order sections).
        float x = theta_deg;
        for (int i = 0; i < _p.order; i++)
        {
            const float y = _b0 * (x + _x1[i]) - _a1 * _y1[i];
            _x1[i] = x;
            _y1[i] = y;
            x = y;
        }
        _theta_f_prev = _theta_f;
        _theta_f = x;

        // 2. Angular velocity.
        _dtheta_prev = _dtheta;
        _dtheta = (_theta_f - _theta_f_prev) / dt;

        // Extrema of the current stride.
        if (_theta_f < _min_th)  _min_th  = _theta_f;
        if (_theta_f > _max_th)  _max_th  = _theta_f;
        if (_dtheta  < _min_dth) _min_dth = _dtheta;
        if (_dtheta  > _max_dth) _max_dth = _dtheta;

        // Maximum flexion: velocity crosses from positive to non-positive,
        // above the centre, and not too soon after the previous maximum.
        const float since_max_ms = _have_last_max ? (float)(uint32_t)(t_us - _last_max_us) * 1e-3f : 1e9f;
        const bool above_centre = !_norm_init || (_theta_f > _c_th);
        if (_dtheta_prev > 0.0f && _dtheta <= 0.0f && above_centre &&
            since_max_ms > 0.5f * _p.min_stride_ms)
        {
            if (_have_last_max)
            {
                const float T = since_max_ms;
                if (T >= _p.min_stride_ms && T <= _p.max_stride_ms)
                {
                    _stride_ms = T;
                    _update_filter();
                    _tc_th  = 0.5f * (_max_th + _min_th);
                    _ta_th  = 0.5f * (_max_th - _min_th);
                    _tc_dth = 0.5f * (_max_dth + _min_dth);
                    _ta_dth = 0.5f * (_max_dth - _min_dth);
                    if (!_norm_init)
                    {
                        _c_th = _tc_th; _a_th = _ta_th; _c_dth = _tc_dth; _a_dth = _ta_dth;
                        _norm_init = true;
                    }
                    if (_strides < 1000) _strides++;
                }
                else
                {
                    _strides = 0; // implausible stride: start over
                }
            }
            _have_last_max = true;
            _last_max_us = t_us;
            _reset_extrema();
            _max_th = _theta_f; // the stride starts at this maximum
        }

        // 3. Normalization parameters, smoothed sample by sample (Eq. 14).
        if (_norm_init)
        {
            const float a = powf(_p.norm_pole_1khz, dt * 1000.0f);
            _c_th  = a * _c_th  + (1.0f - a) * _tc_th;
            _a_th  = a * _a_th  + (1.0f - a) * _ta_th;
            _c_dth = a * _c_dth + (1.0f - a) * _tc_dth;
            _a_dth = a * _a_dth + (1.0f - a) * _ta_dth;
        }

        // Validity.
        const float since_max_now_ms = _have_last_max ? (float)(uint32_t)(t_us - _last_max_us) * 1e-3f : 1e9f;
        _valid = input_ok && _norm_init &&
                 (_strides >= _p.strides_to_valid) &&
                 (_a_th >= _p.min_amplitude_deg) && (_a_dth > 1e-3f) &&
                 (since_max_now_ms < 1.5f * _stride_ms);

        // 4. Phase.
        _prev_phase_pct = _phase_pct;
        if (!_norm_init || _a_th <= 1e-6f || _a_dth <= 1e-6f)
        {
            _phase_pct = -1;
            _stance = false;
            return;
        }
        const float th_n  = (_theta_f - _c_th) / _a_th;
        const float dth_n = (_dtheta - _c_dth) / _a_dth;
        float phi = -atan2f(dth_n, th_n)
                    + (float)_p.order * atanf(1.0f / _p.alpha)
                    - _p.phase_offset_pct * 0.01f * PP_TWO_PI;
        phi = fmodf(phi, PP_TWO_PI);
        if (phi < 0) phi += PP_TWO_PI;
        _phase_pct = 100.0f * phi / PP_TWO_PI;

        if (_valid && _prev_phase_pct >= 0)
        {
            // Virtual heel strike: wrap from the end to the start of the cycle.
            if (_prev_phase_pct > 80.0f && _phase_pct < 20.0f) _heel_strike = true;
            // Virtual toe off.
            if (_prev_phase_pct < _p.toe_off_pct && _phase_pct >= _p.toe_off_pct &&
                _phase_pct - _prev_phase_pct < 20.0f) _toe_off = true;
        }
        _stance = _valid && (_phase_pct < _p.toe_off_pct);
    }

    // ---- outputs ----
    bool  valid()        const { return _valid; }
    float phase_pct()    const { return _phase_pct; }     // -1 if undefined
    float stride_ms()    const { return _stride_ms; }
    bool  heel_strike()  const { return _heel_strike; }   // one-cycle pulse
    bool  toe_off()      const { return _toe_off; }       // one-cycle pulse
    bool  stance()       const { return _stance; }
    float stance_ms()    const { return _stride_ms * _p.toe_off_pct * 0.01f; }
    float swing_ms()     const { return _stride_ms * (1.0f - _p.toe_off_pct * 0.01f); }
    float percent_stance() const
    {
        if (!_valid || _phase_pct < 0) return -1;
        return _stance ? 100.0f * _phase_pct / _p.toe_off_pct : 0.0f;
    }
    float percent_swing() const
    {
        if (!_valid || _phase_pct < 0) return -1;
        return _stance ? 0.0f : 100.0f * (_phase_pct - _p.toe_off_pct) / (100.0f - _p.toe_off_pct);
    }
    float theta_filtered() const { return _theta_f; }
    float dtheta()         const { return _dtheta; }
    float centre()         const { return _c_th; }
    float amplitude()      const { return _a_th; }
    int   strides()        const { return _strides; }
    const PhasePortraitParams& params() const { return _p; }

private:
    void _reset_extrema()
    {
        _min_th = 1e9f; _max_th = -1e9f; _min_dth = 1e9f; _max_dth = -1e9f;
    }

    void _update_filter()
    {
        _fs_used = _fs_hz;
        float fc = _p.alpha / (_stride_ms * 1e-3f);           // Hz
        const float nyq = 0.45f * _fs_used;
        if (fc > nyq) fc = nyq;
        const float K = tanf(3.14159265f * fc / _fs_used);    // bilinear, pre-warped
        _b0 = K / (1.0f + K);
        _a1 = (K - 1.0f) / (K + 1.0f);
    }

    PhasePortraitParams _p;
    bool _first;
    uint32_t _t_prev_us = 0;
    float _fs_hz, _fs_used = 500.0f;
    float _b0 = 0, _a1 = 0;
    float _x1[MAX_ORDER], _y1[MAX_ORDER];
    float _theta_f, _theta_f_prev, _dtheta, _dtheta_prev;
    float _min_th, _max_th, _min_dth, _max_dth;
    bool _have_last_max; uint32_t _last_max_us;
    int _strides;
    float _stride_ms;
    bool _norm_init;
    float _c_th, _a_th, _c_dth, _a_dth;     // smoothed
    float _tc_th, _ta_th, _tc_dth, _ta_dth; // targets
    float _phase_pct, _prev_phase_pct;
    bool _valid, _heel_strike, _toe_off, _stance;
};

#endif
