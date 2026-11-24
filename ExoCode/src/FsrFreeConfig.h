/**
 * @file FsrFreeConfig.h
 * @brief User settings for the FSR-free gait phase estimation, the pelvis IMU
 *        streamed by the Nano 33 BLE, and the SD-card data logger.
 *
 * Everything that must be checked on the real device is marked "CHECK".
 */
#ifndef FSR_FREE_CONFIG_H
#define FSR_FREE_CONFIG_H

/* ------------------------------------------------------------------------
 * 1) Source of the gait phase used by the controllers
 * ------------------------------------------------------------------------
 *  PHASE_SOURCE_FSR          original OpenExo pipeline (condition C1)
 *  PHASE_SOURCE_ENCODER      phase portrait of the hip encoder angle
 *  PHASE_SOURCE_ENCODER_IMU  phase portrait of the thigh angle
 *                            (hip encoder + pelvis pitch from the Nano IMU)
 *
 *  Both estimators ALWAYS run and are ALWAYS logged, whatever is selected
 *  here; this only decides which one drives the assistance.
 */
#define PHASE_SOURCE_FSR          0
#define PHASE_SOURCE_ENCODER      1
#define PHASE_SOURCE_ENCODER_IMU  2

#ifndef PHASE_SOURCE
#define PHASE_SOURCE PHASE_SOURCE_FSR
#endif

/* ------------------------------------------------------------------------
 * 2) Sign conventions (CHECK on the device before any trial)
 * ------------------------------------------------------------------------
 *  Hip angle used by the estimator = HIP_FLEXION_SIGN * joint position,
 *  so that FLEXION IS POSITIVE on both sides.
 *  Walk slowly with the motors at zero torque and look at the logged
 *  hip_deg: it must increase when the thigh moves forward.
 */
#define HIP_FLEXION_SIGN_LEFT   (1.0f)   // CHECK
#define HIP_FLEXION_SIGN_RIGHT  (1.0f)   // CHECK

/*
 *  Thigh angle relative to the vertical (flexion positive):
 *      thigh_deg = hip_deg - pelvis_anterior_tilt_deg
 *  (if the pelvis tilts forward with the thigh still, the encoder reads more
 *  flexion, so the pelvis tilt must be subtracted).
 *  The pelvis axes below define pelvis_anterior_tilt_deg; see section 3.
 */

/* ------------------------------------------------------------------------
 * 3) Mounting of the Nano 33 BLE in the control box (CHECK)
 * ------------------------------------------------------------------------
 *  Axis index: 0 = x, 1 = y, 2 = z of the Nano IMU.
 *  Sign: +1 or -1 so that the named direction is positive.
 *
 *  VERTICAL : axis that points UP when the user stands still
 *             (the accelerometer reads about +1 g on it after the sign).
 *  FORWARD  : axis that points to the user's front.
 *  LATERAL  : axis about which the pelvis tilts forward; sign chosen so that
 *             the gyroscope is POSITIVE for an ANTERIOR tilt.
 *
 *  Quick check: stand still -> logged acc on VERTICAL axis ~ +1 g;
 *  lean the trunk forward -> pelvis_tilt_deg increases.
 */
#define PELVIS_AXIS_VERTICAL    2      // CHECK
#define PELVIS_SIGN_VERTICAL    (1.0f) // CHECK
#define PELVIS_AXIS_FORWARD     0      // CHECK
#define PELVIS_SIGN_FORWARD     (1.0f) // CHECK
#define PELVIS_AXIS_LATERAL     1      // CHECK
#define PELVIS_SIGN_LATERAL     (1.0f) // CHECK

/* Complementary filter: weight of the gyroscope integration (0..1). */
#define PELVIS_CF_GYRO_WEIGHT   (0.98f)

/* IMU data older than this is considered lost (safeguard and logging). */
#define PELVIS_IMU_TIMEOUT_US   (50000UL)

/* Nano side: IMU sampling period sent to the Teensy. */
#define NANO_IMU_PERIOD_US      (10000UL)  // 100 Hz

/* Nano 33 BLE revision: Rev2 boards carry a BMI270 + BMM150, the original
 * board carries an LSM9DS1. Uncomment for Rev2. (CHECK the silkscreen)   */
//#define NANO33_IMU_BMI270

/* ------------------------------------------------------------------------
 * 4) Phase-portrait estimator (Choi et al., Sensors 2023, 23:8276)
 * ------------------------------------------------------------------------ */
#define PP_ALPHA               (2.5f)   // cutoff = ALPHA * gait frequency (Choi: 2.5)
#define PP_FILTER_ORDER        (2)      // Butterworth order (n = 2 inferred from Choi's 0.761 rad delay)
#define PP_NORM_POLE_1KHZ      (0.98f)  // pole of the normalization IIR at 1 kHz (Choi: 0.98)
#define PP_PHASE_OFFSET_PCT    (17.0f)  // phi_off: heel strike after max hip flexion, % of cycle.
                                        // Starting value from Choi 2023 (HS ~17 %); IDENTIFY from your data.
#define PP_TOE_OFF_PCT         (60.0f)  // toe-off position in the cycle (after HS), %. IDENTIFY from your data.
#define PP_INITIAL_STRIDE_MS   (1100.0f)// initial guess of the stride period
#define PP_MIN_STRIDE_MS       (600.0f) // safeguard: valid stride period range
#define PP_MAX_STRIDE_MS       (2000.0f)
#define PP_MIN_AMPLITUDE_DEG   (5.0f)   // half range of the angle below which the gait is not trusted
#define PP_STRIDES_TO_VALID    (2)      // strides needed after start before the phase is trusted

/* ------------------------------------------------------------------------
 * 5) SD-card logger (Teensy 4.1 built-in slot)
 * ------------------------------------------------------------------------
 *  A new file LOGnnn.BIN is opened when a trial starts in the app and closed
 *  when the trial stops. STOP THE TRIAL IN THE APP BEFORE SWITCHING OFF,
 *  otherwise the end of the file is not written.
 *  Convert with tools/oexlog_to_csv.py.
 */
#define DATALOG_ENABLE         1
#define DATALOG_DECIMATION     1                      // 1 = every control loop (500 Hz)
#define DATALOG_PREALLOC_BYTES (300UL * 1024UL * 1024UL) // ~ 30 min at 500 Hz
#define DATALOG_RING_BYTES     (64 * 1024)

#endif
