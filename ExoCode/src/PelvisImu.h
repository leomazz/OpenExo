/**
 * @file PelvisImu.h
 * @brief Pelvis IMU on the Arduino Nano 33 BLE, streamed to the Teensy over
 *        the existing UART link (command UART_command_names::update_pelvis_imu).
 *
 * Nano  : nano_pelvis_imu::update() samples the onboard IMU at NANO_IMU_PERIOD_US
 *         and sends [ax ay az (g), gx gy gz (deg/s), seq, nano_ms].
 * Teensy: the UART handler stores the raw sample in pelvis_imu::raw;
 *         pelvis_imu::update() (once per control cycle) runs a complementary
 *         filter and gives the pelvis anterior tilt in pelvis_imu::state.
 */
#ifndef PELVIS_IMU_H
#define PELVIS_IMU_H

#include "Arduino.h"
#include <stdint.h>
#include "FsrFreeConfig.h"

namespace pelvis_imu
{
    static const uint8_t MSG_LEN = 8;   // number of floats in the UART message

    struct RawSample
    {
        float acc_g[3] = {0, 0, 0};
        float gyro_dps[3] = {0, 0, 0};
        float seq = -1;          // sample counter on the Nano (wraps at 1e6)
        float nano_ms = 0;       // millis() on the Nano when sampled
        uint32_t rx_us = 0;      // micros() on the Teensy when received
        uint32_t count = 0;      // number of messages received
        bool fresh = false;      // set by the UART handler, cleared by update()
    };

    struct State
    {
        float tilt_deg = 0;      // pelvis anterior tilt (complementary filter)
        float tilt_rate_dps = 0; // gyroscope about the lateral axis, anterior +
        float tilt_acc_deg = 0;  // accelerometer-only inclination
        bool  ok = false;        // data received within PELVIS_IMU_TIMEOUT_US
        uint32_t age_us = 0;     // time since the last message
    };

    extern RawSample raw;
    extern State state;

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
    /** Called by the UART handler when a pelvis IMU message arrives. */
    void store_message(const float* data, uint8_t len);

    /** Complementary filter; call once per control cycle. */
    void update();
#endif
}

#if defined(ARDUINO_ARDUINO_NANO33BLE)
namespace nano_pelvis_imu
{
    /** Starts the onboard IMU; returns false if it was not found. */
    bool begin();

    /** Samples the IMU and sends it to the Teensy every NANO_IMU_PERIOD_US. */
    void update();
}
#endif

#endif
