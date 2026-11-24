#include "PelvisImu.h"
#include "UARTHandler.h"
#include "uart_commands.h"
#include <math.h>

namespace pelvis_imu
{
    RawSample raw;
    State state;
}

/* ========================================================================
 * Teensy side
 * ======================================================================== */
#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)

namespace pelvis_imu
{
    void store_message(const float* data, uint8_t len)
    {
        if (len < MSG_LEN) return;
        for (int i = 0; i < 3; i++)
        {
            raw.acc_g[i] = data[i];
            raw.gyro_dps[i] = data[3 + i];
        }
        raw.seq = data[6];
        raw.nano_ms = data[7];
        raw.rx_us = micros();
        raw.count++;
        raw.fresh = true;
    }

    void update()
    {
        static bool initialised = false;
        static uint32_t last_us = 0;

        const uint32_t now = micros();
        state.age_us = (raw.count > 0) ? (uint32_t)(now - raw.rx_us) : 0xFFFFFFFFUL;
        state.ok = (raw.count > 0) && (state.age_us < PELVIS_IMU_TIMEOUT_US);

        if (!raw.fresh) return;   // filter advances only with new samples
        raw.fresh = false;

        const float a_vert = PELVIS_SIGN_VERTICAL * raw.acc_g[PELVIS_AXIS_VERTICAL];
        const float a_fwd  = PELVIS_SIGN_FORWARD  * raw.acc_g[PELVIS_AXIS_FORWARD];
        const float g_lat  = PELVIS_SIGN_LATERAL  * raw.gyro_dps[PELVIS_AXIS_LATERAL];

        // Anterior tilt moves the "up" axis toward the front: gravity appears
        // on the forward axis with positive sign.
        const float tilt_acc = atan2f(a_fwd, a_vert) * 57.2957795f;
        state.tilt_acc_deg = tilt_acc;
        state.tilt_rate_dps = g_lat;

        if (!initialised)
        {
            state.tilt_deg = tilt_acc;
            initialised = true;
            last_us = raw.rx_us;
            return;
        }

        float dt = (float)(uint32_t)(raw.rx_us - last_us) * 1e-6f;
        last_us = raw.rx_us;
        if (dt <= 0.0f || dt > 0.1f) dt = (float)NANO_IMU_PERIOD_US * 1e-6f;

        const float w = PELVIS_CF_GYRO_WEIGHT;
        state.tilt_deg = w * (state.tilt_deg + g_lat * dt) + (1.0f - w) * tilt_acc;
    }
}
#endif

/* ========================================================================
 * Nano 33 BLE side
 * ======================================================================== */
#if defined(ARDUINO_ARDUINO_NANO33BLE)

#ifdef NANO33_IMU_BMI270
  #include <Arduino_BMI270_BMM150.h>   // Library: "Arduino_BMI270_BMM150"
#else
  #include <Arduino_LSM9DS1.h>         // Library: "Arduino_LSM9DS1"
#endif

namespace nano_pelvis_imu
{
    static bool _ok = false;

    bool begin()
    {
        _ok = IMU.begin();
        return _ok;
    }

    void update()
    {
        static uint32_t last_us = 0;
        static uint32_t seq = 0;
        if (!_ok) return;

        const uint32_t now = micros();
        if ((uint32_t)(now - last_us) < NANO_IMU_PERIOD_US) return;
        if (!(IMU.accelerationAvailable() && IMU.gyroscopeAvailable())) return;
        last_us = now;

        float ax, ay, az, gx, gy, gz;
        IMU.readAcceleration(ax, ay, az);   // g
        IMU.readGyroscope(gx, gy, gz);      // deg/s

        float data[pelvis_imu::MSG_LEN] = {ax, ay, az, gx, gy, gz,
                                           (float)(seq % 1000000UL), (float)(millis() % 1000000UL)};
        seq++;
        UARTHandler::get_instance()->UART_msg(UART_command_names::update_pelvis_imu,
                                              pelvis_imu::MSG_LEN, 0, data);
    }
}
#endif
