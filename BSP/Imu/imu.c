#include "imu.h"
#include "easy_log.h"
#include <string.h>

static uint8_t  s_ring_buf[IMU_RING_BUF_SIZE];
static uint16_t s_ring_head = 0;
static uint16_t s_ring_tail = 0;
static uint16_t s_ring_count = 0;

static imu_data_t s_imu_data;
static uint8_t    s_pkg_buf[IMU_PKG_MAX_SIZE];

static const float s_accel_ratio = (IMU_ACCEL_RANGE_G / IMU_RAW_FULL_SCALE) * IMU_GRAVITY_MPS2;
static const float s_gyro_ratio  = (IMU_GYRO_RANGE_DPS / IMU_RAW_FULL_SCALE) * IMU_DEG_TO_RAD;

static void ring_push(uint8_t byte)
{
    if (s_ring_count >= IMU_RING_BUF_SIZE) {
        s_ring_head = (s_ring_head + 1) % IMU_RING_BUF_SIZE;
        s_ring_count--;
    }
    s_ring_buf[s_ring_tail] = byte;
    s_ring_tail = (s_ring_tail + 1) % IMU_RING_BUF_SIZE;
    s_ring_count++;
}

static uint8_t ring_pop(void)
{
    uint8_t val = s_ring_buf[s_ring_head];
    s_ring_head = (s_ring_head + 1) % IMU_RING_BUF_SIZE;
    s_ring_count--;
    return val;
}

static uint8_t ring_peek(uint16_t offset)
{
    uint16_t idx = (s_ring_head + offset) % IMU_RING_BUF_SIZE;
    return s_ring_buf[idx];
}

static int16_t combine_int16_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

void imu_init(void)
{
    s_ring_head = 0;
    s_ring_tail = 0;
    s_ring_count = 0;
    memset(&s_imu_data, 0, sizeof(s_imu_data));
}

void imu_feed(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        ring_push(buf[i]);
    }
}

static void imu_parse_frames(void)
{
    while (s_ring_count >= 3) {
        uint8_t h1 = ring_peek(0);
        uint8_t h2 = ring_peek(1);

        if (h1 != IMU_FRAME_HEADER1 || h2 != IMU_FRAME_HEADER2) {
            ring_pop();
            continue;
        }

        uint8_t pkg_len = ring_peek(2);

        if (pkg_len > IMU_PKG_MAX_SIZE || pkg_len < 5) {
            ring_pop();
            continue;
        }

        if (s_ring_count < pkg_len) {
            break;
        }

        for (uint16_t i = 0; i < pkg_len; i++) {
            s_pkg_buf[i] = ring_pop();
        }

        uint8_t calc_sum = 0;
        for (uint16_t i = 0; i < pkg_len - 1; i++) {
            calc_sum += s_pkg_buf[i];
        }
        if (calc_sum != s_pkg_buf[pkg_len - 1]) {
            LOGE("IMU chk err: expect 0x%02X got 0x%02X", s_pkg_buf[pkg_len - 1], calc_sum);
            continue;
        }

        uint8_t cmd = s_pkg_buf[3];
        uint8_t *payload = &s_pkg_buf[4];

        if (cmd == IMU_CMD_ACC_GYRO) {
            int16_t raw_ax = combine_int16_le(&payload[0]);
            int16_t raw_ay = combine_int16_le(&payload[2]);
            int16_t raw_az = combine_int16_le(&payload[4]);
            int16_t raw_gx = combine_int16_le(&payload[6]);
            int16_t raw_gy = combine_int16_le(&payload[8]);
            int16_t raw_gz = combine_int16_le(&payload[10]);

            s_imu_data.accel_x = (float)raw_ax * s_accel_ratio;
            s_imu_data.accel_y = (float)raw_ay * s_accel_ratio;
            s_imu_data.accel_z = (float)raw_az * s_accel_ratio;
            s_imu_data.gyro_x  = (float)raw_gx * s_gyro_ratio;
            s_imu_data.gyro_y  = (float)raw_gy * s_gyro_ratio;
            s_imu_data.gyro_z  = (float)raw_gz * s_gyro_ratio;
            s_imu_data.data_ready = 1;
        }
    }
}

const imu_data_t *imu_get_data(void)
{
    imu_parse_frames();
    return &s_imu_data;
}