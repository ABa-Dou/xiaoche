#ifndef __IMU_H
#define __IMU_H

#include "sys.h"

#define IMU_FRAME_HEADER1   0x7E
#define IMU_FRAME_HEADER2   0x23
#define IMU_CMD_ACC_GYRO    0x04
#define IMU_CMD_CALIBRATE   0x70
#define IMU_CMD_CALIB_RESP  0x81
#define IMU_CMD_SET_FREQ    0x60

#define IMU_RING_BUF_SIZE   512
#define IMU_PKG_MAX_SIZE    40

#define IMU_ACCEL_RANGE_G       16.0f
#define IMU_GYRO_RANGE_DPS      2000.0f
#define IMU_RAW_FULL_SCALE      32767.0f
#define IMU_GRAVITY_MPS2        9.80665f
#define IMU_DEG_TO_RAD          0.017453292519943295f

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    uint8_t data_ready;
} imu_data_t;

void imu_init(void);
void imu_feed(const uint8_t *buf, uint16_t len);
const imu_data_t *imu_get_data(void);
void imu_calibrate(void);
uint8_t imu_set_freq(uint8_t hz);

#endif