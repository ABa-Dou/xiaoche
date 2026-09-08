#ifndef __ESP_PROTO_H
#define __ESP_PROTO_H

#include "sys.h"
#include "motor.h"
#include "imu.h"
#include "lidar.h"

#define ESP_FRAME_HEADER1   0xAA
#define ESP_FRAME_HEADER2   0x55
#define ESP_FRAME_TAIL1     0x0D
#define ESP_FRAME_TAIL2     0x0A

#define ESP_TYPE_SPEED      1
#define ESP_TYPE_PULSE      2
#define ESP_TYPE_IMU        3
#define ESP_TYPE_LIDAR      4

#define ESP_MAX_PAYLOAD     4098
#define ESP_FRAME_OVERHEAD  13

typedef struct {
    float speed_mm_s[MOTOR_COUNT];
} esp_speed_payload_t;

typedef struct {
    int32_t total_pulse[MOTOR_COUNT];
} esp_pulse_payload_t;

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} esp_imu_payload_t;

void esp_proto_init(void);
void esp_proto_send_speed(uint32_t timestamp, const float speed_mm_s[MOTOR_COUNT]);
void esp_proto_send_pulse(uint32_t timestamp, const int32_t total_pulse[MOTOR_COUNT]);
void esp_proto_send_imu(uint32_t timestamp, const imu_data_t *imu);
void esp_proto_send_lidar(uint32_t timestamp, const lidar_frame_t *frame);

#endif