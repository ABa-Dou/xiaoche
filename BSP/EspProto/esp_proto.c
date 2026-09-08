#include "esp_proto.h"
#include "uart.h"
#include "easy_log.h"
#include <string.h>

static uint8_t s_seq[4] = {0};
static uint8_t s_frame_buf[ESP_FRAME_OVERHEAD + ESP_MAX_PAYLOAD];
static uint8_t s_payload_buf[ESP_MAX_PAYLOAD];

static uint16_t crc16_calc(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint16_t j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

static void build_and_send(uint8_t type, uint32_t timestamp, const uint8_t *payload, uint16_t payload_len)
{
    uint16_t idx = 0;

    s_frame_buf[idx++] = ESP_FRAME_HEADER1;
    s_frame_buf[idx++] = ESP_FRAME_HEADER2;

    uint16_t length = 1 + 1 + 4 + payload_len + 2;
    s_frame_buf[idx++] = (uint8_t)(length & 0xFF);
    s_frame_buf[idx++] = (uint8_t)((length >> 8) & 0xFF);
    s_frame_buf[idx++] = type;
    s_frame_buf[idx++] = s_seq[type - 1]++;

    memcpy(&s_frame_buf[idx], &timestamp, 4);
    idx += 4;

    memcpy(&s_frame_buf[idx], payload, payload_len);
    idx += payload_len;

    uint16_t crc_start = 2;
    uint16_t crc = crc16_calc(&s_frame_buf[crc_start], idx - crc_start);
    s_frame_buf[idx++] = (uint8_t)(crc & 0xFF);
    s_frame_buf[idx++] = (uint8_t)((crc >> 8) & 0xFF);

    s_frame_buf[idx++] = ESP_FRAME_TAIL1;
    s_frame_buf[idx++] = ESP_FRAME_TAIL2;

    uart2_send_buf(s_frame_buf, idx);
}

void esp_proto_init(void)
{
    memset(s_seq, 0, sizeof(s_seq));
}

void esp_proto_send_speed(uint32_t timestamp, const float speed_mm_s[MOTOR_COUNT])
{
    uint8_t payload[sizeof(float) * MOTOR_COUNT];
    memcpy(payload, speed_mm_s, sizeof(payload));
    build_and_send(ESP_TYPE_SPEED, timestamp, payload, sizeof(payload));
}

void esp_proto_send_pulse(uint32_t timestamp, const int32_t total_pulse[MOTOR_COUNT])
{
    uint8_t payload[sizeof(int32_t) * MOTOR_COUNT];
    memcpy(payload, total_pulse, sizeof(payload));
    build_and_send(ESP_TYPE_PULSE, timestamp, payload, sizeof(payload));
}

void esp_proto_send_imu(uint32_t timestamp, const imu_data_t *imu)
{
    esp_imu_payload_t p;
    p.accel_x = imu->accel_x;
    p.accel_y = imu->accel_y;
    p.accel_z = imu->accel_z;
    p.gyro_x  = imu->gyro_x;
    p.gyro_y  = imu->gyro_y;
    p.gyro_z  = imu->gyro_z;
    build_and_send(ESP_TYPE_IMU, timestamp, (const uint8_t *)&p, sizeof(p));
}

void esp_proto_send_lidar(uint32_t timestamp, const lidar_frame_t *frame)
{
    uint16_t point_num = frame->point_num;
    if (point_num > LIDAR_MAX_POINTS)
        point_num = LIDAR_MAX_POINTS;

    uint16_t payload_len = 2 + (uint16_t)point_num * sizeof(lidar_point_t);

    s_payload_buf[0] = (uint8_t)(point_num & 0xFF);
    s_payload_buf[1] = (uint8_t)((point_num >> 8) & 0xFF);
    memcpy(&s_payload_buf[2], frame->points, point_num * sizeof(lidar_point_t));

    build_and_send(ESP_TYPE_LIDAR, timestamp, s_payload_buf, payload_len);
}