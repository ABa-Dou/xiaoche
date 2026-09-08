#include "esp_proto.h"
#include "uart.h"
#include "easy_log.h"
#include <string.h>

#define ESP_LOG_INTERVAL   50
#define ESP_LIDAR_Q_DEPTH  4
#define ESP_SMALL_Q_DEPTH  8
#define ESP_SMALL_MAX      40
#define ESP_LIDAR_MAX      (ESP_FRAME_OVERHEAD + ESP_MAX_PAYLOAD)

static uint8_t  s_seq[4] = {0};
static uint16_t s_log_cnt[4] = {0};

static uint8_t  s_small_q[ESP_SMALL_Q_DEPTH][ESP_SMALL_MAX];
static uint16_t s_small_len[ESP_SMALL_Q_DEPTH];
static volatile uint8_t s_small_widx = 0;
static volatile uint8_t s_small_ridx = 0;

static uint8_t  s_lidar_q[ESP_LIDAR_Q_DEPTH][ESP_LIDAR_MAX];
static uint16_t s_lidar_len[ESP_LIDAR_Q_DEPTH];
static volatile uint8_t s_lidar_widx = 0;
static volatile uint8_t s_lidar_ridx = 0;

static volatile uint8_t s_tx_busy = 0;
static volatile uint8_t s_tx_type = 0;
static volatile uint32_t s_tx_done_cnt = 0;
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

static uint16_t build_frame(uint8_t *out, uint8_t type, uint32_t timestamp,
                            const uint8_t *payload, uint16_t payload_len)
{
    uint16_t idx = 0;

    out[idx++] = ESP_FRAME_HEADER1;
    out[idx++] = ESP_FRAME_HEADER2;

    uint16_t length = 1 + 1 + 4 + payload_len + 2;
    out[idx++] = (uint8_t)(length & 0xFF);
    out[idx++] = (uint8_t)((length >> 8) & 0xFF);
    out[idx++] = type;
    out[idx++] = s_seq[type - 1]++;

    memcpy(&out[idx], &timestamp, 4);
    idx += 4;

    memcpy(&out[idx], payload, payload_len);
    idx += payload_len;

    uint16_t crc = crc16_calc(&out[2], idx - 2);
    out[idx++] = (uint8_t)(crc & 0xFF);
    out[idx++] = (uint8_t)((crc >> 8) & 0xFF);

    out[idx++] = ESP_FRAME_TAIL1;
    out[idx++] = ESP_FRAME_TAIL2;

    return idx;
}

static void try_send_next(void)
{
    if (s_tx_busy)
        return;

    if (s_small_ridx != s_small_widx) {
        s_tx_busy = 1;
        s_tx_type = 1;
        uart2_send_buf(s_small_q[s_small_ridx], s_small_len[s_small_ridx]);
        return;
    }

    if (s_lidar_ridx != s_lidar_widx) {
        s_tx_busy = 1;
        s_tx_type = 2;
        uart2_send_buf(s_lidar_q[s_lidar_ridx], s_lidar_len[s_lidar_ridx]);
        LOGW("DMA start lidar ridx=%u len=%u", s_lidar_ridx, s_lidar_len[s_lidar_ridx]);
        return;
    }
}

void esp_proto_tx_complete(void)
{
    if (!s_tx_busy)
        return;

    s_tx_done_cnt++;

    if (s_tx_type == 1) {
        s_small_ridx = (s_small_ridx + 1) % ESP_SMALL_Q_DEPTH;
    } else if (s_tx_type == 2) {
        s_lidar_ridx = (s_lidar_ridx + 1) % ESP_LIDAR_Q_DEPTH;
    }
    s_tx_type = 0;
    s_tx_busy = 0;

    try_send_next();
}

static void enqueue_small(uint8_t type, uint32_t timestamp,
                          const uint8_t *payload, uint16_t payload_len)
{
    uint8_t next_w = (s_small_widx + 1) % ESP_SMALL_Q_DEPTH;
    if (next_w == s_small_ridx) {
        LOGW("small Q full! type=%u", type);
        return;
    }

    s_small_len[s_small_widx] = build_frame(s_small_q[s_small_widx],
                                            type, timestamp, payload, payload_len);
    s_small_widx = next_w;
    try_send_next();
}

static void enqueue_lidar(uint8_t type, uint32_t timestamp,
                          const uint8_t *payload, uint16_t payload_len)
{
    uint8_t next_w = (s_lidar_widx + 1) % ESP_LIDAR_Q_DEPTH;
    if (next_w == s_lidar_ridx) {
        LOGW("lidar Q full!");
        return;
    }

    s_lidar_len[s_lidar_widx] = build_frame(s_lidar_q[s_lidar_widx],
                                            type, timestamp, payload, payload_len);
    s_lidar_widx = next_w;
    LOGW("lidar enqueue widx=%u ridx=%u busy=%u", s_lidar_widx, s_lidar_ridx, s_tx_busy);
    try_send_next();
}

void esp_proto_init(void)
{
    memset(s_seq, 0, sizeof(s_seq));
    s_small_widx = 0;
    s_small_ridx = 0;
    s_lidar_widx = 0;
    s_lidar_ridx = 0;
    s_tx_busy = 0;
    s_tx_type = 0;
}

void esp_proto_send_speed(uint32_t timestamp, const float speed_mm_s[MOTOR_COUNT])
{
    uint8_t payload[sizeof(float) * MOTOR_COUNT];
    memcpy(payload, speed_mm_s, sizeof(payload));
    enqueue_small(ESP_TYPE_SPEED, timestamp, payload, sizeof(payload));
}

void esp_proto_send_pulse(uint32_t timestamp, const int32_t total_pulse[MOTOR_COUNT])
{
    uint8_t payload[sizeof(int32_t) * MOTOR_COUNT];
    memcpy(payload, total_pulse, sizeof(payload));
    enqueue_small(ESP_TYPE_PULSE, timestamp, payload, sizeof(payload));
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
    enqueue_small(ESP_TYPE_IMU, timestamp, (const uint8_t *)&p, sizeof(p));
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

    enqueue_lidar(ESP_TYPE_LIDAR, timestamp, s_payload_buf, payload_len);

    s_log_cnt[3]++;
    if (s_log_cnt[3] >= ESP_LOG_INTERVAL) {
        s_log_cnt[3] = 0;
        LOGW("[LIDAR] S=%u TS=%u pts=%u plen=%u busy=%u done=%u", s_seq[3] - 1, timestamp,
             point_num, payload_len + ESP_FRAME_OVERHEAD, s_tx_busy, s_tx_done_cnt);
    }
}