#include "lidar.h"
#include "easy_log.h"
#include "uart.h"
#include "delay.h"
#include <string.h>
#include <math.h>

#define LIDAR_CMD_STOP_SCAN       0x65
#define LIDAR_CMD_START_SCAN      0x60
#define LIDAR_CMD_GET_FREQ        0x0D
#define LIDAR_CMD_INC_FREQ_01HZ   0x09
#define LIDAR_CMD_DEC_FREQ_01HZ   0x0A
#define LIDAR_CMD_INC_FREQ_1HZ    0x0B
#define LIDAR_CMD_DEC_FREQ_1HZ    0x0C

#define LIDAR_FREQ_SETTLE_ERR     0.05
#define LIDAR_FREQ_RETRY_MAX      20

static uint8_t  s_ring_buf[LIDAR_RING_BUF_SIZE];
static uint16_t s_ring_head = 0;
static uint16_t s_ring_tail = 0;
static uint16_t s_ring_count = 0;

static lidar_frame_t s_frame;
static uint8_t       s_pkg_buf[LIDAR_PKG_MAX_SIZE];
static uint32_t      s_frame_counter = 0;

static lidar_point_t s_current_points[LIDAR_MAX_POINTS];
static uint16_t      s_current_point_num = 0;

static void ring_push(uint8_t byte)
{
    if (s_ring_count >= LIDAR_RING_BUF_SIZE) {
        s_ring_head = (s_ring_head + 1) % LIDAR_RING_BUF_SIZE;
        s_ring_count--;
    }
    s_ring_buf[s_ring_tail] = byte;
    s_ring_tail = (s_ring_tail + 1) % LIDAR_RING_BUF_SIZE;
    s_ring_count++;
}

static uint8_t ring_pop(void)
{
    uint8_t val = s_ring_buf[s_ring_head];
    s_ring_head = (s_ring_head + 1) % LIDAR_RING_BUF_SIZE;
    s_ring_count--;
    return val;
}

static uint8_t ring_peek(uint16_t offset)
{
    uint16_t idx = (s_ring_head + offset) % LIDAR_RING_BUF_SIZE;
    return s_ring_buf[idx];
}

void lidar_init(void)
{
    s_ring_head = 0;
    s_ring_tail = 0;
    s_ring_count = 0;
    s_current_point_num = 0;
    memset(&s_frame, 0, sizeof(s_frame));
}

void lidar_feed(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        ring_push(buf[i]);
    }
}

static void lidar_parse_frames(void)
{
    while ((s_ring_count - 0) >= 2)
    {
        if (ring_peek(0) != LIDAR_FRAME_HEADER1 || ring_peek(1) != LIDAR_FRAME_HEADER2) {
            ring_pop();
            continue;
        }

        if (s_ring_count < 10) {
            break;
        }

        uint8_t ct  = ring_peek(2);
        uint8_t lsn = ring_peek(3);

        size_t pkg_len = 10 + 2 * lsn;

        if (pkg_len > LIDAR_PKG_MAX_SIZE) {
            ring_pop();
            continue;
        }

        if (s_ring_count < pkg_len) {
            break;
        }

        for (uint16_t i = 0; i < pkg_len; i++) {
            s_pkg_buf[i] = ring_pop();
        }

        bool is_zero_package = ((ct & 0x01) == 1);

        if (is_zero_package) {
            if (s_current_point_num > 0) {
                if (s_frame.data_ready == 0) {
                    s_frame.frame_id = s_frame_counter++;
                    s_frame.point_num = s_current_point_num;
                    if (s_frame.point_num > LIDAR_MAX_POINTS) {
                        s_frame.point_num = LIDAR_MAX_POINTS;
                    }
                    memcpy(s_frame.points, s_current_points, s_frame.point_num * sizeof(lidar_point_t));
                    s_frame.data_ready = 1;
                }
                s_current_point_num = 0;
            }
        }

        uint16_t fsa = (s_pkg_buf[5] << 8) | s_pkg_buf[4];
        uint16_t lsa = (s_pkg_buf[7] << 8) | s_pkg_buf[6];

        double angle_fsa = (double)(fsa >> 1) / 64.0;
        double angle_lsa = (double)(lsa >> 1) / 64.0;

        if (angle_lsa < angle_fsa) {
            angle_lsa += 360.0;
        }

        double angle_diff = angle_lsa - angle_fsa;

        for (uint8_t i = 0; i < lsn; ++i) {
            uint16_t si_raw = (s_pkg_buf[11 + 2 * i] << 8) | s_pkg_buf[10 + 2 * i];
            double distance = (double)si_raw / 4.0;

            double angle = 0.0;
            if (lsn > 1) {
                angle = angle_fsa + (angle_diff / (lsn - 1)) * i;
            } else {
                angle = angle_fsa;
            }

            if (angle >= 360.0) {
                angle -= 360.0;
            }

            if (distance > 0.1) {
                if (s_current_point_num < LIDAR_MAX_POINTS) {
                    s_current_points[s_current_point_num].angle = (float)angle;
                    s_current_points[s_current_point_num].distance = (float)distance;
                    s_current_point_num++;
                }
            }
        }
    }
}

const lidar_frame_t *lidar_get_frame(void)
{
    lidar_parse_frames();
    return &s_frame;
}

void lidar_send_cmd(uint8_t cmd_byte)
{
    uint8_t cmd[2] = {0xA5, cmd_byte};
    LOGW("lidar send cmd: 0xA5 0x%02X", cmd_byte);
    uart4_send_buf(cmd, 2);
}

void lidar_start_scan(void)
{
    lidar_send_cmd(LIDAR_CMD_START_SCAN);
}

void lidar_stop_scan(void)
{
    lidar_send_cmd(LIDAR_CMD_STOP_SCAN);
}

static void lidar_clear_rx(void)
{
    __disable_irq();
    g_uart4_rx_flag = 0;
    g_uart4_rx_len = 0;
    __enable_irq();
    s_ring_head = 0;
    s_ring_tail = 0;
    s_ring_count = 0;
    s_current_point_num = 0;
}

static uint8_t lidar_read_sync(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint16_t total = 0;

    while (total < len) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            LOGW("lidar read_sync timeout: need=%u got=%u", len, total);
            return 0;
        }

        if (g_uart4_rx_flag) {
            __disable_irq();
            uint16_t chunk_len = g_uart4_rx_len;
            uint16_t copy_len = (len - total < chunk_len) ? (len - total) : chunk_len;
            memcpy(buf + total, g_uart4_rx_buf, copy_len);
            g_uart4_rx_flag = 0;
            __enable_irq();
            total += copy_len;
            LOGW("lidar read chunk: chunk=%u copy=%u total=%u", chunk_len, copy_len, total);
        }
    }
    return 1;
}

uint8_t lidar_set_freq(double target_hz)
{
    LOGW("lidar set freq to %.1f Hz", target_hz);

    for (int attempt = 0; attempt < 3; attempt++) {
        LOGW("lidar set freq attempt %d", attempt);

        lidar_stop_scan();
        delay_ms(50);
        lidar_clear_rx();

        uint8_t settled = 0;

        for (int retry = 0; retry < LIDAR_FREQ_RETRY_MAX; retry++) {
            lidar_send_cmd(LIDAR_CMD_GET_FREQ);

            uint8_t resp[11];

            if (!lidar_read_sync(resp, 11, 200)) {
                LOGW("lidar get freq resp timeout, retry=%d", retry);
                continue;
            }

            LOGW("lidar resp: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                 resp[0], resp[1], resp[2], resp[3], resp[4], resp[5],
                 resp[6], resp[7], resp[8], resp[9], resp[10]);

            uint8_t *ans_data = &resp[7];
            uint32_t raw_ans = ((uint32_t)ans_data[3] << 24) |
                               ((uint32_t)ans_data[2] << 16) |
                               ((uint32_t)ans_data[1] << 8)  |
                               ((uint32_t)ans_data[0]);
            double current_hz = (double)raw_ans / 100.0;
            LOGW("lidar cur freq: %.2f Hz", current_hz);

            if (fabs(current_hz - target_hz) < LIDAR_FREQ_SETTLE_ERR) {
                LOGW("lidar freq locked: %.1f Hz", target_hz);
                settled = 1;
                break;
            }

            if (current_hz < target_hz) {
                if (target_hz - current_hz >= 1.0) {
                    LOGW("lidar freq +1Hz");
                    lidar_send_cmd(LIDAR_CMD_INC_FREQ_1HZ);
                } else {
                    LOGW("lidar freq +0.1Hz");
                    lidar_send_cmd(LIDAR_CMD_INC_FREQ_01HZ);
                }
            } else {
                if (current_hz - target_hz >= 1.0) {
                    LOGW("lidar freq -1Hz");
                    lidar_send_cmd(LIDAR_CMD_DEC_FREQ_1HZ);
                } else {
                    LOGW("lidar freq -0.1Hz");
                    lidar_send_cmd(LIDAR_CMD_DEC_FREQ_01HZ);
                }
            }

            delay_ms(40);
            lidar_clear_rx();
        }

        if (!settled) {
            LOGW("lidar freq not settled, retry attempt");
            continue;
        }

        s_frame.data_ready = 0;
        lidar_start_scan();

        uint32_t wait_start = HAL_GetTick();
        while (HAL_GetTick() - wait_start < 3000) {
            if (g_uart4_rx_flag) {
                __disable_irq();
                uint16_t local_len = g_uart4_rx_len;
                uint8_t local_buf[UART4_RX_BUF_SIZE];
                memcpy(local_buf, g_uart4_rx_buf, local_len);
                g_uart4_rx_flag = 0;
                __enable_irq();
                lidar_feed(local_buf, local_len);
            }
            lidar_parse_frames();
            if (s_frame.data_ready) {
                LOGW("lidar data ok, pts=%u", s_frame.point_num);
                s_frame.data_ready = 0;
                return 1;
            }
        }

        LOGW("lidar data timeout, retry attempt");
    }

    LOGW("lidar init failed");
    return 0;
}