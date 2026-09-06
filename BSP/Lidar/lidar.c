#include "lidar.h"
#include "easy_log.h"
#include "uart.h"
#include <string.h>

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
    uart4_send_buf(cmd, 2);
}

void lidar_start_scan(void)
{
    lidar_send_cmd(0x60);
}

void lidar_stop_scan(void)
{
    lidar_send_cmd(0x65);
}