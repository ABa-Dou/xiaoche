#ifndef __LIDAR_H
#define __LIDAR_H

#include "sys.h"

#define LIDAR_FRAME_HEADER1     0xAA
#define LIDAR_FRAME_HEADER2     0x55

#define LIDAR_RING_BUF_SIZE     2048
#define LIDAR_PKG_MAX_SIZE      256
#define LIDAR_MAX_POINTS        400
#define LIDAR_MAX_FRAMES        4

typedef struct {
    float angle;
    float distance;
} lidar_point_t;

typedef struct {
    lidar_point_t points[LIDAR_MAX_POINTS];
    uint16_t point_num;
    uint32_t frame_id;
    uint8_t  data_ready;
} lidar_frame_t;

void lidar_init(void);
void lidar_feed(const uint8_t *buf, uint16_t len);
const lidar_frame_t *lidar_get_frame(void);

void lidar_send_cmd(uint8_t cmd_byte);
void lidar_start_scan(void);
void lidar_stop_scan(void);

#endif