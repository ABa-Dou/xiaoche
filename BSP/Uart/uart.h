#ifndef __UART_H
#define __UART_H

#include "sys.h"

void uart2_init(uint32_t baudrate);
void uart3_init(uint32_t baudrate);
void uart4_init(uint32_t baudrate);

void uart2_send_byte(uint8_t data);
void uart3_send_byte(uint8_t data);
void uart4_send_byte(uint8_t data);

void uart2_send_buf(uint8_t *buf, uint16_t len);
void uart3_send_buf(uint8_t *buf, uint16_t len);
void uart4_send_buf(uint8_t *buf, uint16_t len);

#endif