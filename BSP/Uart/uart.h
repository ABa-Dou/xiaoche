#ifndef __UART_H
#define __UART_H

#include "sys.h"

#define UART2_RX_BUF_SIZE  256
#define UART3_RX_BUF_SIZE  256
#define UART4_RX_BUF_SIZE  512

void uart2_init(uint32_t baudrate);
void uart3_init(uint32_t baudrate);
void uart4_init(uint32_t baudrate);

void uart2_send_byte(uint8_t data);
void uart3_send_byte(uint8_t data);
void uart4_send_byte(uint8_t data);

void uart2_send_buf(uint8_t *buf, uint16_t len);
void uart3_send_buf(uint8_t *buf, uint16_t len);
void uart4_send_buf(uint8_t *buf, uint16_t len);

extern uint8_t  g_uart2_rx_buf[UART2_RX_BUF_SIZE];
extern volatile uint16_t g_uart2_rx_len;
extern volatile uint8_t  g_uart2_rx_flag;

extern uint8_t  g_uart3_rx_buf[UART3_RX_BUF_SIZE];
extern volatile uint16_t g_uart3_rx_len;
extern volatile uint8_t  g_uart3_rx_flag;

extern uint8_t  g_uart4_rx_buf[UART4_RX_BUF_SIZE];
extern volatile uint16_t g_uart4_rx_len;
extern volatile uint8_t  g_uart4_rx_flag;
extern volatile uint32_t g_uart4_irq_cnt;
extern volatile uint32_t g_uart4_rxne_cnt;

UART_HandleTypeDef *uart3_get_handle(void);
UART_HandleTypeDef *uart4_get_handle(void);

#endif