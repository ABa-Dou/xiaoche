#include "uart.h"

static UART_HandleTypeDef huart2;
static UART_HandleTypeDef huart3;
static UART_HandleTypeDef huart4;

void uart2_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_3;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = baudrate;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&huart2);
}

void uart3_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_11;
    gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &gpio);

    huart3.Instance = USART3;
    huart3.Init.BaudRate = baudrate;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&huart3);
}

void uart4_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin = GPIO_PIN_11;
    gpio.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &gpio);

    huart4.Instance = UART4;
    huart4.Init.BaudRate = baudrate;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&huart4);
}

void uart2_send_byte(uint8_t data)
{
    HAL_UART_Transmit(&huart2, &data, 1, HAL_MAX_DELAY);
}

void uart3_send_byte(uint8_t data)
{
    HAL_UART_Transmit(&huart3, &data, 1, HAL_MAX_DELAY);
}

void uart4_send_byte(uint8_t data)
{
    HAL_UART_Transmit(&huart4, &data, 1, HAL_MAX_DELAY);
}

void uart2_send_buf(uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&huart2, buf, len, HAL_MAX_DELAY);
}

void uart3_send_buf(uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&huart3, buf, len, HAL_MAX_DELAY);
}

void uart4_send_buf(uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&huart4, buf, len, HAL_MAX_DELAY);
}