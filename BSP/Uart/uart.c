#include "uart.h"
#include "esp_proto.h"
#include "easy_log.h"

static UART_HandleTypeDef huart2;
static UART_HandleTypeDef huart3;
static UART_HandleTypeDef huart4;
static DMA_HandleTypeDef  hdma_usart2_tx;
static DMA_HandleTypeDef  hdma_usart3_rx;
static DMA_HandleTypeDef  hdma_uart4_rx;

uint8_t  g_uart3_rx_buf[UART3_RX_BUF_SIZE];
volatile uint16_t g_uart3_rx_len = 0;
volatile uint8_t  g_uart3_rx_flag = 0;

uint8_t  g_uart4_rx_buf[UART4_RX_BUF_SIZE];
volatile uint16_t g_uart4_rx_len = 0;
volatile uint8_t  g_uart4_rx_flag = 0;
volatile uint32_t g_uart4_irq_cnt = 0;
volatile uint32_t g_uart4_rxne_cnt = 0;

void uart2_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_3;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    hdma_usart2_tx.Instance                 = DMA1_Stream6;
    hdma_usart2_tx.Init.Channel             = DMA_CHANNEL_4;
    hdma_usart2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode                = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;
    hdma_usart2_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_usart2_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_usart2_tx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_usart2_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    HAL_DMA_Init(&hdma_usart2_tx);

    __HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = baudrate;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&huart2);

    __HAL_DMA_ENABLE_IT(&hdma_usart2_tx, DMA_IT_TC);
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    HAL_NVIC_SetPriority(USART2_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

static void uart3_dma_start(void)
{
    DMA_Stream_TypeDef *stream = DMA1_Stream1;

    stream->CR &= ~DMA_SxCR_EN;
    while (stream->CR & DMA_SxCR_EN) {}

    DMA1->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 |
                  DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 |
                  DMA_LIFCR_CFEIF1;

    stream->PAR  = (uint32_t)&(USART3->DR);
    stream->M0AR = (uint32_t)g_uart3_rx_buf;
    stream->NDTR = UART3_RX_BUF_SIZE;
    stream->FCR  = 0;

    stream->CR = (4U << 25)  |
                 (2U << 16)  |
                 DMA_SxCR_MINC;

    stream->CR |= DMA_SxCR_EN;

    USART3->CR3 |= USART_CR3_DMAR;
}

void uart3_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_11;
    gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &gpio);

    hdma_usart3_rx.Instance                 = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel             = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode                = DMA_NORMAL;
    hdma_usart3_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_usart3_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_usart3_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_usart3_rx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_usart3_rx.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    HAL_DMA_Init(&hdma_usart3_rx);

    __HAL_LINKDMA(&huart3, hdmarx, hdma_usart3_rx);

    huart3.Instance = USART3;
    huart3.Init.BaudRate = baudrate;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&huart3);

    HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    uart3_dma_start();
}

UART_HandleTypeDef *uart3_get_handle(void)
{
    return &huart3;
}

static void uart4_dma_start(void)
{
    DMA_Stream_TypeDef *stream = DMA1_Stream2;

    stream->CR &= ~DMA_SxCR_EN;
    while (stream->CR & DMA_SxCR_EN) {}

    DMA1->LIFCR = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 |
                  DMA_LIFCR_CTEIF2 | DMA_LIFCR_CDMEIF2 |
                  DMA_LIFCR_CFEIF2;

    stream->PAR  = (uint32_t)&(UART4->DR);
    stream->M0AR = (uint32_t)g_uart4_rx_buf;
    stream->NDTR = UART4_RX_BUF_SIZE;
    stream->FCR  = 0;

    stream->CR = (4U << 25)  |
                 (2U << 16)  |
                 DMA_SxCR_MINC;

    stream->CR |= DMA_SxCR_EN;

    UART4->CR3 |= USART_CR3_DMAR;
}

void uart4_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

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

    HAL_NVIC_SetPriority(UART4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);

    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);

    uart4_dma_start();
}

UART_HandleTypeDef *uart4_get_handle(void)
{
    return &huart4;
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
    HAL_StatusTypeDef ret = HAL_UART_Transmit_DMA(&huart2, buf, len);
    if (ret != HAL_OK) {
        LOGW("uart2 DMA ret=%d gState=%d", ret, huart2.gState);
    }
}

void uart3_send_buf(uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&huart3, buf, len, HAL_MAX_DELAY);
}

void uart4_send_buf(uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&huart4, buf, len, HAL_MAX_DELAY);
}

void USART3_IRQHandler(void)
{
    if (USART3->SR & UART_FLAG_IDLE)
    {
        USART3->SR;
        USART3->DR;

        uint32_t ndtr = DMA1_Stream1->NDTR;
        uint16_t rx_len = UART3_RX_BUF_SIZE - ndtr;

        DMA1_Stream1->CR &= ~DMA_SxCR_EN;
        USART3->CR3 &= ~USART_CR3_DMAR;

        if (rx_len > 0 && rx_len < UART3_RX_BUF_SIZE)
        {
            g_uart3_rx_len = rx_len;
            g_uart3_rx_flag = 1;
        }

        uart3_dma_start();
    }
}

void UART4_IRQHandler(void)
{
    uint32_t sr = UART4->SR;

    if (sr & UART_FLAG_IDLE)
    {
        g_uart4_irq_cnt++;

        DMA1_Stream2->CR &= ~DMA_SxCR_EN;
        while (DMA1_Stream2->CR & DMA_SxCR_EN) {}
        UART4->CR3 &= ~USART_CR3_DMAR;

        UART4->DR;

        uint32_t ndtr = DMA1_Stream2->NDTR;
        uint16_t rx_len = UART4_RX_BUF_SIZE - ndtr;

        if (rx_len > 0 && rx_len < UART4_RX_BUF_SIZE)
        {
            g_uart4_rx_len = rx_len;
            g_uart4_rx_flag = 1;
        }

        uart4_dma_start();
    }
    else if (sr & (UART_FLAG_FE | UART_FLAG_ORE))
    {
        DMA1_Stream2->CR &= ~DMA_SxCR_EN;
        while (DMA1_Stream2->CR & DMA_SxCR_EN) {}
        UART4->CR3 &= ~USART_CR3_DMAR;
        UART4->DR;
        uart4_dma_start();
    }
}

void DMA1_Stream6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        esp_proto_tx_complete();
    }
}