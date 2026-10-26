/**
 * port_stm32.c —— 硬件抽象层在 STM32F407（正点原子探索者）上的实现
 *
 * 引脚分配（对照 EXPLORER_V3.4 原理图选的空闲/可用引脚）：
 *   I2C1   PB8=SCL  PB9=SDA     → 幻尔电机驱动板（与板载 IIC 器件同一条总线，地址不冲突）
 *   USART1 PA9=TX   PA10=RX     → 板载 USB 转串口（CH340），日志/调试命令，115200
 *   USART2 PA2=TX   PA3=RX      → ST3215 舵机驱动板，1Mbps 【注意】拔掉 COM2/RS485 选择跳线帽
 *   USART6 PC6=TX   PC7=RX      → SC09 舵机驱动板，1Mbps（不接 OV 摄像头模块时空闲）
 *   USART3 PB10=TX  PB11=RX     → 视觉模块，115200（ATK 模块接口）
 *   KEY0 PE4 / KEY1 PE3 / KEY2 PE2（按下为低）  WK_UP PA0（按下为高）
 *   LED  DS0=PF9  DS1=PF10（低电平亮）   BEEP PF8（高电平响）
 * 串口接线：单片机 TX 接对方 RX，RX 接对方 TX，并且必须共地（GND 连在一起）。
 */
#include "port.h"
#include "stm32f4xx_hal.h"
#include <string.h>

I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart1, huart2, huart3, huart6;

/* ---------------- 串口接收环形缓冲区 ---------------- */
#define RB_SIZE 512
typedef struct {
    volatile uint16_t head, tail;
    uint8_t buf[RB_SIZE];
} ringbuf_t;
static ringbuf_t s_rb[PORT_UART_COUNT];

static UART_HandleTypeDef *const UART_OF[PORT_UART_COUNT] = { &huart1, &huart2, &huart6, &huart3 };

static void rb_put(ringbuf_t *r, uint8_t c)
{
    uint16_t n = (uint16_t)((r->head + 1) % RB_SIZE);
    if (n != r->tail) { r->buf[r->head] = c; r->head = n; }   /* 满了就丢弃 */
}

/* 中断里直接读寄存器，比 HAL 的接收中断更简单可靠 */
static void uart_irq(port_uart_t ch)
{
    USART_TypeDef *u = UART_OF[ch]->Instance;
    uint32_t sr = u->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        uint8_t c = (uint8_t)u->DR;            /* 读 DR 同时清除 RXNE 和错误标志 */
        if (sr & USART_SR_RXNE) rb_put(&s_rb[ch], c);
    }
}
void USART1_IRQHandler(void) { uart_irq(PORT_UART_LOG); }
void USART2_IRQHandler(void) { uart_irq(PORT_UART_SERVO_ST); }
void USART6_IRQHandler(void) { uart_irq(PORT_UART_SERVO_SC); }
void USART3_IRQHandler(void) { uart_irq(PORT_UART_VISION); }

/* ---------------- 引脚与时钟（HAL 初始化时自动调用） ---------------- */
void HAL_I2C_MspInit(I2C_HandleTypeDef *h)
{
    if (h->Instance != I2C1) return;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = { 0 };
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode = GPIO_MODE_AF_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &g);
    __HAL_RCC_I2C1_CLK_ENABLE();
}

void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
    GPIO_InitTypeDef g = { 0 };
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    if (h->Instance == USART1) {
        __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_USART1_CLK_ENABLE();
        g.Pin = GPIO_PIN_9 | GPIO_PIN_10; g.Alternate = GPIO_AF7_USART1; HAL_GPIO_Init(GPIOA, &g);
        HAL_NVIC_SetPriority(USART1_IRQn, 6, 0); HAL_NVIC_EnableIRQ(USART1_IRQn);
    } else if (h->Instance == USART2) {
        __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_USART2_CLK_ENABLE();
        g.Pin = GPIO_PIN_2 | GPIO_PIN_3; g.Alternate = GPIO_AF7_USART2; HAL_GPIO_Init(GPIOA, &g);
        HAL_NVIC_SetPriority(USART2_IRQn, 5, 0); HAL_NVIC_EnableIRQ(USART2_IRQn);
    } else if (h->Instance == USART3) {
        __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_USART3_CLK_ENABLE();
        g.Pin = GPIO_PIN_10 | GPIO_PIN_11; g.Alternate = GPIO_AF7_USART3; HAL_GPIO_Init(GPIOB, &g);
        HAL_NVIC_SetPriority(USART3_IRQn, 6, 0); HAL_NVIC_EnableIRQ(USART3_IRQn);
    } else if (h->Instance == USART6) {
        __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_USART6_CLK_ENABLE();
        g.Pin = GPIO_PIN_6 | GPIO_PIN_7; g.Alternate = GPIO_AF8_USART6; HAL_GPIO_Init(GPIOC, &g);
        HAL_NVIC_SetPriority(USART6_IRQn, 5, 0); HAL_NVIC_EnableIRQ(USART6_IRQn);
    }
}

static void uart_open(UART_HandleTypeDef *h, USART_TypeDef *inst, uint32_t baud)
{
    h->Instance = inst;
    h->Init.BaudRate = baud;
    h->Init.WordLength = UART_WORDLENGTH_8B;
    h->Init.StopBits = UART_STOPBITS_1;
    h->Init.Parity = UART_PARITY_NONE;
    h->Init.Mode = UART_MODE_TX_RX;
    h->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    h->Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(h);
    __HAL_UART_ENABLE_IT(h, UART_IT_RXNE);
}

/* I2C 总线卡死恢复：SDA 被从机拉住时，手动在 SCL 上打 9 个时钟 */
static void i2c_bus_recover(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = { 0 };
    g.Pin = GPIO_PIN_8;
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_9;
    g.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOB, &g);
    for (int i = 0; i < 9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        for (volatile int d = 0; d < 200; d++) {}
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        for (volatile int d = 0; d < 200; d++) {}
    }
}

static void i2c_open(void)
{
    i2c_bus_recover();
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

static void i2c_reset(void)
{
    HAL_I2C_DeInit(&hi2c1);
    i2c_open();
}

static void gpio_open(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    GPIO_InitTypeDef g = { 0 };

    g.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;        /* KEY2 KEY1 KEY0 */
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &g);

    g.Pin = GPIO_PIN_0;                                  /* WK_UP */
    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &g);

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_SET);   /* LED 灭 */
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);               /* 蜂鸣器关 */
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &g);
}

/* ---------------- 对外接口 ---------------- */
void port_init(void)
{
    memset(s_rb, 0, sizeof s_rb);
    gpio_open();
    uart_open(&huart1, USART1, 115200);
    uart_open(&huart2, USART2, 1000000);
    uart_open(&huart6, USART6, 1000000);
    uart_open(&huart3, USART3, 115200);
    i2c_open();
}

uint32_t port_millis(void) { return HAL_GetTick(); }
void port_idle(void) {}

int port_i2c_write(uint8_t addr7, uint8_t reg, const uint8_t *data, uint16_t len)
{
    if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr7 << 1), reg, I2C_MEMADD_SIZE_8BIT,
                          (uint8_t *)data, len, 10) == HAL_OK) return 0;
    i2c_reset();
    return -1;
}

int port_i2c_read(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr7 << 1), reg, I2C_MEMADD_SIZE_8BIT,
                         data, len, 10) == HAL_OK) return 0;
    i2c_reset();
    return -1;
}

int port_i2c_probe(uint8_t addr7)
{
    return HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr7 << 1), 2, 5) == HAL_OK ? 0 : -1;
}

void port_uart_write(port_uart_t ch, const uint8_t *data, uint16_t len)
{
    if (ch >= PORT_UART_COUNT || len == 0) return;
    HAL_UART_Transmit(UART_OF[ch], (uint8_t *)data, len, 20 + len / 8);
}

int port_uart_read_byte(port_uart_t ch, uint8_t *out)
{
    ringbuf_t *r = &s_rb[ch];
    if (r->tail == r->head) return 0;
    *out = r->buf[r->tail];
    r->tail = (uint16_t)((r->tail + 1) % RB_SIZE);
    return 1;
}

void port_uart_flush_rx(port_uart_t ch) { s_rb[ch].tail = s_rb[ch].head; }

bool port_key_down(port_key_t key)
{
    switch (key) {
    case PORT_KEY_START: return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_RESET;
    case PORT_KEY_ZONE:  return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET;
    case PORT_KEY_AUX:   return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET;
    case PORT_KEY_WKUP:  return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET;
    default: return false;
    }
}

void port_led(int idx, bool on)
{
    HAL_GPIO_WritePin(GPIOF, idx == 0 ? GPIO_PIN_9 : GPIO_PIN_10, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void port_beep(bool on) { HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }
