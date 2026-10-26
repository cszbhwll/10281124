/**
 * main.c —— 程序入口
 *
 * 只做三件事：
 *   1. HAL_Init()：初始化 HAL 库、1ms 系统节拍
 *   2. SystemClock_Config()：用板载 8MHz 外部晶振倍频到 168MHz（点灯程序用的是 16MHz 内部时钟）
 *   3. app_main()：进入我们自己的程序（Core/Src/app.c），永不返回
 */
#include "main.h"
#include "app.h"

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    app_main();
    while (1) {}
}

/*
 * 时钟：HSE 8MHz → PLL：/M(8) ×N(336) /P(2) = 168MHz 系统时钟
 *   AHB 168MHz，APB1 = /4 = 42MHz（USART2/3、I2C1），APB2 = /2 = 84MHz（USART1/6）
 *   1Mbps 舵机串口：42MHz/16/1M = 2.625（USART2），84MHz/16/1M = 5.25（USART6），都能精确分频
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = { 0 };
    RCC_ClkInitTypeDef clk = { 0 };

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 8;
    osc.PLL.PLLN = 336;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

/* 出错时红灯常亮、停在这里（外部晶振不起振等） */
void Error_Handler(void)
{
    __disable_irq();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    GPIO_InitTypeDef g = { 0 };
    g.Pin = GPIO_PIN_9;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOF, &g);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
