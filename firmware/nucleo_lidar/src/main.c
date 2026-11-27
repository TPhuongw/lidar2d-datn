/*
 * main.c - khoi tao Nucleo-F411RE roi chay lidar_fw (TF03 + xung buoc + moc 0 do, goi nhi phan len PC).
 * CODE THU NGHIEM. Trang thai: xem README.md.
 *
 * Xung nhip giong cau hinh mac dinh cua board trong CubeMX: SYSCLK = HCLK = 84 MHz, APB1 = 42 MHz (timer 84 MHz), APB2 = 84 MHz.
 * Dung HSI 16 MHz noi (khong phu thuoc xung 8 MHz tu ST-Link): PLLM 16, PLLN 336, PLLP 4 -> 84 MHz; PLLQ 7 -> 48 MHz.
 */
#include "main.h"
#include "lidar_fw.h"

UART_HandleTypeDef huart2;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    lidar_init(&huart2);
    while (1) {
        lidar_loop();
    }
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

static void Error_Handler(void)
{
    __disable_irq();
    while (1) {                                     /* nhay LD2 nhanh de bao loi khoi tao */
        GPIOA->ODR ^= 1u << 5;
        for (volatile uint32_t i = 0; i < 400000; i++) {}
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = 16;
    osc.PLL.PLLN = 336;
    osc.PLL.PLLP = RCC_PLLP_DIV4;
    osc.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    g.Pin = GPIO_PIN_5;                              /* LD2 */
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin = GPIO_PIN_13;                             /* B1: muc thap khi nhan (board co dien tro keo len) */
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &g);
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = LIDAR_VCP_BAUD;          /* ST-Link VCP */
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

/* HAL goi ham nay trong HAL_UART_Init: bat xung nhip va chan PA2 (TX) / PA3 (RX), AF7 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        GPIO_InitTypeDef g = {0};
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        g.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        g.Mode = GPIO_MODE_AF_PP;
        g.Pull = GPIO_NOPULL;
        g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        g.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &g);
    }
}
