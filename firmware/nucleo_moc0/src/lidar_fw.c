/*
 * lidar_fw.c - Nucleo-F411RE thay Arduino UNO trong index_flag_test.ino.
 *
 * CODE THU NGHIEM. Trang thai: bien dich sach voi PlatformIO (stm32cube 1.28.3, gcc 7.2.1, -Wall -Wextra); CHUA chay tren phan cung.
 *
 * Giu NGUYEN giao thuc cua sketch UNO de lidar_app chay khong can sua:
 *   - Lenh (ST-Link VCP = USART2, 115200, ket thuc bang Newline): r <vong/s>, r 0, d, z, ?
 *   - Moi lan qua moc in: "MOC 0 t=<us> #<n> | buoc/vong <s> | <T> ms | lech ..."
 *     t = thoi diem canh moc do TIM2 chot bang phan cung (input capture, 1 us, quay vong 2^32 us nhu micros()).
 *
 * Chan (Arduino tren Nucleo -> STM32), theo PCB v0.1:
 *   D10 = PB6  PUL  (GPIO day-keo, dao muc trong ngat TIM4, dem tung buoc nhu Timer1 cua UNO)
 *   D7  = PA8  DIR
 *   D3  = PB3  moc 0 do (TIM2_CH2, AF1, keo len noi). PB3 mac dinh la SWO: KHONG bat SWO trace khi debug.
 *   LD2 = PA5  dao trang thai moi lan qua moc;  B1 = PC13  nhan de giam toc ve 0
 *
 * Noi driver: mac dinh qua transistor NPN (PUL+/DIR+ -> 5 V, PUL-/DIR- -> collector, emitter -> GND,
 * base <- 1 kOhm <- chan STM32). Chan STM32 muc CAO = opto trong driver SANG. Khong phu thuoc chan co chiu 5 V hay khong.
 * Neu da xac nhan chan la FT (bang pin definitions trong datasheet DS10314) va muon noi thang kieu anode chung
 * (PUL+ -> 5 V, PUL- -> chan STM32) thi dat OUT_OPEN_DRAIN = 1: chan open-drain, muc THAP = opto sang.
 */
#include "lidar_fw.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OUT_OPEN_DRAIN   0          /* 0: qua transistor NPN (mac dinh). 1: noi thang, chi khi chan la FT */
#define IDX_RISING       1          /* 1: canh len la moc (giong UNO: la chan che tia -> chan len cao) */
#define STEPS_PER_REV    3200L      /* PHAI khop DIP driver */
#define ACCEL_RPS2       1.0f       /* vong/s^2 */
#define MAX_RPS          5.0f
#define TIM4_TICK_HZ     2000000UL  /* 2 MHz nhu Timer1 UNO: ARR = 1e6/f - 1 */
#define TIM2_TICK_HZ     1000000UL  /* 1 us cho thoi diem moc */

static UART_HandleTypeDef *uart;

/* ---- phat xung + dem buoc (TIM4, ngat moi nua chu ky) ---- */
static volatile int32_t  stepAbs = 0;        /* so buoc tuyet doi, co dau theo chieu */
static volatile int8_t   stepDir = 1;
static volatile uint8_t  pulHigh = 0;
static volatile uint8_t  running = 0;
static volatile uint16_t nextArr = 60000;

static inline void pul_set(int active)
{
    /* active = opto driver sang */
    int high = OUT_OPEN_DRAIN ? !active : active;
    GPIOB->BSRR = high ? (1u << 6) : (1u << (6 + 16));
}

static inline void dir_set(int cw)
{
    int high = OUT_OPEN_DRAIN ? !cw : cw;
    GPIOA->BSRR = high ? (1u << 8) : (1u << (8 + 16));
}

void TIM4_IRQHandler(void)
{
    if (TIM4->SR & TIM_SR_UIF) {
        TIM4->SR = ~TIM_SR_UIF;
        if (!running) return;
        pulHigh = !pulHigh;
        if (pulHigh) { pul_set(1); stepAbs += stepDir; }   /* canh len = 1 buoc */
        else         { pul_set(0); }
        TIM4->ARR = nextArr;                               /* ARPE = 0: co hieu luc ngay chu ky nay (CNT con nho) */
    }
}

/* ---- moc 0 do (TIM2 CH2 input capture) ---- */
static volatile uint32_t lastIdxUs = 0, lockUs = 2000000UL, revUs = 0, idxUs = 0;
static volatile int32_t  lastIdxSteps = 0, idxSteps = 0, revSteps = 0;
static volatile uint8_t  haveLast = 0, newIdx = 0, haveRev = 0;

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_CC2IF) {
        uint32_t t = TIM2->CCR2;                     /* doc CCR2 xoa co CC2IF */
        if (TIM2->SR & TIM_SR_CC2OF) TIM2->SR = ~TIM_SR_CC2OF;
        if (haveLast && (uint32_t)(t - lastIdxUs) < lockUs) return;   /* bo canh rung o mep la chan */
        int32_t s = stepAbs;                         /* TIM2 uu tien cao hon TIM4 nen khong bi chen giua chung */
        haveRev = haveLast;
        if (haveRev) { revSteps = s - lastIdxSteps; revUs = t - lastIdxUs; }
        idxSteps = s; idxUs = t; lastIdxSteps = s; lastIdxUs = t; haveLast = 1; newIdx = 1;
    }
}

/* ---- toc do ---- */
static float target = 0, current = 0;
static uint8_t dirCW = 1;
static uint32_t tLast = 0;

static void apply_speed(float rps)
{
    float f = rps * STEPS_PER_REV;                   /* xung/s */
    if (f < 16.0f) {                                 /* duoi muc 16-bit tao duoc -> dung, giong UNO */
        running = 0; pulHigh = 0; pul_set(0);
        return;
    }
    uint32_t arr = (uint32_t)(TIM4_TICK_HZ / 2 / f) - 1;   /* nua chu ky */
    if (arr > 65535) arr = 65535;
    if (arr < 20) arr = 20;
    __disable_irq();
    nextArr = (uint16_t)arr;
    if (!running) { TIM4->ARR = arr; TIM4->CNT = 0; running = 1; }
    uint32_t lk = (uint32_t)(0.5e6f / rps);          /* khoa chong rung moc: nua vong */
    lockUs = lk > 2000000UL ? 2000000UL : lk;
    __enable_irq();
}

/* ---- UART ---- */
static void out(const char *s)
{
    HAL_UART_Transmit(uart, (uint8_t *)s, (uint16_t)strlen(s), 200);
}

static char rxLine[48];
static uint8_t rxLen = 0;

/* tra ve 1 khi co dong lenh moi trong rxLine (doc USART2 kieu hoi vong, khong can ngat) */
static int poll_rx(void)
{
    USART_TypeDef *u = uart->Instance;
    while (u->SR & (USART_SR_RXNE | USART_SR_ORE)) {
        char c = (char)(u->DR & 0xFF);               /* doc SR roi DR: xoa ca co ORE */
        if (c == '\n' || c == '\r') {
            if (rxLen) { rxLine[rxLen] = 0; rxLen = 0; return 1; }
        } else if (rxLen < sizeof(rxLine) - 1) {
            rxLine[rxLen++] = c;
        }
    }
    return 0;
}

/* so nguyen chia 100 -> "-1.23" (khong dung printf so thuc: newlib-nano mac dinh khong ho tro) */
static const char *fmt_c(int32_t v, char *buf)
{
    int32_t a = v < 0 ? -v : v;
    sprintf(buf, "%s%ld.%02ld", v < 0 ? "-" : "", (long)(a / 100), (long)(a % 100));
    return buf;
}

/* ---- khoi tao ---- */
static uint32_t apb1_timer_clk(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    return ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1) ? pclk1 : 2 * pclk1;
}

void lidar_init(UART_HandleTypeDef *huart)
{
    uart = huart;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM4EN;
    (void)RCC->APB1ENR;

    /* PB6 PUL, PA8 DIR: output, toc do trung binh */
    pul_set(0); dir_set(dirCW);
    GPIOB->MODER   = (GPIOB->MODER & ~(3u << 12)) | (1u << 12);
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(3u << 12)) | (1u << 12);
    GPIOA->MODER   = (GPIOA->MODER & ~(3u << 16)) | (1u << 16);
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3u << 16)) | (1u << 16);
    if (OUT_OPEN_DRAIN) { GPIOB->OTYPER |= 1u << 6;    GPIOA->OTYPER |= 1u << 8; }
    else                { GPIOB->OTYPER &= ~(1u << 6); GPIOA->OTYPER &= ~(1u << 8); }

    /* PB3: AF1 = TIM2_CH2, keo len noi (thay SWO cua cau hinh mac dinh) */
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0xFu << 12)) | (1u << 12);
    GPIOB->PUPDR  = (GPIOB->PUPDR & ~(3u << 6)) | (1u << 6);
    GPIOB->MODER  = (GPIOB->MODER & ~(3u << 6)) | (2u << 6);

    uint32_t tclk = apb1_timer_clk();               /* 84 MHz voi cau hinh mac dinh cua board */

    /* TIM4: dem len, 2 MHz, ngat update moi nua chu ky xung */
    TIM4->CR1 = 0;
    TIM4->PSC = tclk / TIM4_TICK_HZ - 1;
    TIM4->ARR = 60000;
    TIM4->EGR = TIM_EGR_UG;                          /* nap PSC */
    TIM4->SR = 0;
    TIM4->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIM4_IRQn, 1);
    NVIC_EnableIRQ(TIM4_IRQn);
    TIM4->CR1 = TIM_CR1_CEN;

    /* TIM2 (32 bit): 1 MHz chay tu do, CH2 input capture tu TI2 (PB3) */
    TIM2->CR1 = 0;
    TIM2->PSC = tclk / TIM2_TICK_HZ - 1;
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->CCMR1 = (1u << TIM_CCMR1_CC2S_Pos)         /* CC2 = input, IC2 -> TI2 */
                | (3u << TIM_CCMR1_IC2F_Pos);        /* loc: fCK_INT, N = 8 (~0,1 us) */
    TIM2->CCER = TIM_CCER_CC2E | (IDX_RISING ? 0 : TIM_CCER_CC2P);
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
    TIM2->DIER = TIM_DIER_CC2IE;
    NVIC_SetPriority(TIM2_IRQn, 0);                  /* cao hon TIM4: chot so buoc khong bi xen */
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 = TIM_CR1_CEN;

    char b[96];
    sprintf(b, "nucleo_moc0 | lenh: r <vong/s>, r 0, d, z, ? | TIM clk %lu Hz | %s\r\n",
            (unsigned long)tclk, OUT_OPEN_DRAIN ? "open-drain" : "qua NPN");
    out(b);
    tLast = HAL_GetTick();
}

/* ---- vong lap chinh ---- */
static uint8_t haveRef = 0;
static int32_t refSteps = 0, devMin = 0, devMax = 0, revCount = 0;

void lidar_loop(void)
{
    char b[200], c1[16], c2[16];

    if (poll_rx()) {
        char *s = rxLine;
        while (*s == ' ') s++;
        if (s[0] == 'r') {
            float v = strtof(s + 1, NULL);
            target = v < 0 ? 0 : (v > MAX_RPS ? MAX_RPS : v);
            sprintf(b, "Dat toc do %s vong/s\r\n", fmt_c((int32_t)(target * 100 + 0.5f), c1)); out(b);
        } else if (s[0] == 'd' && s[1] == 0) {
            if (current == 0) {
                dirCW = !dirCW; dir_set(dirCW);
                __disable_irq(); stepDir = dirCW ? 1 : -1; __enable_irq();
                haveRef = 0;
                out(dirCW ? "Chieu 1\r\n" : "Chieu 2\r\n");
            } else out("Dung lai truoc (r 0)\r\n");
        } else if (s[0] == 'z' && s[1] == 0) {
            haveRef = 0; revCount = 0; out("Da xoa thong ke\r\n");
        } else if (s[0] == '?' && s[1] == 0) {
            sprintf(b, "toc do %s vong/s | buoc tuyet doi %ld\r\n", fmt_c((int32_t)(current * 100 + 0.5f), c1), (long)stepAbs); out(b);
        }
    }

    /* B1 (PC13, muc thap khi nhan): giam toc ve 0 */
    if (!(GPIOC->IDR & (1u << 13))) target = 0;

    /* tang/giam toc moi 20 ms */
    uint32_t now = HAL_GetTick();
    if (now - tLast >= 20) {
        float dt = (now - tLast) / 1000.0f; tLast = now;
        float prev = current, st = ACCEL_RPS2 * dt;
        if (current < target)      current = (current + st > target) ? target : current + st;
        else if (current > target) current = (current - st < target) ? target : current - st;
        if (current != prev) apply_speed(current);
    }

    /* moi lan qua moc 0 do */
    if (newIdx) {
        __disable_irq();
        int32_t s = idxSteps, rs = revSteps; uint32_t ru = revUs, tu = idxUs; uint8_t hr = haveRev; newIdx = 0;
        __enable_irq();
        GPIOA->ODR ^= 1u << 5;                       /* LD2 */
        uint8_t steady = (current == target) && current > 0;
        if (steady && !haveRef) { haveRef = 1; refSteps = s; devMin = devMax = 0; revCount = 0; }
        if (haveRef) {
            int32_t dev = (s - refSteps) % STEPS_PER_REV;
            if (dev > STEPS_PER_REV / 2) dev -= STEPS_PER_REV;
            if (dev < -STEPS_PER_REV / 2) dev += STEPS_PER_REV;
            if (steady) { if (dev < devMin) devMin = dev; if (dev > devMax) devMax = dev; revCount++; }
            int n = sprintf(b, "MOC 0 t=%lu #%ld", (unsigned long)tu, (long)revCount);
            if (hr) n += sprintf(b + n, " | buoc/vong %ld | %lu.%lu ms", (long)rs, (unsigned long)(ru / 1000), (unsigned long)(ru / 100 % 10));
            n += sprintf(b + n, " | lech %ld buoc (%s do)", (long)dev, fmt_c(dev * 36000 / STEPS_PER_REV, c1));
            sprintf(b + n, " | min..max %ld..%ld buoc = %s do\r\n", (long)devMin, (long)devMax,
                    fmt_c((devMax - devMin) * 36000 / STEPS_PER_REV, c2));
        } else {
            sprintf(b, "MOC 0 t=%lu (dang tang/giam toc, chua thong ke)\r\n", (unsigned long)tu);
        }
        out(b);
    }
}
