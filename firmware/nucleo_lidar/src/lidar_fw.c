/*
 * lidar_fw.c - Nucleo-F411RE: phat xung buoc, bat moc 0 do, doc TF03 va gan SO BUOC KE TU MOC 0 DO vao tung khung.
 * Goc = so buoc x 360 / 3200 (CLAUDE.md muc 3), khong con ghep goc theo thoi gian tren PC.
 *
 * CODE THU NGHIEM. Trang thai: xem README.md (bien dich / phan cung).
 *
 * Chan:
 *   D10 = PB6  PUL (GPIO, dao muc trong ngat TIM4)       D7 = PA8 DIR
 *   D3  = PB3  moc 0 do (TIM2_CH2 input capture, AF1)     (PB3 mac dinh la SWO: khong bat SWO trace)
 *   D2  = PA10 USART1_RX <- TF03 TX (day nau)              D8 = PA9 USART1_TX -> TF03 RX (day xanh duong)
 *   USART2 (PA2/PA3) = ST-Link VCP len PC, LIDAR_VCP_BAUD
 *   LD2 = PA5 dao moi lan qua moc;  B1 = PC13 nhan de giam toc ve 0
 *
 * Noi driver: nhu nucleo_moc0. Da chay that 06/11 kieu noi thang PUL+ <- PB6, PUL- -> GND (chan cao = opto sang).
 *
 * Lenh (Newline): r <vong/s> | d | z | ? | b 1 (bat goi nhi phan) | b 0 (ve che do chu) | u <baud> (baud USART1 toi TF03)
 *   | t (tu kiem tra USART1: noi tam D8 sang D2) | f <Hz> (dat va luu tan so TF03 qua D8)
 *
 * Goi nhi phan, 16 byte, so nguyen little-endian, CRC-8 (da thuc 0x07, khoi dau 0) tren byte 0..14:
 *   Khung TF03: AA 55 | seq u8 | t_us u32 | step i16 | dist_cm u16 | str u16 | rev u8 | flags u8 | crc
 *     t_us, step: chot luc nhan byte 0x59 dau khung (truoc khi het 9 byte, giam tre 0,7 ms o 115200)
 *     step: so buoc ke tu moc 0 do gan nhat (co dau theo chieu quay); rev: so thu tu moc (mod 256)
 *     flags: bit0 da co moc, bit1 dang quay deu (dat toc do dat), bit2 chieu 1, bit3 qua lau khong co moc (step bi kep)
 *   Moc 0 do:   AA 56 | seq u8 | t_us u32 | rev_steps i16 | n_khung u16 | tf_loi u16 | rev u8 | flags u8 | crc
 *     rev_steps: so buoc cua vong vua xong (phai = 3200); n_khung: so khung TF03 nhan duoc trong vong vua xong
 *     tf_loi: tong so khung TF03 sai checksum + tran bo dem (mod 65536)
 *   seq chung cho ca hai loai goi, tang 1 moi goi: PC dem goi mat.
 */
#include "lidar_fw.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OUT_OPEN_DRAIN   0          /* 0: chan day-keo, muc cao = opto sang (noi thang hoac qua NPN). 1: open-drain, chi khi chan la FT */
#define IDX_RISING       1
#define STEPS_PER_REV    3200L
#define ACCEL_RPS2       1.0f
#define MAX_RPS          5.0f
#define TIM4_TICK_HZ     2000000UL
#define TIM2_TICK_HZ     1000000UL
#define TF_BAUD_DEFAULT  460800UL   /* TF03 dang dung (27/11, do that): 460800 baud, 2000 Hz. Doi bang lenh u <baud> */

static UART_HandleTypeDef *uart;

/* =============================== xung buoc (TIM4) =============================== */
static volatile int32_t  stepAbs = 0;
static volatile int8_t   stepDir = 1;
static volatile uint8_t  pulHigh = 0, running = 0;
static volatile uint16_t nextArr = 60000;

static inline void pul_set(int active)
{
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
        if (pulHigh) { pul_set(1); stepAbs += stepDir; }
        else         { pul_set(0); }
        TIM4->ARR = nextArr;
    }
}

/* =============================== moc 0 do (TIM2 CH2) =============================== */
static volatile uint32_t lastIdxUs = 0, lockUs = 2000000UL, revUs = 0, idxUs = 0;
static volatile int32_t  lastIdxSteps = 0, idxSteps = 0, revSteps = 0;
static volatile uint8_t  haveLast = 0, newIdx = 0, haveRev = 0;
static volatile uint32_t idxCount = 0;                 /* so lan qua moc (rev) */
static volatile uint16_t framesThisRev = 0, framesLastRev = 0;

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_CC2IF) {
        uint32_t t = TIM2->CCR2;
        if (TIM2->SR & TIM_SR_CC2OF) TIM2->SR = ~TIM_SR_CC2OF;
        if (haveLast && (uint32_t)(t - lastIdxUs) < lockUs) return;
        int32_t s = stepAbs;
        haveRev = haveLast;
        if (haveRev) { revSteps = s - lastIdxSteps; revUs = t - lastIdxUs; }
        idxSteps = s; idxUs = t; lastIdxSteps = s; lastIdxUs = t; haveLast = 1;
        idxCount++;
        framesLastRev = framesThisRev; framesThisRev = 0;
        newIdx = 1;
    }
}

/* =============================== TF03 (USART1 RX, ngat moi byte) =============================== */
typedef struct { uint32_t t; int16_t step; uint16_t dist, str; uint8_t rev, flags; } frame_t;
#define FQ 256u                                         /* luy thua 2 */
static frame_t fq[FQ];
static volatile uint32_t fqHead = 0, fqTail = 0;
static volatile uint32_t tfFrames = 0, tfBad = 0, tfOverrun = 0, fqDrop = 0;
static volatile uint32_t rxBytes = 0, rxFe = 0, rx59 = 0;   /* chan doan day TF03: so byte, loi khung UART (sai baud), so byte 0x59 */
static volatile uint64_t rxLast5 = 0;                         /* 5 byte nhan gan nhat (bat phan hoi lenh TF03) */
static volatile uint32_t saveAck = 0;                         /* so lan TF03 xac nhan luu cau hinh */
static volatile uint8_t  steadyFlag = 0, dirFlag = 1;

static uint8_t  fb[9], fi = 0;
static uint32_t f_t; static int16_t f_step; static uint8_t f_rev, f_flags;

static inline void frame_snapshot(void)
{
    f_t = TIM2->CNT;
    uint8_t fl = 0;
    if (haveLast) {
        int32_t s = stepAbs - lastIdxSteps;            /* TIM2 uu tien cao hon co the xen giua: lech toi da 1 buoc */
        if (s > 32767) { s = 32767; fl |= 8; }
        if (s < -32768) { s = -32768; fl |= 8; }
        f_step = (int16_t)s; fl |= 1;
    } else {
        f_step = 0;
    }
    if (steadyFlag) fl |= 2;
    if (dirFlag) fl |= 4;
    f_rev = (uint8_t)idxCount; f_flags = fl;
}

void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;
    if (!(sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE))) return;
    uint8_t b = (uint8_t)USART1->DR;                    /* doc SR roi DR: xoa ca ORE/FE/NE */
    if (sr & USART_SR_ORE) tfOverrun++;
    rxBytes++;
    if (sr & USART_SR_FE) rxFe++;
    if (b == 0x59) rx59++;
    rxLast5 = (rxLast5 << 8) | b;                      /* phan hoi lenh luu cua TF03: 5A 05 11 00 70 */
    if ((rxLast5 & 0xFFFFFFFFFFull) == 0x5A05110070ull) saveAck++;
    if (fi == 0) {
        if (b == 0x59) { fb[0] = b; fi = 1; frame_snapshot(); }
    } else if (fi == 1) {
        if (b == 0x59) { fb[1] = b; fi = 2; } else fi = 0;
    } else {
        fb[fi++] = b;
        if (fi == 9) {
            fi = 0;
            uint8_t sum = 0;
            for (int k = 0; k < 8; k++) sum += fb[k];
            if (sum != fb[8]) { tfBad++; return; }
            tfFrames++; framesThisRev++;
            uint32_t h = fqHead;
            if (h - fqTail >= FQ) { fqDrop++; return; }
            frame_t *f = &fq[h & (FQ - 1)];
            f->t = f_t; f->step = f_step; f->rev = f_rev; f->flags = f_flags;
            f->dist = (uint16_t)(fb[2] | (fb[3] << 8)); f->str = (uint16_t)(fb[4] | (fb[5] << 8));
            fqHead = h + 1;
        }
    }
}

static uint32_t tfBaud = 0;

/* gui lenh toi TF03 (hoi vong, chi dung luc cau hinh) */
static void tf_send(const uint8_t *p, int n)
{
    for (int k = 0; k < n; k++) {
        while (!(USART1->SR & USART_SR_TXE)) {}
        USART1->DR = p[k];
    }
    while (!(USART1->SR & USART_SR_TC)) {}
}

static void tf_set_baud(uint32_t baud)
{
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    tfBaud = baud;
    USART1->CR1 &= ~USART_CR1_UE;
    USART1->BRR = (pclk2 + baud / 2) / baud;            /* OVER8 = 0: BRR = fck / baud (lam tron) */
    USART1->CR1 |= USART_CR1_UE;
}

/* =============================== USART2 len PC: phat bang ngat =============================== */
#define TQ 4096u
static uint8_t tq[TQ];
static volatile uint32_t tqHead = 0, tqTail = 0, txDrop = 0;

void USART2_IRQHandler(void)
{
    if ((USART2->CR1 & USART_CR1_TXEIE) && (USART2->SR & USART_SR_TXE)) {
        uint32_t t = tqTail;
        if (t != tqHead) { USART2->DR = tq[t & (TQ - 1)]; tqTail = t + 1; }
        else USART2->CR1 &= ~USART_CR1_TXEIE;
    }
}

/* ca goi vao hang doi hoac bo ca goi (khong cat doi goi) */
static int tx_put(const void *p, uint32_t n)
{
    uint32_t h = tqHead;
    if (TQ - (h - tqTail) < n) { txDrop++; return 0; }
    const uint8_t *s = p;
    for (uint32_t k = 0; k < n; k++) tq[(h + k) & (TQ - 1)] = s[k];
    tqHead = h + n;
    USART2->CR1 |= USART_CR1_TXEIE;
    return 1;
}

static void out(const char *s) { tx_put(s, (uint32_t)strlen(s)); }

static uint8_t crc8(const uint8_t *p, int n)
{
    uint8_t c = 0;
    while (n--) {
        c ^= *p++;
        for (int k = 0; k < 8; k++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
    return c;
}

static uint8_t seq = 0;

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, (uint16_t)v); put16(p + 2, (uint16_t)(v >> 16)); }

static void send_frame(const frame_t *f)
{
    uint8_t p[16];
    p[0] = 0xAA; p[1] = 0x55; p[2] = seq;
    put32(p + 3, f->t); put16(p + 7, (uint16_t)f->step); put16(p + 9, f->dist); put16(p + 11, f->str);
    p[13] = f->rev; p[14] = f->flags; p[15] = crc8(p, 15);
    if (tx_put(p, 16)) seq++;
}

static void send_index(uint32_t t, int32_t rs, uint16_t nfr, uint8_t rev, uint8_t flags)
{
    uint8_t p[16];
    p[0] = 0xAA; p[1] = 0x56; p[2] = seq;
    put32(p + 3, t); put16(p + 7, (uint16_t)(int16_t)rs); put16(p + 9, nfr);
    put16(p + 11, (uint16_t)(tfBad + tfOverrun + fqDrop));
    p[13] = rev; p[14] = flags; p[15] = crc8(p, 15);
    if (tx_put(p, 16)) seq++;
}

/* =============================== toc do =============================== */
static float target = 0, current = 0;
static uint8_t dirCW = 1, binMode = 0;
static uint32_t tLast = 0;

static void apply_speed(float rps)
{
    float f = rps * STEPS_PER_REV;
    if (f < 16.0f) { running = 0; pulHigh = 0; pul_set(0); return; }
    uint32_t arr = (uint32_t)(TIM4_TICK_HZ / 2 / f + 0.5f) - 1;   /* lam tron: sai so toc do <= 0,16 % */
    if (arr > 65535) arr = 65535;
    if (arr < 20) arr = 20;
    __disable_irq();
    nextArr = (uint16_t)arr;
    if (!running) { TIM4->ARR = arr; TIM4->CNT = 0; running = 1; }
    uint32_t lk = (uint32_t)(0.5e6f / rps);
    lockUs = lk > 2000000UL ? 2000000UL : lk;
    __enable_irq();
}

/* =============================== lenh tu PC =============================== */
static char rxLine[48];
static uint8_t rxLen = 0;

static int poll_rx(void)
{
    USART_TypeDef *u = uart->Instance;
    while (u->SR & (USART_SR_RXNE | USART_SR_ORE)) {
        char c = (char)(u->DR & 0xFF);
        if (c == '\n' || c == '\r') {
            if (rxLen) { rxLine[rxLen] = 0; rxLen = 0; return 1; }
        } else if (rxLen < sizeof(rxLine) - 1) {
            rxLine[rxLen++] = c;
        }
    }
    return 0;
}

static const char *fmt_c(int32_t v, char *buf)
{
    int32_t a = v < 0 ? -v : v;
    sprintf(buf, "%s%ld.%02ld", v < 0 ? "-" : "", (long)(a / 100), (long)(a % 100));
    return buf;
}

static uint32_t apb1_timer_clk(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    return ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1) ? pclk1 : 2 * pclk1;
}

/* =============================== khoi tao =============================== */
void lidar_init(UART_HandleTypeDef *huart)
{
    uart = huart;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM4EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;

    pul_set(0); dir_set(dirCW);
    GPIOB->MODER   = (GPIOB->MODER & ~(3u << 12)) | (1u << 12);           /* PB6 out */
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(3u << 12)) | (1u << 12);
    GPIOA->MODER   = (GPIOA->MODER & ~(3u << 16)) | (1u << 16);           /* PA8 out */
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3u << 16)) | (1u << 16);
    if (OUT_OPEN_DRAIN) { GPIOB->OTYPER |= 1u << 6;    GPIOA->OTYPER |= 1u << 8; }
    else                { GPIOB->OTYPER &= ~(1u << 6); GPIOA->OTYPER &= ~(1u << 8); }

    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0xFu << 12)) | (1u << 12);         /* PB3 AF1 TIM2_CH2, keo len */
    GPIOB->PUPDR  = (GPIOB->PUPDR & ~(3u << 6)) | (1u << 6);
    GPIOB->MODER  = (GPIOB->MODER & ~(3u << 6)) | (2u << 6);

    /* PA9 USART1_TX, PA10 USART1_RX: AF7; RX keo len de khong nhan rac khi chua cam TF03 */
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~((0xFu << 4) | (0xFu << 8))) | (7u << 4) | (7u << 8);
    GPIOA->PUPDR  = (GPIOA->PUPDR & ~(3u << 20)) | (1u << 20);
    GPIOA->MODER  = (GPIOA->MODER & ~((3u << 18) | (3u << 20))) | (2u << 18) | (2u << 20);

    uint32_t tclk = apb1_timer_clk();

    TIM4->CR1 = 0;
    TIM4->PSC = tclk / TIM4_TICK_HZ - 1;
    TIM4->ARR = 60000;
    TIM4->EGR = TIM_EGR_UG;
    TIM4->SR = 0;
    TIM4->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIM4_IRQn, 1);
    NVIC_EnableIRQ(TIM4_IRQn);
    TIM4->CR1 = TIM_CR1_CEN;

    TIM2->CR1 = 0;
    TIM2->PSC = tclk / TIM2_TICK_HZ - 1;
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->CCMR1 = (1u << TIM_CCMR1_CC2S_Pos) | (3u << TIM_CCMR1_IC2F_Pos);
    TIM2->CCER = TIM_CCER_CC2E | (IDX_RISING ? 0 : TIM_CCER_CC2P);
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
    TIM2->DIER = TIM_DIER_CC2IE;
    NVIC_SetPriority(TIM2_IRQn, 0);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 = TIM_CR1_CEN;

    /* USART1: 8N1, nhan (+ phat de doi cau hinh TF03 sau nay), ngat moi byte */
    USART1->CR1 = 0; USART1->CR2 = 0; USART1->CR3 = 0;
    tf_set_baud(TF_BAUD_DEFAULT);
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    NVIC_SetPriority(USART1_IRQn, 2);
    NVIC_EnableIRQ(USART1_IRQn);

    /* USART2 (HAL da khoi tao): them ngat phat, nhan van hoi vong */
    NVIC_SetPriority(USART2_IRQn, 3);
    NVIC_EnableIRQ(USART2_IRQn);

    char b[200];
    sprintf(b, "nucleo_lidar | lenh: r <vong/s>, r 0, d, z, ?, b 1, b 0, u <baud>, p, f <Hz>, B <baud>, t | VCP %lu | TF03 %lu | TIM clk %lu Hz\r\n",
            (unsigned long)LIDAR_VCP_BAUD, (unsigned long)TF_BAUD_DEFAULT, (unsigned long)tclk);
    out(b);
    tLast = HAL_GetTick();
}

/* =============================== vong lap chinh =============================== */
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
            if (!binMode) { sprintf(b, "Dat toc do %s vong/s\r\n", fmt_c((int32_t)(target * 100 + 0.5f), c1)); out(b); }
        } else if (s[0] == 'd' && s[1] == 0) {
            if (current == 0) {
                dirCW = !dirCW; dir_set(dirCW);
                __disable_irq(); stepDir = dirCW ? 1 : -1; dirFlag = dirCW; __enable_irq();
                haveRef = 0;
                out(dirCW ? "Chieu 1\r\n" : "Chieu 2\r\n");
            } else out("Dung lai truoc (r 0)\r\n");
        } else if (s[0] == 'z' && s[1] == 0) {
            haveRef = 0; revCount = 0; out("Da xoa thong ke\r\n");
        } else if (s[0] == 'b') {
            binMode = (uint8_t)(atoi(s + 1) != 0);
            if (!binMode) out("Che do chu\r\n");
        } else if (s[0] == 'f') {
            /* dat va luu tan so TF03 qua USART1 TX (D8). Chi dung khi app khong chay: vong lap dung ~1,5 s */
            long hz = atol(s + 1);
            if (!(hz == 100 || hz == 200 || hz == 500 || hz == 1000 || hz == 2000 || hz == 5000)) {
                out("Chi cho phep 100, 200, 500, 1000, 2000, 5000 Hz\r\n");
            } else if ((uint32_t)hz * 112u > tfBaud) {
                sprintf(b, "%ld Hz can baud TF03 >= %lu, dang la %lu: TF03 se qua tai, khong dat\r\n", hz, (unsigned long)hz * 112u, (unsigned long)tfBaud);
                out(b);
            } else {
                uint8_t cmd[6] = {0x5A, 0x06, 0x03, (uint8_t)hz, (uint8_t)(hz >> 8), 0};
                for (int k = 0; k < 5; k++) cmd[5] += cmd[k];
                tf_send(cmd, 6);
                HAL_Delay(100);
                uint32_t a0 = saveAck;
                static const uint8_t sv[4] = {0x5A, 0x04, 0x11, 0x6F};
                tf_send(sv, 4);
                uint32_t t0 = HAL_GetTick();
                while (saveAck == a0 && HAL_GetTick() - t0 < 300) {}
                int ok = saveAck != a0;
                HAL_Delay(200);
                uint32_t f0 = tfFrames;
                HAL_Delay(1000);
                sprintf(b, "TF03: dat %ld Hz, lenh luu %s, do lai %lu khung/s\r\n", hz,
                        ok ? "da duoc xac nhan" : "KHONG duoc xac nhan", (unsigned long)(tfFrames - f0));
                out(b);
            }
        } else if (s[0] == 'p' && s[1] == 0) {
            /* do baud TF03: thu baud hien tai truoc, roi cac baud TF03 ho tro. Vong lap dung toi ~2,5 s */
            static const uint32_t bl[4] = {115200, 230400, 460800, 921600};
            uint32_t cand[5], nc = 0, found = 0;
            cand[nc++] = tfBaud;
            for (int k = 0; k < 4; k++) if (bl[k] != tfBaud) cand[nc++] = bl[k];
            for (uint32_t k = 0; k < nc && !found; k++) {
                tf_set_baud(cand[k]);
                HAL_Delay(20);
                uint32_t f0 = tfFrames, e0 = tfBad;
                HAL_Delay(300);
                uint32_t nf = tfFrames - f0, ne = tfBad - e0;
                if (nf >= 10 && nf > 10 * ne) found = cand[k];   /* sai baud chi con vai khung trung checksum ngau nhien */
            }
            if (found) {
                uint32_t f0 = tfFrames;
                HAL_Delay(1000);
                sprintf(b, "TF03 DO baud %lu khung/s %lu\r\n", (unsigned long)found, (unsigned long)(tfFrames - f0));
            } else {
                tf_set_baud(cand[0]);
                sprintf(b, "TF03 DO khong thay (giu baud %lu)\r\n", (unsigned long)cand[0]);
            }
            out(b);
        } else if (s[0] == 'B') {
            /* doi va luu baud TF03; chi co hieu luc sau khi rut nguon TF03 */
            long nb = atol(s + 1);
            uint32_t f0 = tfFrames;
            HAL_Delay(500);
            uint32_t hz2 = (tfFrames - f0) * 2;
            if (!(nb == 115200 || nb == 230400 || nb == 460800 || nb == 921600)) {
                out("TF03 BAUD loi: chi cho phep 115200, 230400, 460800, 921600\r\n");
            } else if (hz2 == 0) {
                out("TF03 BAUD loi: khong nhan duoc khung TF03, hay do baud (p) truoc\r\n");
            } else if ((float)hz2 * 112.0f > (float)nb * 1.02f) {
                sprintf(b, "TF03 BAUD loi: TF03 dang chay ~%lu Hz, %ld baud khong du. Giam tan so truoc\r\n", (unsigned long)hz2, nb);
                out(b);
            } else {
                uint8_t cmd[8] = {0x5A, 0x08, 0x06, (uint8_t)nb, (uint8_t)(nb >> 8), (uint8_t)(nb >> 16), (uint8_t)(nb >> 24), 0};
                for (int k = 0; k < 7; k++) cmd[7] += cmd[k];
                tf_send(cmd, 8);
                HAL_Delay(200);
                uint32_t a0 = saveAck;
                static const uint8_t sv[4] = {0x5A, 0x04, 0x11, 0x6F};
                tf_send(sv, 4);
                uint32_t t0 = HAL_GetTick();
                while (saveAck == a0 && HAL_GetTick() - t0 < 300) {}
                sprintf(b, "TF03 BAUD %ld, lenh luu %s. Rut nguon TF03 roi cam lai\r\n", nb,
                        saveAck != a0 ? "da duoc xac nhan" : "KHONG duoc xac nhan");
                out(b);
            }
        } else if (s[0] == 't' && s[1] == 0) {
            /* tu kiem tra USART1: noi tam D8 (PA9, TX) sang D2 (PA10, RX), phat 100 khung kieu TF03 (dist 123 cm) */
            uint32_t b0 = rxBytes, f0 = tfFrames;
            for (int n = 0; n < 100; n++) {
                uint8_t fr[9] = {0x59, 0x59, 123, 0, 0x2C, 0x01, 0, 0, 0};
                for (int k = 0; k < 8; k++) fr[8] += fr[k];
                for (int k = 0; k < 9; k++) {
                    while (!(USART1->SR & USART_SR_TXE)) {}
                    USART1->DR = fr[k];
                }
            }
            while (!(USART1->SR & USART_SR_TC)) {}
            HAL_Delay(5);
            sprintf(b, "Tu kiem tra D8 -> D2: phat 900 byte / 100 khung, nhan %lu byte / %lu khung (%s)\r\n",
                    (unsigned long)(rxBytes - b0), (unsigned long)(tfFrames - f0),
                    (tfFrames - f0) == 100 ? "USART1 + chan D2 tot" : "chua noi D8-D2, hoac D2/USART1 loi");
            out(b);
        } else if (s[0] == 'u') {
            long baud = atol(s + 1);
            if (baud >= 9600 && baud <= 921600) {
                tf_set_baud((uint32_t)baud);
                sprintf(b, "USART1 (TF03) = %ld baud\r\n", baud); out(b);
            }
        } else if (s[0] == '?' && s[1] == 0) {
            sprintf(b, "toc do %s vong/s | buoc tuyet doi %ld | TF03 khung %lu, loi checksum %lu, tran %lu | bo goi: hang khung %lu, hang phat %lu\r\n",
                    fmt_c((int32_t)(current * 100 + 0.5f), c1), (long)stepAbs, (unsigned long)tfFrames, (unsigned long)tfBad,
                    (unsigned long)tfOverrun, (unsigned long)fqDrop, (unsigned long)txDrop);
            out(b);
            sprintf(b, "USART1 (D2): byte nhan %lu, loi khung UART %lu, byte 0x59 %lu | PA10 (D2) dang muc %s\r\n",
                    (unsigned long)rxBytes, (unsigned long)rxFe, (unsigned long)rx59, (GPIOA->IDR & (1u << 10)) ? "cao" : "thap");
            out(b);
        }
    }

    if (!(GPIOC->IDR & (1u << 13))) target = 0;          /* B1 */

    uint32_t now = HAL_GetTick();
    if (now - tLast >= 20) {
        float dt = (now - tLast) / 1000.0f; tLast = now;
        float prev = current, st = ACCEL_RPS2 * dt;
        if (current < target)      current = (current + st > target) ? target : current + st;
        else if (current > target) current = (current - st < target) ? target : current - st;
        if (current != prev) apply_speed(current);
        steadyFlag = (current == target) && current > 0;
    }

    /* khung TF03 -> goi nhi phan (che do chu: bo qua, chi dem) */
    while (fqTail != fqHead) {
        frame_t f = fq[fqTail & (FQ - 1)];
        fqTail++;
        if (binMode) send_frame(&f);
    }

    if (newIdx) {
        __disable_irq();
        int32_t s = idxSteps, rs = revSteps; uint32_t ru = revUs, tu = idxUs; uint8_t hr = haveRev;
        uint16_t nfr = framesLastRev; uint8_t rev = (uint8_t)idxCount; newIdx = 0;
        __enable_irq();
        GPIOA->ODR ^= 1u << 5;
        uint8_t steady = (current == target) && current > 0;
        if (binMode) {
            send_index(tu, hr ? rs : 0, nfr, rev, (uint8_t)((hr ? 1 : 0) | (steady ? 2 : 0) | (dirCW ? 4 : 0)));
            return;
        }
        if (steady && !haveRef) { haveRef = 1; refSteps = s; devMin = devMax = 0; revCount = 0; }
        if (haveRef) {
            int32_t dev = (s - refSteps) % STEPS_PER_REV;
            if (dev > STEPS_PER_REV / 2) dev -= STEPS_PER_REV;
            if (dev < -STEPS_PER_REV / 2) dev += STEPS_PER_REV;
            if (steady) { if (dev < devMin) devMin = dev; if (dev > devMax) devMax = dev; revCount++; }
            int n = sprintf(b, "MOC 0 t=%lu #%ld", (unsigned long)tu, (long)revCount);
            if (hr) n += sprintf(b + n, " | buoc/vong %ld | %lu.%lu ms | %u khung", (long)rs, (unsigned long)(ru / 1000),
                                 (unsigned long)(ru / 100 % 10), (unsigned)nfr);
            n += sprintf(b + n, " | lech %ld buoc (%s do)", (long)dev, fmt_c(dev * 36000 / STEPS_PER_REV, c1));
            sprintf(b + n, " | min..max %ld..%ld buoc = %s do\r\n", (long)devMin, (long)devMax,
                    fmt_c((devMax - devMin) * 36000 / STEPS_PER_REV, c2));
        } else {
            sprintf(b, "MOC 0 t=%lu (dang tang/giam toc, chua thong ke) | %u khung\r\n", (unsigned long)tu, (unsigned)nfr);
        }
        out(b);
    }
}
