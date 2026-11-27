/*
 * lidar_fw.h - Nucleo-F411RE: xung buoc + moc 0 do + doc TF03, gan so buoc vao tung khung, goi nhi phan len PC.
 * CODE THU NGHIEM. Xem README.md cung thu muc.
 */
#ifndef LIDAR_FW_H
#define LIDAR_FW_H

#include "main.h"

#define LIDAR_VCP_BAUD 460800UL       /* USART2 = ST-Link VCP. 16 byte x 1000 khung/s = 16 kB/s, can >= ~200 kbaud */

void lidar_init(UART_HandleTypeDef *huart);   /* goi 1 lan, sau MX_USART2_UART_Init() */
void lidar_loop(void);                        /* goi lien tuc trong while (1) */

#endif
