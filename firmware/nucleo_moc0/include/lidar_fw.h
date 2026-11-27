/*
 * lidar_fw.h - Nucleo-F411RE thay Arduino UNO (index_flag_test.ino): phat xung buoc + bat moc 0 do.
 * CODE THU NGHIEM. Xem README.md cung thu muc.
 */
#ifndef LIDAR_FW_H
#define LIDAR_FW_H

#include "main.h"

void lidar_init(UART_HandleTypeDef *huart);   /* goi 1 lan, sau MX_USART2_UART_Init() */
void lidar_loop(void);                        /* goi lien tuc trong while (1) */

#endif
