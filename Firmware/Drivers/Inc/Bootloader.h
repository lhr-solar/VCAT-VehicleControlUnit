#pragma once

#include "stm32xx_hal.h"

#define BOOTLOADER_COMMAND "$BOOT"
#define BOOTLOADER_ACK "BOOT:ACK\r\n"
#define BOOTLOADER_LISTEN_TIMEOUT_MS 750

void Bootloader_CheckForCommand(UART_HandleTypeDef *huart);
void Bootloader_JumpToSystemMemory(void);
