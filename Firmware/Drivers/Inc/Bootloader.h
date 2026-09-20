#pragma once

#include "stm32xx_hal.h"

#define BOOTLOADER_COMMAND "$BOOT"
#define BOOTLOADER_ACK "BOOT:ACK\r\n"

void Task_Bootloader(void *args);
void Bootloader_JumpToSystemMemory(void);
