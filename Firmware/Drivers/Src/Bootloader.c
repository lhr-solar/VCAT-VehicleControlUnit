#include "Bootloader.h"
#include <string.h>

#define STM32G4_SYSTEM_MEMORY_BASE 0x1FFF0000UL

typedef void (*BootloaderEntry_t)(void);

void Bootloader_JumpToSystemMemory(void) {
    uint32_t bootloader_stack = *(volatile uint32_t *)STM32G4_SYSTEM_MEMORY_BASE;
    uint32_t bootloader_reset = *(volatile uint32_t *)(STM32G4_SYSTEM_MEMORY_BASE + 4U);
    BootloaderEntry_t bootloader_entry = (BootloaderEntry_t)bootloader_reset;

    __disable_irq();
    HAL_DeInit();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    for (uint32_t irq = 0; irq < 8U; irq++) {
        NVIC->ICER[irq] = 0xFFFFFFFFUL;
        NVIC->ICPR[irq] = 0xFFFFFFFFUL;
    }

    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
    __set_MSP(bootloader_stack);
    bootloader_entry();

    while (1) {
    }
}

void Bootloader_CheckForCommand(UART_HandleTypeDef *huart) {
    uint8_t rx = 0;
    uint8_t idx = 0;
    const uint8_t command_len = (uint8_t)strlen(BOOTLOADER_COMMAND);
    const uint32_t deadline = HAL_GetTick() + BOOTLOADER_LISTEN_TIMEOUT_MS;

    while ((int32_t)(deadline - HAL_GetTick()) > 0) {
        if (HAL_UART_Receive(huart, &rx, 1, 10) != HAL_OK) {
            continue;
        }

        if (rx == (uint8_t)BOOTLOADER_COMMAND[idx]) {
            idx++;
            if (idx == command_len) {
                HAL_UART_Transmit(huart, (uint8_t *)BOOTLOADER_ACK, strlen(BOOTLOADER_ACK), 100);
                HAL_Delay(50);
                Bootloader_JumpToSystemMemory();
            }
        } else {
            idx = (rx == (uint8_t)BOOTLOADER_COMMAND[0]) ? 1 : 0;
        }
    }
}
