#pragma once

#include "FaultHandlerTask.h"
#include "PrechargeTask.h"
#include "FSMTask.h"
#include "UpdateVCUInputsTask.h"
#include "VCUStatusTask.h"
#include "CANbus.h"

#define INIT_TASK_STACK_SIZE                configMINIMAL_STACK_SIZE
#define FAULT_HANDLER_TASK_STACK_SIZE       configMINIMAL_STACK_SIZE * 2
#define PRECHARGE_TASK_STACK_SIZE           configMINIMAL_STACK_SIZE * 2
#define FSM_TASK_STACK_SIZE                 configMINIMAL_STACK_SIZE * 2
#define VCU_STATUS_TASK_STACK_SIZE          configMINIMAL_STACK_SIZE * 2
#define UPDATE_VCU_INPUTS_STACK_SIZE        configMINIMAL_STACK_SIZE * 2
#define BOOTLOADER_TASK_STACK_SIZE          configMINIMAL_STACK_SIZE


extern StaticTask_t FaultHandler_Task_Buffer;
extern StackType_t FaultHandler_Task_Stack[FAULT_HANDLER_TASK_STACK_SIZE];

extern StaticTask_t Precharge_Task_Buffer;
extern StackType_t Precharge_Task_Stack[PRECHARGE_TASK_STACK_SIZE];

extern StaticTask_t Init_Task_Buffer;
extern StackType_t Init_Task_Stack[INIT_TASK_STACK_SIZE];

extern StaticTask_t FSM_Task_Buffer;
extern StackType_t FSM_Task_Stack[FSM_TASK_STACK_SIZE];

extern StaticTask_t VCUStatus_Task_Buffer;
extern StackType_t VCUStatus_Task_Stack[VCU_STATUS_TASK_STACK_SIZE];

extern StaticTask_t UpdateVCUInputs_Task_Buffer;
extern StackType_t UpdateVCUInputs_Task_Stack[UPDATE_VCU_INPUTS_STACK_SIZE];

extern StaticTask_t Bootloader_Task_Buffer;
extern StackType_t Bootloader_Task_Stack[BOOTLOADER_TASK_STACK_SIZE];



#define FAULT_HANDLER_THREAD_PRIO           (tskIDLE_PRIORITY + 4)
#define PRECHARGE_THREAD_PRIO               (tskIDLE_PRIORITY + 3)
#define UPDATE_VCU_INPUTS_THREAD_PRIO       (tskIDLE_PRIORITY + 2)
#define FSM_THREAD_PRIO                     (tskIDLE_PRIORITY + 2)
#define UPDATE_CONTROL_STATUS_THREAD_PRIO   (tskIDLE_PRIORITY + 2)
#define VCU_STATUS_THREAD_PRIO              (tskIDLE_PRIORITY + 1)
#define BOOTLOADER_THREAD_PRIO              (tskIDLE_PRIORITY + 1)
#define SDCARD_WORKER_THREAD_PRIO           (tskIDLE_PRIORITY + 1)


// Period the fault thread runs at (once a fault is active)
#define FAULT_LOOP_PERIOD_MS 500

// Period the motor control thread runs at
#define MOTOR_CONTROL_TASK_PERIOD_MS 100

// Period the precharge thread runs at 
#define PRECHARGE_TASK_DELAY_MS 100

#define FSM_TASK_DELAY_MS 100

#define VCU_STATUS_TASK_DELAY_MS 500

#define UPDATE_VCU_INPUTS_TASK_DELAY_MS 50


extern TaskHandle_t precharge_task_handle;



void Task_Init();
