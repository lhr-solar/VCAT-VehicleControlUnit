#include "InitTask.h"
#include "DebugConfig.h"
#include "StatusLEDs.h"
#include "Watchdogs.h"
#include "MotorTelemetryTask.h"
#include "Bootloader.h"

StaticTask_t FaultHandler_Task_Buffer;
StackType_t FaultHandler_Task_Stack[FAULT_HANDLER_TASK_STACK_SIZE];

StaticTask_t Precharge_Task_Buffer;
StackType_t Precharge_Task_Stack[PRECHARGE_TASK_STACK_SIZE];

StaticTask_t Init_Task_Buffer;
StackType_t Init_Task_Stack[INIT_TASK_STACK_SIZE];

StaticTask_t FSM_Task_Buffer;
StackType_t FSM_Task_Stack[FSM_TASK_STACK_SIZE];

StaticTask_t VCUStatus_Task_Buffer;
StackType_t VCUStatus_Task_Stack[VCU_STATUS_TASK_STACK_SIZE];

StaticTask_t UpdateVCUInputs_Task_Buffer;
StackType_t UpdateVCUInputs_Task_Stack[UPDATE_VCU_INPUTS_STACK_SIZE];

TaskHandle_t precharge_task_handle = NULL;

void Task_Init() {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    
    Init_UART_Printf();
    Bootloader_CheckForCommand(husart3);

    // prech
    ADC_Sense_Init();
    contactor_init();
    MotorSafeBits_Init();

    if (MotorCAN_Init() != CAN_OK) {
        printf("MotorCAN_Init FAILED\r\n");
    }
    if (CarCAN_Init() != CAN_OK) {
        printf("CarCAN_Init FAILED\r\n");
    }

#if FSM_DEBUG_BUILD
    // Bench-test: route motor controller telemetry/commands over the CarCAN
    // bus instead of the (possibly unwired) dedicated MotorCAN bus.
    motorfdcan = carfdcan;
#endif

    fsm_init();

    watchdog_init();
    watchdog_start_all();


    // Required for VCU status testing
    faults_init();

    MotorTelemetryTask_Init();

    xTaskCreateStatic(
        Task_FaultHandler,
        "FaultHandler",
        FAULT_HANDLER_TASK_STACK_SIZE,
        NULL,
        FAULT_HANDLER_THREAD_PRIO,
        FaultHandler_Task_Stack,
        &FaultHandler_Task_Buffer
    );

#if !FSM_DEBUG_BUILD
    precharge_task_handle = xTaskCreateStatic(
        Task_Precharge,
        "Precharge",
        PRECHARGE_TASK_STACK_SIZE,
        NULL,
        PRECHARGE_THREAD_PRIO,
        Precharge_Task_Stack,
        &Precharge_Task_Buffer
    );
#endif

    xTaskCreateStatic(
        Task_FSM,
        "FSM Thread",
        FSM_TASK_STACK_SIZE,
        NULL,
        FSM_THREAD_PRIO,
        FSM_Task_Stack,
        &FSM_Task_Buffer
    );

    xTaskCreateStatic(
        Task_BroadcastVCUStatus,
        "VCU Status Thread",
        VCU_STATUS_TASK_STACK_SIZE,
        NULL,
        VCU_STATUS_THREAD_PRIO,
        VCUStatus_Task_Stack,
        &VCUStatus_Task_Buffer
    );

    xTaskCreateStatic(
        Task_UpdateVCUInputs,
        "Update FSM Inputs Thread",
        UPDATE_VCU_INPUTS_STACK_SIZE,
        NULL,
        UPDATE_VCU_INPUTS_THREAD_PRIO,
        UpdateVCUInputs_Task_Stack,
        &UpdateVCUInputs_Task_Buffer
    );


    vTaskDelete(NULL);
}
