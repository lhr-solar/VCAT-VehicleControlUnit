#include "UpdateVCUInputsTask.h"
#include "InitTask.h"
#include "Contactors.h"
#include "FaultBits.h"
#include "event_groups.h"
#include <string.h>
#include <math.h>

static float brake_threshold = BRAKE_THRESH;

static VCUDataIn_t fsm_input_a = {0};
static VCUDataIn_t fsm_input_b = {0};

VCUDataIn_t * volatile g_data_read = &fsm_input_a;
VCUDataIn_t * volatile g_data_write = &fsm_input_b;

static void rebuild_inputs(void) {
    EventBits_t s = 0;

    if (g_data_read->driver_input.Gear_Forward)
        s |= FORWARD_BIT;
    else if (g_data_read->driver_input.Gear_Reverse)
        s |= REVERSE_BIT;
    else
        s |= NEUTRAL_BIT;

    if (g_data_read->driver_input.Regen_Activate) 
        s |= REGEN_BUTTON_BIT;
    if (g_data_read->driver_input.Regen_Enable) 
        s |= REGEN_ENABLED_BIT;
    if (g_data_read->driver_input.Cruise_Enable) 
        s |= CRUISE_CONTROL_BUTTON_BIT;
    if (g_data_read->bps_status.BPS_Regen_OK) 
        s |= READY_TO_REGEN_BIT;
    if (contactor_get_sense(MOTOR_CONTACTOR) && contactor_get_sense(MOTOR_PRE_CONTACTOR))
        s |= PRECHARGE_COMPLETE_BIT;

    if (g_data_read->brake_pressure2.Brake_Pressure >= brake_threshold) {
        s |= BRAKE_BIT;
        brake_threshold = BRAKE_THRESH_HYST;
    } else {
        brake_threshold = BRAKE_THRESH;
    }

    fsm_set_all_inputs(s);

    if(fsm_is_input_set(REGEN_BUTTON_BIT) && !fsm_is_input_set(READY_TO_REGEN_BIT)) {
        warning_set(WARNING_ID_REGEN_NOT_ALLOWED);
    } else {
        warning_clear(WARNING_ID_REGEN_NOT_ALLOWED);
    }
}

void UFI_throw_faults() {
    EventBits_t mask = 0;
    if (g_data_read->bps_status.BPS_Fault) 
        mask |= FAULT_BIT(FAULT_ID_BPS_FAULT);
    if (g_data_read->controls_status.Controls_Leader_Fault) 
        mask |= FAULT_BIT(FAULT_ID_CONTROLS_FAULT);
    
    // Moco faults
    if (g_data_read->motor_status.MC_FAULT_HardwareOverCurrent)
        mask |= FAULT_BIT(FAULT_ID_MOTOR_HARDWARE_OVERCURRENT);
    if (g_data_read->motor_status.MC_FAULT_SoftwareOverCurrent)
        mask |= FAULT_BIT(FAULT_ID_MOTOR_SOFTWARE_OVERCURRENT);
    if (g_data_read->motor_status.MC_FAULT_DcBusOverVoltage)
        mask |= FAULT_BIT(FAULT_ID_MOTOR_DC_BUS_OVERVOLTAGE);
    if (g_data_read->motor_status.MC_FAULT_BadMotorPositionHallSeq)
        mask |= FAULT_BIT(FAULT_ID_MOTOR_BAD_HALL_SEQUENCE);
    if (g_data_read->motor_status.MC_FAULT_WatchdogCausedLastReset)
        mask |= FAULT_BIT(FAULT_ID_MOTOR_WD_RESET);
    if (g_data_read->motor_status.MC_FAULT_ConfigRead)
        mask |= FAULT_BIT(FAULT_ID_MOTOR_CONFIG_READ);
    if (g_data_read->motor_status.MC_FAULT_15vRailUnderVoltage)
        mask |= FAULT_BIT(FAULT_ID_MOTOR_15V_UNDERVOLTAGE);
    if (g_data_read->motor_status.MC_FAULT_DesaturationFault)
        mask |= FAULT_BIT(FAULT_ID_MOTOR_DESATURATION);
    if (g_data_read->motor_status.MC_FAULT_MotorOverSpeed)
        mask |= FAULT_BIT(FAULT_ID_MOTOR_OVERSPEED);

    // Pedals faults
    if (g_data_read->accel_brake.AccelPedal_Main_Fault ||
        g_data_read->accel_brake.AccelPedal_Redundant_Fault ||
        g_data_read->accel_brake.Brake_Pressure_1_Fault ||
        g_data_read->accel_brake.Brake_Pressure_2_Fault) {
        mask |= FAULT_BIT(FAULT_ID_PEDAL_BOARD_FAULT);
    }

    // if (g_data_read->lws.LWS_Fault) 
    //     mask |= FAULT_BIT(FAULT_ID_STEERING_SENSOR_FAULT);

    faults_set_mask(mask);
}

void Task_UpdateVCUInputs(void *args __attribute__((unused))) {
    TickType_t last = xTaskGetTickCount();
    int count = 0;
    while (1) {
        // printf("Task_UpdateVCUInputs: %ld", last);
        // update from can
        VCUDataIn_t *volatile update = g_data_write;
        MotorCAN_Recv_Status(&update->motor_status, 0);
        
        MotorCAN_Recv_Velocity(&update->motor_velocity, 0);
        MotorCAN_Recv_Control_Src(&update->motor_controls_src, 0);

        CarCAN_Recv_BPS_Status(&update->bps_status, 0);
        CarCAN_Recv_Pedals_Position(&update->accel_brake, 0);
        CarCAN_Recv_Brake_Pressure1(&update->brake_pressure1, 0);
        CarCAN_Recv_Brake_Pressure2(&update->brake_pressure2, 0);
        CarCAN_Recv_Controls_Status(&update->controls_status, 0);
        CarCAN_Recv_Driver_Input(&update->driver_input, 0);
        CarCAN_Recv_LWS(&update->lws, 0);

        // if (count % 10 == 0) {
        //     printf("Motor Status: %s\r\n", (motor_status == CAN_OK) 
        //                                         ? "CAN_OK" : (motor_status == CAN_ERR) 
        //                                             ? "CAN_ERROR" : "CAN_EMPTY");
        //     printf("Motor Velocity Status: %s\r\n", (motor_velocity_status == CAN_OK) 
        //                                                 ? "CAN_OK" : (motor_velocity_status == CAN_ERR) 
        //                                                     ? "CAN_ERROR" : "CAN_EMPTY");
        //     printf("Motor Controls Src Status: %s\r\n", (motor_controls_src_status == CAN_OK) 
        //                                                 ? "CAN_OK" : (motor_controls_src_status == CAN_ERR) 
        //                                                     ? "CAN_ERROR" : "CAN_EMPTY");
        //     printf("BPS Status: %s\r\n", (bps_status == CAN_OK) 
        //                                 ? "CAN_OK" : (bps_status == CAN_ERR) 
        //                                     ? "CAN_ERROR" : "CAN_EMPTY");
        //     printf("Accel/Brake Status: %s\r\n", (accel_brake_status == CAN_OK) 
        //                                         ? "CAN_OK" : (accel_brake_status == CAN_ERR) 
        //                                             ? "CAN_ERROR" : "CAN_EMPTY");
        //     printf("Brake Pressure 1 Status: %s\r\n", (brake_pressure1_status == CAN_OK) 
        //                                             ? "CAN_OK" : (brake_pressure1_status == CAN_ERR) 
        //                                                 ? "CAN_ERROR" : "CAN_EMPTY");
        //     printf("Brake Pressure 2 Status: %s\r\n", (brake_pressure2_status == CAN_OK) 
        //                                             ? "CAN_OK" : (brake_pressure2_status == CAN_ERR) 
        //                                                 ? "CAN_ERROR" : "CAN_EMPTY");
        //     printf("Controls Status: %s\r\n", (controls_status == CAN_OK) 
        //                                     ? "CAN_OK" : (controls_status == CAN_ERR) 
        //                                         ? "CAN_ERROR" : "CAN_EMPTY");
        //     printf("Driver Input Status: %s\r\n", (driver_input_status == CAN_OK) 
        //                                         ? "CAN_OK" : (driver_input_status == CAN_ERR) 
        //                                             ? "CAN_ERROR" : "CAN_EMPTY");
        //     printf("LWS Status: %s\r\n", (lws_status == CAN_OK) 
        //                                 ? "CAN_OK" : (lws_status == CAN_ERR) 
        //                                     ? "CAN_ERROR" : "CAN_EMPTY");
        // }

        // printf("Update VCU inputs: about to switch read and write ptrs\r\n");

        VCUDataIn_t *volatile tmp;
        taskENTER_CRITICAL();
        tmp = g_data_read;
        g_data_read = g_data_write;
        g_data_write = tmp;
        taskEXIT_CRITICAL();

        // printf("Updated from CAN!\r\n");

        memcpy(g_data_write, g_data_read, sizeof(VCUDataIn_t));

        // Throw relevant faults
        UFI_throw_faults();

        rebuild_inputs();

        count++;
        
        vTaskDelayUntil(&last, pdMS_TO_TICKS(UPDATE_VCU_INPUTS_TASK_DELAY_MS));
    }
}
