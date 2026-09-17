#include "VCUStatusTask.h"
#include "Contactors.h"
#include "FSMTask.h"
#include "InitTask.h"
#include "FaultBits.h"
#include "StatusLEDs.h"
#include "UpdateVCUInputsTask.h"
#include "Watchdogs.h"
#include "event_groups.h"
#include "math.h"

#define VCU_STATUS_CAN_SEND_TIMEOUT_MS 5

void Task_BroadcastVCUStatus(void *args __attribute__((unused))) {
    uint8_t buf[CAN_DLC_VCU_STATUS];
    TickType_t last = xTaskGetTickCount();

    while (1) {
        // Byte 0
        uint8_t state = current_state.stateName & 0x0FU;
        bool motor_ready = (current_state.stateName != DISABLED) &&
                           (current_state.stateName != STATE_INIT) &&
                           (current_state.stateName != CAR_NOT_READY);
        bool motor_prech_cont_state = contactor_get_sense(MOTOR_PRE_CONTACTOR);
        bool motor_cont_state = contactor_get_sense(MOTOR_CONTACTOR);
        bool driver_inp_wdog = watchdog_is_alive(WD_IDX_DRIVER_INPUT);
        buf[0] = (state) | (motor_ready << 4) | (motor_prech_cont_state << 5) |
                 (motor_cont_state << 6) | (driver_inp_wdog << 7);

        // Byte 1
        bool pedals_wdog = watchdog_is_alive(WD_IDX_ACCEL_BRAKE);
        bool bps_wdog = watchdog_is_alive(WD_IDX_BPS_STATUS);
        bool steering_wdog = watchdog_is_alive(WD_IDX_STEERING_ANGLE);
        bool bps_fault = faults_is_active(FAULT_ID_BPS_FAULT);
        bool controls_fault = faults_is_active(FAULT_ID_CONTROLS_FAULT);
        bool motor_fault = !!(faults_get() & FAULT_MASK_MOTOR_ALL);
        bool pedals_fault = ((bool)g_data_read->accel_brake.AccelPedal_Main_Fault) ||
                             ((bool)g_data_read->accel_brake.AccelPedal_Redundant_Fault) ||
                             ((bool)g_data_read->accel_brake.BrakePedal_Main_Fault) ||
                             ((bool)g_data_read->accel_brake.BrakePedal_Redundant_Fault); //||
                            //  (fabs(g_data_read->accel_brake.AccelPedal_Main_Pos -
                            //       g_data_read->accel_brake.AccelPedal_Redundant_Pos) <
                            //   ACCEPTABLE_PEDAL_DEVIATION);

        bool steering_fault = false; //g_data_read->lws.LWS_Fault;
        buf[1] = (pedals_wdog) | (bps_wdog << 1) | (steering_wdog << 2) | (bps_fault << 3) |
                 (controls_fault << 4) | (motor_fault << 5) | (pedals_fault << 6) |
                 (steering_fault << 7);

        buf[2] = g_data_read->motor_controls_src.Motor_Command_Source |
                 ((current_state.stateName == REGEN) << 1) |
                 (g_data_read->bps_status.BPS_Regen_OK << 2) |
                 (faults_is_active(FAULT_ID_PRECHARGE_TIMEOUT) << 3) |
                 (faults_is_active(FAULT_ID_PRECHARGE_SENSE_TIMEOUT) << 4) |
                 (faults_is_active(FAULT_ID_PRECHARGE_SENSE_MISMATCH) << 5) |
                 (faults_is_active(FAULT_ID_MOTOR_SENSE_MISMATCH) << 6) |
                 (faults_is_active(FAULT_ID_MOTOR_SENSE_TIMEOUT) << 7);

        buf[3] = (faults_is_active(FAULT_ID_BATTERY_OVERVOLTAGE)) |
                 (faults_is_active(FAULT_ID_BATTERY_UNDERVOLTAGE) << 1) |
                 (faults_is_active(FAULT_ID_MOTOR_GT_BATTERY) << 2) | // TODO check if this is right
                 (faults_is_active(FAULT_ID_MOTOR_LT_BATTERY) << 3) |
                 //(faults_any_active() << 4) | // TODO: make this "other" but not enough faults bits rn
                 //The rest of these bits and one of the next one are all warnings
                 (warning_is_active(WARNING_ID_MOTOR_DIRECTION_CHANGE_LOCKOUT) << 5) |
                 (warning_is_active(WARNING_ID_TIPPING_LIMIT_ACTIVE) << 6) |
                 (warning_is_active(WARNING_ID_REGEN_NOT_ALLOWED) << 7);

        buf[4] = (warning_is_active(WARNING_ID_REGEN_NOT_ENABLED) << 0) |
                 (fsm_is_input_set(BRAKE_BIT) << 1) |
                 (fsm_is_input_set(PRECHARGE_COMPLETE_BIT) << 2) |
                 (fsm_is_input_set(CRUISE_CONTROL_BUTTON_BIT) << 3) |
                 (fsm_is_input_set(REGEN_BUTTON_BIT) << 4) |
                 (fsm_is_input_set(REGEN_ENABLED_BIT) << 5) |
                 (fsm_is_input_set(READY_TO_REGEN_BIT) << 6) |
                 (fsm_is_input_set(FORWARD_BIT) << 7);

        buf[5] = (fsm_is_input_set(NEUTRAL_BIT) << 0) |
                 (fsm_is_input_set(REVERSE_BIT) << 1);


        FDCAN_TxHeaderTypeDef tx_header = {0};
        tx_header.Identifier = CAN_ID_VCU_STATUS;
        tx_header.IdType = FDCAN_STANDARD_ID;
        tx_header.TxFrameType = FDCAN_DATA_FRAME;
        tx_header.DataLength = FDCAN_DLC_BYTES(CAN_DLC_VCU_STATUS);
        tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx_header.BitRateSwitch = FDCAN_BRS_OFF;
        tx_header.FDFormat = FDCAN_CLASSIC_CAN;
        tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        tx_header.MessageMarker = 0;

        CarCAN_Send(&tx_header, buf, pdMS_TO_TICKS(VCU_STATUS_CAN_SEND_TIMEOUT_MS));

        LED_toggle(HB);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(VCU_STATUS_TASK_DELAY_MS));
    }
}
