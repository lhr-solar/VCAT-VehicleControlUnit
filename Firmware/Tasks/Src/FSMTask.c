/**
 * @file FSM.c
 * @brief Motor controller FSM implementation
 * @copyright Copyright (c) 2026 UT Longhorn Racing Solar
 */

#define DEFINE_FSM_TABLE
#include "fsm_table_dnr.h"

#include "FSMTask.h"
#include "InitTask.h"
#include "UpdateVCUInputsTask.h"
#include "FaultBits.h"
#include "Watchdogs.h"
#include "motor.h"

StaticEventGroup_t fsmInputBuffer = {0};
EventGroupHandle_t fsmInputGroup = {0};
MocoState_t current_state = {0};

static volatile uint16_t fsm_inputs = 0;

// forward declarations for FSM[] handler table wiring in fsm_init()
static void handle_state_init(void);
static void handle_state_not_ready(void);
static void handle_state_forward(void);
static void handle_state_neutral(void);
static void handle_state_reverse(void);
static void handle_state_regen(void);
static void handle_state_cruise(void);
static void handle_state_disabled(void);

static void handle_drive_state(bool);

void fsm_init(void) {
    FSM[STATE_INIT].stateHandler = handle_state_init;
    FSM[FORWARD_DRIVE].stateHandler = handle_state_forward;
    FSM[NEUTRAL_DRIVE].stateHandler = handle_state_neutral;
    FSM[REVERSE_DRIVE].stateHandler = handle_state_reverse;
    FSM[REGEN].stateHandler = handle_state_regen;
    FSM[CRUISE_CONTROL].stateHandler = handle_state_cruise;
    FSM[DISABLED].stateHandler = handle_state_disabled;
    FSM[CAR_NOT_READY].stateHandler = handle_state_not_ready;

    current_state = FSM[STATE_INIT];
    current_state.stateHandler();

    fsmInputGroup = xEventGroupCreateStatic(&fsmInputBuffer);
}

void fsm_step(void) {
    fsm_inputs = xEventGroupGetBits(fsmInputGroup);
    motor_set_performance_mode((fsm_inputs & CRUISE_CONTROL_BUTTON_BIT) != 0);
    current_state = FSM[current_state.NextStates[fsm_inputs]];
    if (current_state.stateHandler) current_state.stateHandler();
}

void fsm_disable(void) { current_state = FSM[DISABLED]; }
void fsm_recover(void) { current_state = FSM[CAR_NOT_READY]; }
uint16_t fsm_get_fsm_inputs(void) { return (uint16_t)fsm_inputs; }
bool fsm_is_over_rollover_speed(void) { return motor_is_over_rollover_speed(); }

void fsm_set_all_inputs(EventBits_t mask) {
    taskENTER_CRITICAL();
    xEventGroupClearBits(fsmInputGroup, FSM_INPUTS_MASK_ALL);
    xEventGroupSetBits(fsmInputGroup, mask & FSM_INPUTS_MASK_ALL);
    taskEXIT_CRITICAL();
}

void fsm_set_input(EventBits_t mask) {
    xEventGroupSetBits(fsmInputGroup, mask & FSM_INPUTS_MASK_ALL);
}

uint16_t fsm_get_inputs(void) {
    return (uint16_t)xEventGroupGetBits(fsmInputGroup);
}

bool fsm_is_input_set(InputBits_t bit) {
    return (xEventGroupGetBits(fsmInputGroup) & bit) != 0;
}

//// state handlers

static void handle_state_init(void) { 
    current_state = FSM[CAR_NOT_READY]; 
}

static void handle_state_disabled(void) { 
    motor_reset_current_ramp(); 
    CAN_Send_Drive_Cmd(0.0f, 0.0f, 0); 
}
static void handle_state_not_ready(void) { 
    motor_reset_current_ramp(); 
    CAN_Send_Drive_Cmd(0.0f, 0.0f, 0); 
}

static void handle_state_regen(void) { 
    motor_reset_current_ramp(); 
    CAN_Send_Drive_Cmd(0.0f, 0.15f, 0); 
}

static void handle_state_neutral(void) { 
    motor_reset_current_ramp(); 
    CAN_Send_Drive_Cmd(0.0f, 0.0f, 0); 
}

static void handle_state_forward(void) {
    handle_drive_state(false);
}

static void handle_state_reverse(void) {
    handle_drive_state(true);
}

static void handle_drive_state(bool reverse) {
    float velocity_setpoint = 0.0f;
    float current_setpoint = 0.0f;
    float current_pwr = 0.0f;

    const float motor_rpm = g_data_read->motor_velocity.MC_MotorVelocity;
    const float vehicle_velocity = g_data_read->motor_velocity.MC_VehicleVelocity;
    const int16_t lws_angle = g_data_read->lws.LWS_Angle;
    const uint8_t accel = g_data_read->accel_brake.AccelPedal_Main_Pos;

    const bool is_wrong_direction =
        (!reverse && vehicle_velocity < -0.5f) ||
        ( reverse && vehicle_velocity >  0.5f);

    if (is_wrong_direction) {
        // If we're moving opposite the requested direction,
        // force zero torque until vehicle slows down
        velocity_setpoint = 0.0f;
        current_setpoint = 0.0f;
        warning_set(WARNING_ID_MOTOR_DIRECTION_CHANGE_LOCKOUT);
    } else {
        if (!reverse) warning_clear(WARNING_ID_REGEN_NOT_ALLOWED);

        velocity_setpoint = reverse ? -(float)MOTOR_MAX_RPM : (float)MOTOR_MAX_RPM;

        current_setpoint = motor_get_drive_current(motor_rpm, vehicle_velocity, lws_angle, accel);
        current_pwr = motor_get_pwr_current(accel);
    }

    current_setpoint = motor_ramp_current(current_setpoint);

    CAN_Send_Drive_Cmd(velocity_setpoint, current_setpoint, pdMS_TO_TICKS(1));
    MotorCAN_Send_Power_Cmd(current_pwr, pdMS_TO_TICKS(1));
    CarCAN_Send_Power_Cmd(current_pwr, pdMS_TO_TICKS(1));
    printf("%s Drive cmd: %f vel, %f curr\r\n",
           reverse ? "Reverse" : "Forward",
           velocity_setpoint,
           current_setpoint);
}

static void handle_state_cruise(void) {
    float velocity = motor_is_over_rollover_speed() ? 0.0f : (float)MOTOR_MAX_RPM;
    float current = motor_get_drive_current(g_data_read->motor_velocity.MC_MotorVelocity,
                                            g_data_read->motor_velocity.MC_VehicleVelocity,
                                            g_data_read->lws.LWS_Angle,
                                            g_data_read->accel_brake.AccelPedal_Main_Pos);
    CAN_Send_Drive_Cmd(velocity, motor_ramp_current(current), 0);
}

//////// rtos tasks

void Task_FSM(void *args __attribute__((unused))) {
    TickType_t last = xTaskGetTickCount();
    while (1) {
        fsm_step();
        vTaskDelayUntil(&last, pdMS_TO_TICKS(FSM_TASK_DELAY_MS));
    }
}
