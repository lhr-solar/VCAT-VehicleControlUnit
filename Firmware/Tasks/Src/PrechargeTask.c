#include "PrechargeTask.h"
#include "UpdateVCUInputsTask.h"

#define PRECHARGE_PRINTF_DEBUG_PERIOD_MS 2000
#define PRECHARGE_PRINTF_DEBUG_COUNTER (PRECHARGE_PRINTF_DEBUG_PERIOD_MS / PRECHARGE_TASK_DELAY_MS)
#define PRECHARGE_CAN_SEND_TIMEOUT_MS 5

uint32_t battery_voltage = 0;
uint32_t motor_voltage = 0;
PrechargeState_e g_precharge_state = PRECHARGE_STATE_WAITING;

const char *precharge_state_names[PRECHARGE_STATE_COUNT] = {
#define X(name) [PRECHARGE_STATE_##name] = #name,
    PRECHARGE_STATE_LIST(X)
#undef X
};

void PT_ign_off() {
    printf("Ignition OFF, opening motor precharge contactor\r\n");

    if (contactor_set(MOTOR_PRE_CONTACTOR, OPEN, CALLBACK_BLOCKING_TIME, NORMAL) != SUCCESS) {
        faults_set(FAULT_ID_PRECHARGE_SENSE_TIMEOUT);
    }

    printf("Ignition OFF, opening motor contactor\r\n");

    if (contactor_set(MOTOR_CONTACTOR, OPEN, CALLBACK_BLOCKING_TIME, NORMAL) != SUCCESS) {
        faults_set(FAULT_ID_MOTOR_SENSE_TIMEOUT);
    }

    taskENTER_CRITICAL();
    g_precharge_state = PRECHARGE_STATE_WAITING;
    taskEXIT_CRITICAL();

    printf("Ignition OFF, shutdown complete\r\n");
}

// TODO add message countign to c=transition
void PT_check_ign() {
    static bool last_ign;
    bool curr_ign = g_data_read->driver_input.Ignition_Motor;

    if (curr_ign && !last_ign) {
        taskENTER_CRITICAL();
        g_precharge_state = PRECHARGE_STATE_INITIAL;
        taskEXIT_CRITICAL();
        printf("Ignition ON, starting precharge sequence\r\n");
    } else if (!curr_ign && last_ign) {
        printf("Ignition OFF, stopping precharge sequence\r\n");
        PT_ign_off();
    }

    last_ign = curr_ign;
}

void Fault_Checker(uint32_t Motor_Voltage, uint32_t Battery_Voltage) {
    if (Motor_Voltage >
        (Battery_Voltage * VOLTAGE_TOLERANCE_NUMERATOR / VOLTAGE_TOLERANCE_DENOMINATOR)) {
        // faults_set(FAULT_ID_MOTOR_GT_BATTERY);
    }

    // is battery booming
    if (Battery_Voltage > OVERVOLTAGE_THRESHOLD_MV) {
        // faults_set(FAULT_ID_BATTERY_OVERVOLTAGE);
    }

    // Is batt voltage too low or disconnected?
    if (Battery_Voltage < UNDERVOLTAGE_THRESHOLD_MV) {
        faults_set(FAULT_ID_BATTERY_UNDERVOLTAGE);
    }

    if (contactor_get_sense(MOTOR_CONTACTOR) != contactor_get_commanded_state(MOTOR_CONTACTOR)) {
        faults_set(FAULT_ID_MOTOR_SENSE_MISMATCH);
    }

    if (contactor_get_sense(MOTOR_PRE_CONTACTOR) !=
        contactor_get_commanded_state(MOTOR_PRE_CONTACTOR)) {
        faults_set(FAULT_ID_PRECHARGE_SENSE_MISMATCH);
    }
}

static void PT_print_state(PrechargeState_e state) {
    printf("Precharge state: %s\r\n", precharge_state_names[state]);
}

void Task_Precharge() {
    TickType_t timeout_start_tick = 0;
    ADC_Sense_Result ADC_Result = {0};
    uint8_t can_send_errors = 0;
    uint8_t printDebugCounter = 0;
    LED_set(HALL_EFFECT, LED_ON);

    while (1) {
        if (Read_ADC(ADC_TIMEOUT_MS, &ADC_Result) != ADC_SENSE_OK) {
            faults_set(FAULT_ID_PRECHARGE_SENSE_TIMEOUT);
        }

        battery_voltage = ADC_Result.Battery_Voltage;
        motor_voltage = ADC_Result.Motor_Voltage;

        can_status_t result = CarCAN_Send_Precharge_Voltages(
            motor_voltage, battery_voltage, pdMS_TO_TICKS(PRECHARGE_CAN_SEND_TIMEOUT_MS));

        if (result == CAN_ERR) {
            can_send_errors++;
            printf("Precharge: CarCAN_Send_Precharge_Voltages FAILED (count=%u)\r\n", can_send_errors);
        } else {
            can_send_errors = 0;
        }

        PT_check_ign();

        PrechargeState_e curr_state;
        taskENTER_CRITICAL();
        curr_state = g_precharge_state;
        taskEXIT_CRITICAL();

        switch (curr_state) {
            case PRECHARGE_STATE_WAITING:
                // Wait for ignition on message from driver input task,
                // then move to initial precharge state
                // printf("Precharge State: Waiting for Ignition\r\n");
                break;

            // Startup state: Closes main contactor and moves to precharging state
            case PRECHARGE_STATE_INITIAL:
                // printf("Precharge State: Initial\r\n");
                
                // dont attempt to close contactors before bps ready
                if (!g_data_read->bps_status.HV_Plus_Contactor_State ||
                    !g_data_read->bps_status.HV_Minus_Contactor_State || 
                    g_data_read->bps_status.BPS_Fault) {
                    break;
                }

                if (contactor_set(MOTOR_CONTACTOR, CLOSED, CALLBACK_BLOCKING_TIME, NORMAL) !=
                    SUCCESS) {
                    faults_set(FAULT_ID_MOTOR_SENSE_TIMEOUT);
                }

                taskENTER_CRITICAL();
                g_precharge_state = PRECHARGE_STATE_PRECHARGING;
                taskEXIT_CRITICAL();

                set_MotorSafeBit(MOTOR_CONTACTOR_ENABLED);

                // Start a timer for precharging
                timeout_start_tick = xTaskGetTickCount();
                break;

            // Precharging state: Waits for battery voltage to reach 90% of motor voltage,
            // then closes precharge contactor and moves to run state
            case PRECHARGE_STATE_PRECHARGING:
                // Check for faults while precharging, if any fault conditions are met,
                // will call fault handler and not proceed with precharge sequence
                Fault_Checker(motor_voltage, battery_voltage);

                // Check how long we've been precharging for, fault if not precharged after
                // PRECHARGE_TIMEOUT_MS
                const TickType_t timeout_curr_tick = xTaskGetTickCount();
                // printf("Precharge State: Precharging\r\n");

                // Faults if precharging takes too long
                if ((timeout_curr_tick - timeout_start_tick) >
                    pdMS_TO_TICKS(PRECHARGE_TIMEOUT_MS)) {
                    // Check if motor voltage is within 90% of battery voltage (precharge complete)
                    if (motor_voltage * RATIO_SCALE >= battery_voltage * PRECHARGE_THRESHOLD_90) {
                        if (contactor_set(MOTOR_PRE_CONTACTOR, CLOSED, CALLBACK_BLOCKING_TIME,
                                          false) != SUCCESS) {
                            faults_set(FAULT_ID_MOTOR_SENSE_TIMEOUT);
                        }
                        taskENTER_CRITICAL();
                        g_precharge_state = PRECHARGE_STATE_COMPLETE;
                        taskEXIT_CRITICAL();
                    } else {
                        faults_set(FAULT_ID_PRECHARGE_TIMEOUT);
                    }
                }
                set_MotorSafeBit(MOTOR_PRECHARGE_CONTACTOR_ENABLED);
                break;

            // Run state: Continuously checks that motor voltage stays
            // within 80% of battery voltage (Threshold lowered to 80%)
            // after precharge is complete
            case PRECHARGE_STATE_COMPLETE:
                // Check for faults while precharging, if any fault
                // conditions are met, will call fault handler and
                // not proceed with precharge sequence
                Fault_Checker(motor_voltage, battery_voltage);

                // Use 80% threshold for hysteresis
                if (motor_voltage * RATIO_SCALE < battery_voltage * PRECHARGE_THRESHOLD_80) {
                    faults_set(FAULT_ID_MOTOR_LT_BATTERY);
                }
                break;
            default:
                break;
        }

        taskENTER_CRITICAL();
        curr_state = g_precharge_state;
        taskEXIT_CRITICAL();

        printDebugCounter++;
        if (printDebugCounter >= PRECHARGE_PRINTF_DEBUG_COUNTER) {

            // prints battery and motor voltage
            printf("Motor: %ld mV | Battery: %ld mV\r\n", motor_voltage, battery_voltage);

            // prints current precharge state
            PT_print_state(curr_state);
            printDebugCounter = 0;
        }

        // set the Precharge Complete LED
        LED_set(PRECHARGE_COMPLETE, curr_state == PRECHARGE_STATE_COMPLETE ? LED_ON : LED_OFF);

        // update the motor safe bits with Contactor state
        if (contactor_get_sense(MOTOR_PRE_CONTACTOR) == CLOSED) {
            set_MotorSafeBit(MOTOR_PRECHARGE_CONTACTOR_ENABLED);
        } else {
            clear_MotorSafeBit(MOTOR_PRECHARGE_CONTACTOR_ENABLED);
        }

        if (contactor_get_sense(MOTOR_CONTACTOR) == CLOSED) {
            set_MotorSafeBit(MOTOR_CONTACTOR_ENABLED);
        } else {
            clear_MotorSafeBit(MOTOR_CONTACTOR_ENABLED);
        }

        // LED_toggle(HB);
        vTaskDelay(pdMS_TO_TICKS(PRECHARGE_TASK_DELAY_MS));
    }
}
