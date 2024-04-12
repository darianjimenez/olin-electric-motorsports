#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdlib.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/io.h>
#include "libs/can/mob.h"

#include "libs/gpio/api.h"
#include "libs/timer/api.h"

#include "air_config.h"
#include "utils/timer.h"
#include "utils/utils.h"
#include "vehicle/mkvi/software/air_control/can_api.h"

#include "projects/btldr/btldr_lib.h"
#include "projects/btldr/git_sha.h"
#include "projects/btldr/libs/image/api.h"

/*
 * Required for btldr
 */
image_hdr_t image_hdr __attribute__((section(".image_hdr"))) = {
    .image_magic = IMAGE_MAGIC,
    .git_sha = STABLE_GIT_COMMIT,
};

volatile bool send_can = false;

// A global variable that is used to tell if we are connected to the vehicle
// (MOTOR_CONTROLLER) or the charger (CHARGER)
static enum tractive_system tractive_sys = MOTOR_CONTROLLER;

void timer0_isr(void) {
    send_can = true;
}

static enum can_id_mode_e id_mode = ID_MODE_STANDARD;

static inline void mob_configure_alt(uint32_t id, uint32_t mask, uint8_t dlc) {
    switch (id_mode) {
        case ID_MODE_STANDARD: {
            CANIDT1 = id >> 3;
            CANIDT2 = id << 5;
            CANIDT3 = 0;
            CANIDT4 = 0;
            CANIDM1 = mask >> 3;
            CANIDM2 = mask << 5;
            CANIDM3 = 0;
            CANIDM4 = (1 << RTRMSK) | (1 << IDEMSK);
            CANCDMOB &= ~(1 << IDE);
            CANCDMOB |= ((dlc & 0xF) << DLC0);
        } break;
        case ID_MODE_EXTENDED: {
            CANIDT1 = id >> 21;
            CANIDT2 = id >> 13;
            CANIDT3 = id >> 5;
            CANIDT4 = id << 3;
            CANIDM1 = mask >> 21;
            CANIDM2 = mask >> 13;
            CANIDM3 = mask >> 5;
            CANIDM4 = (mask << 3) | (1 << RTRMSK) | (1 << IDEMSK);
            CANCDMOB |= (1 << IDE) | ((dlc & 0xF) << DLC0);
        } break;
    }
}

int can_send_alt(can_frame_t* frame) {
    PORTB |= 1;
    while (CANGSTA & (1 << TXBSY))
        ;

    select_mob(frame->mob);
    mob_reset();

    mob_configure_alt(frame->id, 0, frame->dlc);

    // Set the message
    for (uint8_t i = 0; i < frame->dlc; i++) {
        mob_write_data(frame->data[i]);
    }

    mob_enable_tx();

    // bool once;
    // Wait for TX to finish
    bool once = true;
    while (!(CANSTMOB & (1 << TXOK)))
        // gpio_set_pin(GENERAL_LED);
        if (once) {
          start_time = get_time();
          once = false;
        }
        if (get_time() - start_time > 1000) {
          return 1;
        }
    CANSTMOB &= ~(1 << TXOK);

    // gpio_clear_pin(GENERAL_LED);
    return 0;
}

void  can_send_air_control_critical_alt(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // We can be sure here that the CAN data struct won't change here

        can_tools_air_control_critical_pack(
            air_control_critical_data,
            // We can safely discard the volatile qualifier because we are in an
            // ATOMIC block, so the value will not be changed in an ISR
            (const struct can_tools_air_control_critical_t*) &air_control_critical,
            4
        );
    }
    if (can_send_alt(&air_control_critical_msg)) {
      can_init_air_control();
      gpio_toggle_pin(GENERAL_LED);
    }
}

static void set_fault(enum air_fault_e the_fault) {
    // gpio_set_pin(FAULT_LED);

    if (air_control_critical.air_fault == AIR_FAULT_NONE) {
        // Only update fault state for the first fault to occur
        air_control_critical.air_fault = the_fault;
    }
}

// static int set_charger_connected() {
//     uint32_t start_time = get_time();
//
//     (void)can_receive_bms_core();
//
//     uint8_t rc = 1;
//
//     do {
//         rc = can_poll_receive_bms_core();
//
//         if (rc == 0) {
//             tractive_sys = (enum tractive_system) bms_core.charger_connected;
//             return 0;
//         } else if (rc == 1) {
//             // CAN error--fault
//             return 1;
//         } else if (get_time() - start_time > 1000) {
//             // Timeout, so use default
//             return 0;
//         }
//     } while (rc != 0);
//
//     // Catch-all, shouldn't happen
//     return 1;
// }

void pcint0_callback(void) {
    air_control_critical.ss_tsms = !gpio_get_pin(SS_TSMS);
    air_control_critical.ss_imd = !gpio_get_pin(SS_IMD_LATCH);
    air_control_critical.ss_mpc = !gpio_get_pin(SS_MPC);
    air_control_critical.ss_hvd_conn = !gpio_get_pin(SS_TSMP);
    air_control_critical.ss_hvd = !gpio_get_pin(SS_HVD);
}

void pcint1_callback(void) {
    air_control_critical.ss_bms = !gpio_get_pin(SS_BMS);
    air_control_critical.air_p_status = !!gpio_get_pin(AIR_P_WELD_DETECT);
    air_control_critical.air_n_status = !!gpio_get_pin(AIR_N_WELD_DETECT);
}

void pcint2_callback(void) {
    // air_control_critical.imd_status = !!gpio_get_pin(IMD_SENSE);

    // if (!air_control_critical.imd_status) {
    //     set_fault(AIR_FAULT_IMD_STATUS);
    // }
}

/*
 * Run through initial checks to ensure safe operation. Checks are:
 *  - BMS voltage is above minimum
 *  - Motor controller voltage is close to 0
 *  - Both AIRs are open
 *  - Shutdown circuit is open (SS_TSMS is open)
 *  - IMD is OK
 */
static int initial_checks(void) {
    int rc = 0;

    /*
     * Get MC and BMS voltages
     *
     * Will poll for 1 second and if the CAN message isn't received, will fault
     */
    int16_t bms_voltage = 0;
    rc = get_bms_voltage(&bms_voltage);

    if (rc == 1) {
        set_fault(AIR_FAULT_CAN_ERROR);
        goto bail;
    } else if (rc == 2) {
        set_fault(AIR_FAULT_CAN_BMS_TIMEOUT);
        rc = 1;
        goto bail;
    }

    if (bms_voltage < BMS_VOLTAGE_THRESHOLD_LOW) {
        set_fault(AIR_FAULT_BMS_VOLTAGE);
        rc = 1;
        goto bail;
    }

    can_send_air_control_critical_alt();

    int16_t mc_voltage = 0;
    rc = get_tractive_voltage(&mc_voltage, tractive_sys, 1000);

    if (rc == 1) {
        set_fault(AIR_FAULT_CAN_ERROR);
        goto bail;
    } else if (rc == 2) {
        set_fault(AIR_FAULT_CAN_MC_TIMEOUT);
        rc = 1;
        goto bail;
    }

    if (mc_voltage > TRACTIVE_THRESHOLD_LOW_dV) {
        set_fault(AIR_FAULT_TRACTIVE_VOLTAGE);
        rc = 1;
        goto bail;
    }

    can_send_air_control_critical_alt();

    // The following checks ensure that the hardware is in the correct initial
    // state.
    air_control_critical.air_p_status = !!gpio_get_pin(AIR_P_WELD_DETECT);
    air_control_critical.air_n_status = !!gpio_get_pin(AIR_N_WELD_DETECT);

    if (air_control_critical.air_p_status) {
        set_fault(AIR_FAULT_AIR_P_WELD);
        rc = 1;
        goto bail;
    }

    can_send_air_control_critical_alt();

    if (air_control_critical.air_n_status) {
        set_fault(AIR_FAULT_AIR_N_WELD);
        rc = 1;
        goto bail;
    }

    can_send_air_control_critical_alt();

    if (!gpio_get_pin(SS_TSMS)) {
        // SS_TSMS should start high
        air_control_critical.ss_tsms = true;
        set_fault(AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
        rc = 1;
        goto bail;
    }

    can_send_air_control_critical_alt();

    // Wait for IMD to stabilize
    _delay_ms(IMD_STABILITY_CHECK_DELAY_MS);

    if (!gpio_get_pin(IMD_SENSE)) {
        // IMD_SENSE pin should start high
        air_control_critical.imd_status = false;
        set_fault(AIR_FAULT_IMD_STATUS);
        rc = 1;
        goto bail;
    } else {
        air_control_critical.imd_status = true;
    }

    can_send_air_control_critical_alt();

bail:
    return rc;
}

static void state_machine_run(void) {
    if (air_control_critical.air_fault != AIR_FAULT_NONE) {
        air_control_critical.air_state = AIR_STATE_FAULT;
    }

    switch (air_control_critical.air_state) {
        case AIR_STATE_IDLE: {
            // Idle until shutdown circuit is closed
            if (air_control_critical.ss_tsms) {
                air_control_critical.air_state
                    = AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED;
            }
        } break;
        case AIR_STATE_SHUTDOWN_CIRCUIT_CLOSED: {
            /*
             * This pattern ensures that we only call get_time() once because we
             * only want to capture the time that PRECHARGE starts
             */
            static bool once = true;

            if (once) {
                start_time = get_time();
                once = false;
            }

            if (get_time() - start_time < 200) {
                if (!air_control_critical.air_p_status) {
                    air_control_critical.air_state = AIR_STATE_PRECHARGE;
                    can_send_air_control_critical_alt();
                    once = true;
                }
            } else {
                set_fault(AIR_FAULT_SHUTDOWN_IMPLAUSIBILITY);
                once = true;
            }
            return;
        } break;
        case AIR_STATE_PRECHARGE: {
            // Start precharge
            gpio_set_pin(PRECHARGE_CTL);

            // Get pack voltage to compare with MC voltage
            int16_t pack_voltage = 0;
            int16_t tractive_voltage = 0;
            int rc;

            rc = get_bms_voltage(&pack_voltage);

            if (rc != 0) {
                set_fault(AIR_FAULT_CAN_BMS_TIMEOUT);
                return;
            }

            // Set correct scale for pack voltage
            pack_voltage = (pack_voltage << 8) * 0.0001; // (x << 8 == x * 256)

            /*
             * This pattern ensures that we only call get_time() once because we
             * only want to capture the time that PRECHARGE starts
             */
            static bool once = true;
            if (once) {
                start_time = get_time();
                once = false;
            }

            if (get_time() - start_time >= PRECHARGE_DELAY_MS) {
                rc = get_tractive_voltage(&tractive_voltage, tractive_sys,
                                          500); // 500ms
                                                // timeout
                if (rc != 0) {
                    set_fault(AIR_FAULT_CAN_MC_TIMEOUT);
                    once = true;
                    return;
                }

                // Set correct scale for MC voltage
                tractive_voltage = tractive_voltage * 0.1;

                // if (tractive_voltage > (PRECHARGE_THRESHOLD * pack_voltage)) {
                // if(get_time() > start_time + 2000) {
                if(1) {
                    gpio_set_pin(AIR_N_LSD); // Close AIR negative
                    gpio_clear_pin(PRECHARGE_CTL); // Open precharge relay
                    once = true;
                    air_control_critical.air_state = AIR_STATE_TS_ACTIVE;
                    return;
                } else {
                    once = true;
                    // set_fault(AIR_FAULT_PRECHARGE_FAIL);
                    return;
                }
            } else {
                // Yield for timer to complete
                return;
            }
        } break;
        case AIR_STATE_TS_ACTIVE: {
            // If any of the shutdown nodes open, the SS_TSMS will trigger as
            // well, so we can just read that one (it is the last node in the
            // shutdown circuit.
            if (!air_control_critical.ss_tsms) {
                air_control_critical.air_state = AIR_STATE_DISCHARGE;
            }
        } break;
        case AIR_STATE_DISCHARGE: {
            gpio_clear_pin(AIR_N_LSD);

            /*
             * This pattern ensures that we only call get_time() once because we
             * only want to capture the time that DISCHARGE starts
             */
            static bool once = true;

            if (once) {
                start_time = get_time();
                once = false;
            }

            // Wait 100ms and then check to make sure both AIRs are open
            if (get_time() - start_time > 100) {
                if (air_control_critical.air_p_status
                    && air_control_critical.air_n_status) {
                    air_control_critical.air_state = AIR_STATE_FAULT;
                    set_fault(AIR_FAULT_BOTH_AIRS_WELD);
                    once = true;
                    return;
                } else if (air_control_critical.air_p_status) {
                    air_control_critical.air_state = AIR_STATE_FAULT;
                    set_fault(AIR_FAULT_AIR_P_WELD);
                    once = true;
                    return;
                } else if (air_control_critical.air_n_status) {
                    air_control_critical.air_state = AIR_STATE_FAULT;
                    set_fault(AIR_FAULT_AIR_N_WELD);
                    once = true;
                    return;
                }
            } else {
                // Hasn't been 100ms yet, continue to the next loop
                return;
            }

            // Both AIRs should be open here

            // Monitor tractive voltage
            int16_t tractive_voltage = 0;

            if (get_time() - start_time < DISCHARGE_TIMEOUT) {
                int rc = get_tractive_voltage(&tractive_voltage, tractive_sys,
                                              500);

                if (rc == 1) {
                    set_fault(AIR_FAULT_CAN_ERROR);
                    once = true;
                    return;
                }

                if (rc == 2) {
                    set_fault(AIR_FAULT_CAN_MC_TIMEOUT);
                    once = true;
                    return;
                }

                if (rc == 0) {
                    if (tractive_voltage < TRACTIVE_THRESHOLD_LOW_dV) {
                        // Tractive system voltage has fallen below 5V
                        once = true;
                        air_control_critical.air_state = AIR_STATE_IDLE;
                        return;
                    }
                }
            } else {
                // 10 seconds have elapsed and we haven't fallen below the
                // threshold
                set_fault(AIR_FAULT_DISCHARGE_FAIL);
                once = true;
                return;
            }
        } break;
        case AIR_STATE_FAULT: {
            // gpio_set_pin(FAULT_LED);
            gpio_clear_pin(PRECHARGE_CTL);
            gpio_clear_pin(AIR_N_LSD);
        } break;
        default: {
            // Shouldn't happen, but just in case
            air_control_critical.air_state = AIR_STATE_FAULT;
        } break;
    }
}


int main(void) {
    can_init_air_control();
    timer_init(&timer0_cfg);
    timer_init(&timer1_cfg);
    updater_init(BTLDR_ID, 5);

    gpio_set_mode(PRECHARGE_CTL, OUTPUT);
    gpio_set_mode(AIR_N_LSD, OUTPUT);
    gpio_set_mode(GENERAL_LED, OUTPUT);
    gpio_set_mode(FAULT_LED, OUTPUT);

    gpio_set_mode(IMD_SENSE, INPUT);
    gpio_set_mode(SS_TSMS, INPUT);
    gpio_set_mode(SS_IMD_LATCH, INPUT);
    gpio_set_mode(SS_BMS, INPUT);
    gpio_set_mode(SS_MPC, INPUT);
    gpio_set_mode(SS_TSMP, INPUT);
    gpio_set_mode(SS_HVD, INPUT);

    gpio_enable_interrupt(SS_TSMS);
    gpio_enable_interrupt(SS_IMD_LATCH);
    gpio_enable_interrupt(SS_BMS);
    gpio_enable_interrupt(SS_MPC);
    gpio_enable_interrupt(SS_TSMP);
    gpio_enable_interrupt(SS_HVD);
    gpio_enable_interrupt(IMD_SENSE);
    gpio_enable_interrupt(AIR_N_WELD_DETECT);
    gpio_enable_interrupt(AIR_P_WELD_DETECT);

    // Ensure pull-ups are disabled
    gpio_clear_pin(SS_TSMS);
    gpio_clear_pin(SS_BMS);
    gpio_clear_pin(SS_IMD_LATCH);
    gpio_clear_pin(SS_MPC);
    gpio_clear_pin(SS_TSMP);
    gpio_clear_pin(SS_HVD);

    sei();
    air_control_critical.air_state = AIR_STATE_INIT;

    can_send_air_control_critical_alt();

    // bool once_alt = true;
    // bool once_alt2 = true;
    // volatile uint32_t start_time;
    // volatile uint32_t start_time2;
    // while(1){
    //   if (once_alt){
    //     start_time = get_time();
    //     once_alt = false;
    //   }
    //   if (once_alt2){
    //     start_time2 = get_time();
    //     once_alt2 = false;
    //   }
    //   if (get_time() - start_time > 10){
    //     // gpio_toggle_pin(GENERAL_LED);
    //     can_send_air_control_critical_alt();
    //     once_alt = true;
    //   }
    //   if (get_time() - start_time2 > 5000){
    //     gpio_toggle_pin(GENERAL_LED);
    //     can_init_air_control();
    //     once_alt2 = true;
    //   }
    // }

    // set_charger_connected();

    // can_send_air_control_critical_alt();

    // gpio_set_pin(GENERAL_LED);

    pcint0_callback();
    pcint1_callback();
    pcint2_callback();

    if (initial_checks() != 0) {
        goto fault;
    }

    // Clear general LED to indicate that initialization has completed
    // gpio_clear_pin(GENERAL_LED);

    // Get initial states of pins
    pcint0_callback();
    pcint1_callback();
    pcint2_callback();

    can_send_air_control_critical_alt();
    // gpio_clear_pin(GENERAL_LED);

    air_control_critical.air_state = AIR_STATE_IDLE;

    while (1) {
        if (run_1ms) {
            state_machine_run();
            
            // can_send_air_control_critical_alt();
            run_1ms = false;
        }

        // // Updates can only occur when the AIR control state machine is in IDLE
        // if (air_control_critical.air_state == AIR_STATE_IDLE) {
            updater_loop();
        // }

        if (send_can) {
            can_send_air_control_critical_alt();
            send_can = false;
        }
    }

fault:
    // gpio_set_pin(FAULT_LED);

    while (1) {
        // Allow updates in the event of a fault
        updater_loop();

        /*
         * Continue senging CAN messages
         */
        if (send_can) {
            can_send_air_control_critical_alt();
            send_can = false;
        }
    };
}
