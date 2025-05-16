/* Copyright 2018-2021 James Laird-Wah, Islam Sharabash
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "matrix.h"
#include "i2c_master.h"
#include "wait.h"
#include <string.h>
#include "wire-protocol-constants.h"
#include "print.h"
#include "debug.h"
#include "timer.h"
#include <hal.h>
#include "gpio.h"

// shifting << 1 is because drivers/chibios/i2c_master.h expects the address
// shifted.
// 0x58 and 0x59 are the addresses defined in dygma/raise/Hand.h
#define I2C_ADDR_LEFT (0x58 << 1)
#define I2C_ADDR_RIGHT (0x59 << 1)
#define I2C_ADDR(hand) ((hand) ? I2C_ADDR_RIGHT : I2C_ADDR_LEFT)
#define LEFT 0
#define RIGHT 1
#define WATCHDOG_MS 50
#define CONSECUTIVE_HAND_READ_TIMEOUT_LIMIT 2
/* If no key events have occurred, the scanners will time out on reads.
 * So we don't want to be too permissive here. */
// TODO(ibash) not convinced this is needed...
#define MY_I2C_TIMEOUT 10
#define ROWS_PER_HAND (MATRIX_ROWS / 2)

#define ERR_TRIGGER_PIN A13

typedef enum { CHANGED, OFFLINE, UNCHANGED } read_hand_t;

static read_hand_t last_state[2] = {OFFLINE, OFFLINE};
static bool matrix_was_reset[2] = {false, false};
static int consecutive_hand_read_timeouts = 0;
static int consecutive_hand_offline_count[2] = {0,0};
static bool i2c_pins_were_reset = false;
static int debug_pin_reset_count = 0;
static int debug_i2c_read_timeout_count[2] = {0,0};
static int debug_i2c_read_error_count[2] = {0,0};

void debug_print_mike_data(void) {
    dprintf("MIKE DEBUG:\npin_reset_count=%d\nleft_read_timeout=%d\nright_read_timeout=%d\nleft_read_error=%d\nright_read_error=%d\n", debug_pin_reset_count, debug_i2c_read_timeout_count[LEFT], debug_i2c_read_timeout_count[RIGHT], debug_i2c_read_error_count[LEFT], debug_i2c_read_error_count[RIGHT]);
}

void debug_reset_mike_data(void) {
    debug_print_mike_data();
    dprintf("resetting counts\n");
    debug_pin_reset_count = 0;
    debug_i2c_read_timeout_count[LEFT] = 0;
    debug_i2c_read_timeout_count[RIGHT] = 0;
    debug_i2c_read_error_count[LEFT] = 0;
    debug_i2c_read_error_count[RIGHT] = 0;
}

void reset_i2c_pins(void) {
    debug_pin_reset_count++;
    // From : https://github.com/Dygmalab/Kaleidoscope/blob/b3df553c0af8db6ba63475b4d49b56d2adad6d51/src/kaleidoscope/device/dygma/raise/TWI.cpp#L85-L112
    //try i2c bus recovery at 100kHz = 5uS high, 5uS low
    palClearLine(ERR_TRIGGER_PIN);
    palSetLineMode(I2C1_SDA_PIN, PAL_MODE_OUTPUT_OPENDRAIN);
    palSetLine(I2C1_SDA_PIN); //keeping SDA high during recovery
    palSetLineMode(I2C1_SCL_PIN, PAL_MODE_OUTPUT_OPENDRAIN);
    dprintf("RESETTING I2C PINS!\n");

    for (int i = 0; i < 10; i++) {
        palSetLine(I2C1_SCL_PIN);
        wait_us(5);
        palClearLine(I2C1_SCL_PIN);
        wait_us(5);
    }

    //a STOP signal (SDA from low to high while CLK is high)
    palClearLine(I2C1_SDA_PIN);
    wait_us(5);
    palSetLine(I2C1_SCL_PIN);
    wait_us(2);
    palSetLine(I2C1_SDA_PIN);
    wait_us(2);

    // The following wait just makes it easier to find things in the logic
    // wait_us(5000);
    dprintf("Restoring pins back to i2c mode...\n");

    palSetLineMode(I2C1_SCL_PIN, PAL_MODE_ALTERNATE(I2C1_SCL_PAL_MODE) | PAL_OUTPUT_TYPE_OPENDRAIN);
    palSetLineMode(I2C1_SDA_PIN, PAL_MODE_ALTERNATE(I2C1_SDA_PAL_MODE) | PAL_OUTPUT_TYPE_OPENDRAIN);
    palSetLine(ERR_TRIGGER_PIN);
    consecutive_hand_read_timeouts = 0;
    i2c_pins_were_reset = true;
}

static read_hand_t i2c_read_hand(int hand, matrix_row_t current_matrix[]) {
    // dygma raise firmware says online is true iff we get the number of
    // expected bytes (e.g. 6 bytes or ROWS_PER_HAND + 1).
    // In the case where no keys are pressed the keyscanner will send the same 0
    // byte over and over. -- so this case is set.
    //
    // On the stm32 side if we don't get as many bytes as expecetd the
    // i2c_receive times out -- so online can be defined as getting
    // "I2C_STATUS_SUCCESS".

    uint8_t      buf[ROWS_PER_HAND + 1];
    i2c_status_t ret = i2c_receive(I2C_ADDR(hand), buf, sizeof(buf), MY_I2C_TIMEOUT);
    if (ret == I2C_STATUS_TIMEOUT) {
        debug_i2c_read_timeout_count[hand]++;
        consecutive_hand_read_timeouts++;
    } else if (ret == I2C_STATUS_ERROR) {
        debug_i2c_read_error_count[hand]++;
    } else {
        consecutive_hand_read_timeouts = 0;
    }

    if (ret != I2C_STATUS_SUCCESS) {
        return OFFLINE;
    }

    if (i2c_pins_were_reset) {
        dprintf("Successful read after pin reset!\n");
        i2c_pins_were_reset = false;
    }

    if (buf[0] != TWI_REPLY_KEYDATA) {
        return UNCHANGED;
    }


    int           start_row = hand ? ROWS_PER_HAND : 0;
    matrix_row_t *out       = &current_matrix[start_row];
    memcpy(out, &buf[1], ROWS_PER_HAND);

    return CHANGED;
}

static i2c_status_t i2c_set_keyscan_interval(int hand, int delay) {
    uint8_t      buf[] = {TWI_CMD_KEYSCAN_INTERVAL, delay};
    return i2c_transmit(I2C_ADDR(hand), buf, sizeof(buf), MY_I2C_TIMEOUT);
}


/*
static int sendAndRecv(int hand, uint8_t cmd, uint8_t *rxBuf) {
    uint8_t    buf[] = {cmd};
    i2c_status_t ret = i2c_transmit(I2C_ADDR(hand), buf, sizeof(buf), 100);
    if (ret == I2C_STATUS_ERROR) {
        return -1;
    } else if (ret == I2C_STATUS_TIMEOUT) {
        return -2;
    }
    wait_us(40);
    ret = i2c_receive(I2C_ADDR(hand), rxBuf, sizeof(rxBuf), 100);
    if (ret == I2C_STATUS_ERROR) {
        return -10;
    } else if (ret == I2C_STATUS_TIMEOUT) {
        return -20;
    }
    return I2C_STATUS_SUCCESS;
}

static int readReg(int hand, uint8_t cmd) {
    uint8_t in[1];
    int res = sendAndRecv(hand, cmd, in);
    if (res != I2C_STATUS_SUCCESS) {
        return res;
    }

    return in[0];
}
*/

/*
static int i2c_read_layout(int hand) {
    return readReg(hand, TWI_CMD_LAYOUT);
}

static int i2c_get_keyscan_interval(int hand) {
    return readReg(hand, TWI_CMD_KEYSCAN_INTERVAL);
}
    */


static uint32_t last_reinit           = 0;

void reinit_matrix(void) {
    last_reinit = timer_read32();
    // consecutive_hand_offline_count[LEFT] = 0;
    // consecutive_hand_offline_count[RIGHT] = 0;
    // matrix_was_reset[LEFT] = false;
    // matrix_was_reset[RIGHT] = false;
    // ref: https://github.com/Dygmalab/Kaleidoscope/blob/7bac53de106c42ffda889e6854abc06cf43a3c6f/src/kaleidoscope/device/dygma/Raise.cpp#L83
    // ref: https://github.com/Dygmalab/Kaleidoscope/blob/7bac53de106c42ffda889e6854abc06cf43a3c6f/src/kaleidoscope/device/dygma/raise/Hand.cpp#L73
    int leftSetInterval = i2c_set_keyscan_interval(LEFT, 50);
    wait_us(10);
    int rightSetInterval = i2c_set_keyscan_interval(RIGHT, 50);
    dprintf("reinit_matrix: setInterval res: left=%d, right=%d\n", leftSetInterval, rightSetInterval);
}

void matrix_init_custom(void) {
    dprintf("matrix_init_custom...\n");
    i2c_init();

    palSetLineMode(ERR_TRIGGER_PIN, PAL_MODE_OUTPUT_PUSHPULL);

    palSetLine(ERR_TRIGGER_PIN);

    reinit_matrix();
}

void reset_matrix_for_hand(int hand, matrix_row_t current_matrix[]) {
    memset(&current_matrix[hand ? ROWS_PER_HAND : 0], 0, ROWS_PER_HAND);
    // // Nuke current state for the affected hand. This ensures that a held-down key
    // // on a hand that becomes "offline" doesn't result in it appearing to be held down.
    // for (int i = start_row; i < start_row + ROWS_PER_HAND; i++) {
    //     current_matrix[i] = 0;
    // }
}

bool matrix_scan_custom(matrix_row_t current_matrix[]) {
    // HACK(ibash) without the delay between the two calls to i2c_read_hand, the
    // second call to i2c_read_hand breaks. I observed that the i2s start isn't
    // sent, or maybe it is, but the address matcher in the attiny can't recognize
    // it. In any case, a short delay fixes it.
    read_hand_t left_state = i2c_read_hand(LEFT, current_matrix);
    wait_us(20);
    read_hand_t right_state = i2c_read_hand(RIGHT, current_matrix);

    bool force_matrix_has_changed = false;

    if (left_state == OFFLINE) {
        if (last_state[LEFT] == OFFLINE) {
            consecutive_hand_offline_count[LEFT]++;
        } else {
            dprintf("left just went offline.\n");
         //   wait_us(5000);
        }
    }

    if (right_state == OFFLINE) {
        if (last_state[RIGHT] == OFFLINE) {
            consecutive_hand_offline_count[RIGHT]++;
        } else {
            dprintf("right just went offline.\n");
         //   wait_us(5000);
        }
    }

    if (consecutive_hand_read_timeouts > CONSECUTIVE_HAND_READ_TIMEOUT_LIMIT) {
        dprintf("Exceeded consecutive read timeout limit (%d). Resetting pins.\n", CONSECUTIVE_HAND_READ_TIMEOUT_LIMIT);
        reset_i2c_pins();
        wait_us(20);
        reinit_matrix();
    }

    if (last_state[LEFT] == OFFLINE && consecutive_hand_offline_count[LEFT] == 10 && !matrix_was_reset[LEFT]) {
        dprintf("left has been offline for 10 scans in a row - resetting its matrix.\n");
        reset_matrix_for_hand(LEFT, current_matrix);
        force_matrix_has_changed = true;
        matrix_was_reset[LEFT] = true;
    }

    if (last_state[RIGHT] == OFFLINE && consecutive_hand_offline_count[RIGHT] == 10 && !matrix_was_reset[RIGHT]) {
        dprintf("right has been offline for 10 scans in a row - resetting its matrix.\n");
        reset_matrix_for_hand(RIGHT, current_matrix);
        force_matrix_has_changed = true;
        matrix_was_reset[RIGHT] = true;
    }


    if ((last_state[LEFT] == OFFLINE && left_state != OFFLINE && matrix_was_reset[LEFT]) || (last_state[RIGHT] == OFFLINE && right_state != OFFLINE && matrix_was_reset[RIGHT])) {
        uint32_t timer_now = timer_read32();

        if (TIMER_DIFF_32(timer_now, last_reinit) >= 100) {
            dprintf("matrix_scan_custom reset: left_state: %d, right_state: %d\n", left_state, right_state);
            reinit_matrix();
        }
    }
    /* else if (left_state == OFFLINE && right_state == OFFLINE && left_matrix_was_reset && right_matrix_was_reset && !i2c_pins_were_reset) {
        reset_i2c_pins();
        i2c_pins_were_reset = true;
        reinit_matrix();
    }*/

    if (left_state != OFFLINE) {
        // if (consecutive_hand_offline_count[LEFT] > 0) {
        //     dprintf("Left side came back online after %d scans.\n", consecutive_hand_offline_count[LEFT]);
        // }
        consecutive_hand_offline_count[LEFT] = 0;
        matrix_was_reset[LEFT] = false;
    }

    if (right_state != OFFLINE) {
        // if (consecutive_hand_offline_count[RIGHT] > 0) {
        //     dprintf("Right side came back online after %d scans.\n", consecutive_hand_offline_count[RIGHT]);
        // }
        consecutive_hand_offline_count[RIGHT] = 0;
        matrix_was_reset[RIGHT] = false;
    }

    last_state[LEFT]  = left_state;
    last_state[RIGHT] = right_state;

    bool matrix_has_changed = left_state == CHANGED || right_state == CHANGED || force_matrix_has_changed;

    return matrix_has_changed;
}
