// MIT License

// Copyright (c) 2018 Joshua Riddell

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "cbots.h"

#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/binary.h"
#include "mpconfigport.h"

#include "machine_hw_i2c.h"
#include "platform.h"

#include <math.h>
#include <stdio.h>

//--------------------------------------------------------------

#define PCA_SLAVE_ADDR 0x40

//--------------------------------------------------------------

static const float max_angle = 1.5708;
static const float rad2off = 200 / (M_PI / 2);

static float current_pos[NUM_SERVOS] = {0};
static float zero_pos[NUM_SERVOS] = {0};

static mp_machine_i2c_obj_t *i2c_obj;

//--------------------------------------------------------------

// set_servo takes servo index and angle in radians and writes the angle to the servo
static inline void set_servo_raw(uint8_t idx, float angle) {
    // check we are not out of the reasonable range
    if (abs(angle) > max_angle) {
        printf("servo index: %d, out of range\n", idx);
        return;
    }

    // record current position
    current_pos[idx] = angle;

    // calculate on and off timestamp for PCA chip
    uint16_t on_time = idx * 200;
    uint16_t off_time = (idx * 200) + 340 - (mp_int_t)(angle * rad2off);

    // add numbers to i2c buffer
    uint8_t buff[4];
    *(uint16_t *)buff = on_time;
    *(uint16_t *)(buff + 2) = off_time;

    // work out register address for the servo at this index
    uint8_t addr = 0x06 + idx * 4;

    // write to i2c
    mp_i2c_master_write(i2c_obj, PCA_SLAVE_ADDR, 1, addr, buff, 4, true);
}

//--------------------------------------------------------------

// set_servo takes servo index and angle in radians. It offsets the angle
//              to centre the value then writes it to the servo
void set_servo(uint8_t idx, float angle) {
    // apply zero position offsets
    angle += zero_pos[idx];

    // set servo angle with centred value
    set_servo_raw(idx, angle);    
}

//--------------------------------------------------------------

// saves the already initialised i2c object for later
STATIC mp_obj_t cbots_set_i2c(mp_obj_t i2c) {
    i2c_obj = i2c;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cbots_set_i2c_obj, cbots_set_i2c);

//--------------------------------------------------------------

// set servo calibration from passed mp list
STATIC mp_obj_t cbots_set_servo_zero_pos(mp_obj_t zero_pos_list) {
    mp_obj_t *list_items;

    mp_obj_get_array_fixed_n(zero_pos_list, NUM_SERVOS, &list_items);

    // unpack list
    for (uint8_t i = 0; i < NUM_SERVOS; ++i) {
        zero_pos[i] = mp_obj_get_float(list_items[i]);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cbots_set_servo_zero_pos_obj, cbots_set_servo_zero_pos);

//--------------------------------------------------------------

// return tuple of all servo positions
STATIC mp_obj_t cbots_servo_get_all() {
    mp_obj_t angles[NUM_SERVOS];

    for (uint8_t i = 0; i < NUM_SERVOS; ++i) {
        angles[i] = mp_obj_new_float(current_pos[i]);
    }

    return mp_obj_new_tuple(NUM_SERVOS, angles);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(cbots_servo_get_all_obj, cbots_servo_get_all);

//--------------------------------------------------------------

// get angle of servo index
STATIC mp_obj_t cbots_servo_get_rad(mp_obj_t servo_index) {
    uint8_t idx = mp_obj_get_int(servo_index);
    return mp_obj_new_float(current_pos[idx]);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cbots_servo_get_rad_obj, cbots_servo_get_rad);

//--------------------------------------------------------------

// get angle of servo index
STATIC mp_obj_t cbots_servo_get_deg(mp_obj_t servo_index) {
    uint8_t idx = mp_obj_get_int(servo_index);
    return mp_obj_new_float(current_pos[idx] * 180 / M_PI);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cbots_servo_get_deg_obj, cbots_servo_get_deg);

//--------------------------------------------------------------

// set servo degrees without centre offsetting
STATIC mp_obj_t cbots_servo_set_deg_raw(mp_obj_t servo_index, mp_obj_t angle) {
    mp_int_t idx = mp_obj_get_int(servo_index);
    mp_float_t a_rad = mp_obj_get_float(angle);

    set_servo_raw(idx, a_rad / M_PI * 180);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(cbots_servo_set_deg_raw_obj, cbots_servo_set_deg_raw);

//--------------------------------------------------------------

// set servo radians without centre offsetting
STATIC mp_obj_t cbots_servo_set_rad_raw(mp_obj_t servo_index, mp_obj_t angle) {
    mp_int_t idx = mp_obj_get_int(servo_index);
    mp_float_t a_rad = mp_obj_get_float(angle);

    set_servo_raw(idx, a_rad);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(cbots_servo_set_rad_raw_obj, cbots_servo_set_rad_raw);

//--------------------------------------------------------------

// set servo radians
STATIC mp_obj_t cbots_servo_set_rad(mp_obj_t servo_index, mp_obj_t angle) {
    mp_int_t idx = mp_obj_get_int(servo_index);
    mp_float_t a_rad = mp_obj_get_float(angle);

    set_servo(idx, a_rad);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(cbots_servo_set_rad_obj, cbots_servo_set_rad);

//--------------------------------------------------------------

// set servo degrees
STATIC mp_obj_t cbots_servo_set_deg(mp_obj_t servo_index, mp_obj_t angle) {
    mp_int_t idx = mp_obj_get_int(servo_index);
    mp_float_t a_rad = mp_obj_get_float(angle);

    set_servo(idx, a_rad / M_PI * 180);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(cbots_servo_set_deg_obj, cbots_servo_set_deg);

//--------------------------------------------------------------

STATIC const mp_map_elem_t cbots_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_cbots) },

    { MP_OBJ_NEW_QSTR(MP_QSTR_set_i2c), (mp_obj_t)&cbots_set_i2c_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_set_servo_zero_pos), (mp_obj_t)&cbots_set_servo_zero_pos_obj },

    { MP_OBJ_NEW_QSTR(MP_QSTR_servo_get_all), (mp_obj_t)&cbots_servo_get_all_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_servo_get_rad), (mp_obj_t)&cbots_servo_get_rad_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_servo_get_deg), (mp_obj_t)&cbots_servo_get_deg_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_servo_set_deg_raw), (mp_obj_t)&cbots_servo_set_deg_raw_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_servo_set_rad_raw), (mp_obj_t)&cbots_servo_set_rad_raw_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_servo_set_rad), (mp_obj_t)&cbots_servo_set_rad_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_servo_set_deg), (mp_obj_t)&cbots_servo_set_deg_obj },
};

STATIC MP_DEFINE_CONST_DICT (
    mp_module_cbots_globals,
    cbots_globals_table
);

const mp_obj_module_t mp_module_cbots = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_cbots_globals,
};
