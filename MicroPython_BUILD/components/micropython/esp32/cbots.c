// MIT License

// Copyright (c) 2018 Out of the cBOTS and Joshua Riddell

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

#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/binary.h"
#include "mpconfigport.h"

#include "machine_hw_i2c.h"

#include <math.h>
#include <stdio.h>

#define NUM_SERVOS 16
#define PCA_SLAVE 0x40

static const float max_angle = 1.5708;
static const float rad2off = 200 / (M_PI / 2);

static const float zero_pos[] = {
    1.0471975511966,
    0.820304748437335,
    0.698131700797732,
    0.523598775598299,
    -0.401425727958696,
    -0.401425727958696,
    -0.628318530717959,
    0.15707963267949,
    0.785398163397448,
    0.994837673636768,
    0.785398163397448,
    0,
    -0.523598775598299,
    -0.820304748437335,
    -0.785398163397448,
    0.314159265358979,
};

static float current_pos[NUM_SERVOS] = {0};

mp_machine_i2c_obj_t *i2c_obj;

STATIC mp_obj_t cbots_set_i2c(mp_obj_t i2c) {
    i2c_obj = i2c;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cbots_set_i2c_obj, cbots_set_i2c);

STATIC mp_obj_t cbots_set_servo_rad(mp_obj_t servo_index, mp_obj_t angle) {
    mp_int_t idx = mp_obj_get_int(servo_index);
    mp_float_t a_rad = mp_obj_get_float(angle);

    a_rad += zero_pos[idx];

    if (abs(a_rad) > max_angle) {
        printf("servo index: %d, out of range\n", idx);
        return mp_const_false;
    }

    current_pos[idx] = a_rad;

    uint16_t on_time = idx * 200;
    uint16_t off_time = (idx * 200) + 340 - (mp_int_t)(a_rad * rad2off);

    uint8_t buff[4];
    *(uint16_t *)buff = on_time;
    *(uint16_t *)(buff + 2) = off_time;

    uint8_t addr = 0x06 + idx * 4;

    printf("set on: %d, off: %d\n", on_time, off_time);

    for (uint8_t i = 0; i < 4; ++i) {
        printf("buff %u: %u\n", i, buff[i]);
    }

    mp_i2c_master_write(i2c_obj, PCA_SLAVE, 1, addr, buff, 4, true);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(cbots_set_servo_rad_obj, cbots_set_servo_rad);


STATIC const mp_map_elem_t cbots_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_cbots) },

    { MP_OBJ_NEW_QSTR(MP_QSTR_set_servo_rad), (mp_obj_t)&cbots_set_servo_rad_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_set_i2c), (mp_obj_t)&cbots_set_i2c_obj },
};

STATIC MP_DEFINE_CONST_DICT (
    mp_module_cbots_globals,
    cbots_globals_table
);

const mp_obj_module_t mp_module_cbots = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_cbots_globals,
};
