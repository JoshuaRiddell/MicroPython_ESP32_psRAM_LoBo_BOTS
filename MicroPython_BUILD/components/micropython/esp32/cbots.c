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
#include "platform.h"

#include <math.h>
#include <stdio.h>

#define NUM_LEGS 4
#define NUM_SERVOS 16
#define PCA_SLAVE 0x40

#define L1   25
#define L2   48
#define L3   75
#define L4   10  // offset of foot to leg axis

#define BW   47
#define BL   88

static const float max_angle = 1.5708;
static const float rad2off = 200 / (M_PI / 2);

static const float zero_pos[NUM_SERVOS] = {
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

static const float legs0[NUM_LEGS][3] = {
    {90, -70, -35},
    {90, 70, -35},
    {-90, 70, -35},
    {-90, -70, -35},
};

static float legs[NUM_LEGS][3] = {
    {90, -70, -35},
    {90, 70, -35},
    {-90, 70, -35},
    {-90, -70, -35},
};

static int16_t body_offsets[NUM_LEGS][2] = {
    {BW/2,  -BL/2},
    {BW/2,  BL/2},
    {-BW/2, BL/2},
    {-BW/2, -BL/2},
};

static int8_t angle_signs[NUM_SERVOS] = {
    -1, -1,  -1,  0,
    1,  1,  1,  0,
    -1,  -1,  -1,  0,
    1,  1,  1,  0,
};



static float current_pos[NUM_SERVOS] = {0};

static mp_machine_i2c_obj_t *i2c_obj;

static float walk_t0 = 0;  // time at beginning of walk
static float walk_tn = 0;  // current time in walk
static float dt = 0.1;
static float leg_freq = 0.5;

static float body_x, body_y, body_z, body_roll, body_pitch, body_yaw;

#define LEAN_AMOUNT 15

// takes servo index and angle in radians
void set_servo(uint8_t idx, float angle) {
    angle += zero_pos[idx];

    if (abs(angle) > max_angle) {
        printf("servo index: %d, out of range\n", idx);
        return;
    }

    current_pos[idx] = angle;

    uint16_t on_time = idx * 200;
    uint16_t off_time = (idx * 200) + 340 - (mp_int_t)(angle * rad2off);

    uint8_t buff[4];
    *(uint16_t *)buff = on_time;
    *(uint16_t *)(buff + 2) = off_time;

    uint8_t addr = 0x06 + idx * 4;

    mp_i2c_master_write(i2c_obj, PCA_SLAVE, 1, addr, buff, 4, true);
}

void rot2d(float a, float *x, float *y) {
    // calculate trig functions
    float sa = sin(a);
    float ca = cos(a);
    
    // calculate coordinates
    *x = *x * ca - *y * sa;
    *y = *x * sa + *y * ca;
}

void body_to_leg(uint8_t idx, float *x, float *y, float *z) {
    // load the leg coordinates and apply body offset
    *x = legs[idx][0] - body_x;
    *y = legs[idx][1] - body_y;
    *z = legs[idx][2] - body_z;

    // apply body rotations
    rot2d(body_roll, x, z);
    rot2d(body_pitch, z, y);
    rot2d(-body_yaw, x, y);

    // apply offsets due to frame
    *x -= body_offsets[idx][0];
    *y -= body_offsets[idx][1];
}

void leg_ik(float x, float y, float z, float *t1_ret, float *t2_ret, float *t3_ret) {
    float t1, t2, t3;

    // root angle
    t1 = atan2(y, x);

    // offset from root segment
    x = x - cos(t1) * L1;
    y = y - sin(t1) * L1;

    // horiztonal and total distance left
    float r = sqrt(x * x + y * y);
    float rt = sqrt(r * r + z * z);

    // knee angle needed to cover that distance
    float tk = acos( (L2*L2 + L3*L3 - rt*rt) / (2 * L2 * L3) );
    t3 = M_PI - tk;

    // angle of depression for hip angle
    float d = atan2(z, r);
    float e = asin(L3 * sin(tk) / rt);
    t2 = e + d;

    *t1_ret = t1;
    *t2_ret = t2;
    *t3_ret = t3;
}

void set_leg(uint8_t id, float x, float y, float z) {
    float t1, t2, t3;
    
    leg_ik(x, y, z, &t1, &t2, &t3);

    uint8_t r_id = id * 4;
    set_servo(r_id, t3 * angle_signs[r_id]);
    ++r_id;
    set_servo(r_id, t2 * angle_signs[r_id]);
    ++r_id;
    set_servo(r_id, t1 * angle_signs[r_id]);
}

void update_body() {
    float x, y, z;

    body_to_leg(0, &x, &y, &z);
    set_leg(0, x, -y, z);

    body_to_leg(1, &x, &y, &z);
    set_leg(1, x, y, z);

    body_to_leg(2, &x, &y, &z);
    set_leg(2, -x, y, z);

    body_to_leg(3, &x, &y, &z);
    set_leg(3, -x, -y, z);
}

void change_body_for_walk(float t, float x_rate, float y_rate, float yaw_rate) {
    float tf = (float)t;

    // get body position in x,y
    float x = LEAN_AMOUNT * cos(tf * 2 * M_PI * leg_freq);
    float y = LEAN_AMOUNT * cos(tf * 2 * M_PI * leg_freq + M_PI/2);

    // write position to body
    body_x = x;
    body_y = y;

    // write leg positions
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
        // handle periodic lifting of legs in phase with body lean
        float z = 150 * cos(-tf * 2 * M_PI * leg_freq - 3*M_PI/4 - M_PI/2 * i) - 120;
        z = fmax(0., z);
        
        legs[i][2] = legs0[i][2] + z;

        // reset leg position when it is high enough
        if (z > 20) {
            legs[i][0] = legs0[i][0];
            legs[i][1] = legs0[i][1];
        }

        // apply translational shift to move in the x,y directions
        legs[i][0] -= x_rate * dt;
        legs[i][1] -= y_rate * dt;

        // apply rotational shift to yaw in each direction
        rot2d(yaw_rate*dt, &(legs[i][0]), &(legs[i][1]));
    }
}

void update_walk(float x_rate, float y_rate, float yaw_rate) {
    walk_tn += dt;
    float t = walk_tn - walk_t0;

    change_body_for_walk(t, x_rate, y_rate, yaw_rate);
    update_body();
}


STATIC mp_obj_t cbots_update_walk(mp_obj_t x_rate, mp_obj_t y_rate, mp_obj_t yaw_rate) {
    float x_rate_f = mp_obj_get_float(x_rate);
    float y_rate_f = mp_obj_get_float(y_rate);
    float yaw_rate_f = mp_obj_get_float(yaw_rate);

    update_walk(x_rate_f, y_rate_f, yaw_rate_f);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(cbots_update_walk_obj, cbots_update_walk);

STATIC mp_obj_t cbots_begin_walk(mp_obj_t input_dt, mp_obj_t input_freq) {
    walk_t0 = platform_tick_get_ms();
    walk_tn = walk_t0;

    dt = mp_obj_get_float(input_dt);
    leg_freq = mp_obj_get_float(input_freq);

    body_z = 50;

    update_body();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(cbots_begin_walk_obj, cbots_begin_walk);

STATIC mp_obj_t cbots_set_i2c(mp_obj_t i2c) {
    i2c_obj = i2c;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cbots_set_i2c_obj, cbots_set_i2c);

STATIC mp_obj_t cbots_set_servo_rad(mp_obj_t servo_index, mp_obj_t angle) {
    mp_int_t idx = mp_obj_get_int(servo_index);
    mp_float_t a_rad = mp_obj_get_float(angle);

    set_servo(idx, a_rad);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(cbots_set_servo_rad_obj, cbots_set_servo_rad);

STATIC const mp_map_elem_t cbots_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_cbots) },

    { MP_OBJ_NEW_QSTR(MP_QSTR_set_servo_rad), (mp_obj_t)&cbots_set_servo_rad_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_set_i2c), (mp_obj_t)&cbots_set_i2c_obj },

    // walk functions
    { MP_OBJ_NEW_QSTR(MP_QSTR_begin_walk), (mp_obj_t)&cbots_begin_walk_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_update_walk), (mp_obj_t)&cbots_update_walk_obj },
};

STATIC MP_DEFINE_CONST_DICT (
    mp_module_cbots_globals,
    cbots_globals_table
);

const mp_obj_module_t mp_module_cbots = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_cbots_globals,
};
