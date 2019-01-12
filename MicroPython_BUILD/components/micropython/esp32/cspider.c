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

#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/binary.h"
#include "mpconfigport.h"

#include "platform.h"
#include "cbots.h"

#include <math.h>
#include <stdio.h>

//--------------------------------------------------------------

#define NUM_LEGS 4

// leg component lengths
#define L1   25
#define L2   48
#define L3   75
#define L4   10  // offset of foot to leg axis

// body dimensions
#define BW   47
#define BL   88

//--------------------------------------------------------------

static const int16_t body_offsets[NUM_LEGS][2] = {
    {BW/2,  -BL/2},
    {BW/2,  BL/2},
    {-BW/2, BL/2},
    {-BW/2, -BL/2},
};

static const int8_t angle_signs[NUM_SERVOS] = {
    -1, -1,  -1,  0,
    1,  1,  1,  0,
    -1,  -1,  -1,  0,
    1,  1,  1,  0,
};

//--------------------------------------------------------------

// these are set via the python function cspider_set_legs0
static float legs0[NUM_LEGS][3];
static float legs[NUM_LEGS][3];

//--------------------------------------------------------------

static float walk_tn = 0;  // current time in walk

// variables storing state of body and walking
static float body_x, body_y, body_z, body_roll, body_pitch, body_yaw;
static float walk_x_rate, walk_y_rate, walk_yaw_rate;

// walk running parameters
static float dt = 0.1;

typedef enum {
    STATE_STEP_IDLE,
    STATE_STEP_MOVING,
    STATE_STEP_STEPPING,
} step_state_t;

static step_state_t step_state;
static int8_t step_leg;
static float step_state_t0;
static float step_last_tn;

static float move_time = 0.5;
static float step_time = 0.5;

static float step_period = 1;
static float step_thresh = 40;

static float support_scale_factor = 0.8;

//--------------------------------------------------------------

// rot2d rotates the point x,y about the origin by a radians
void rot2d(float a, float *x, float *y) {
    // calculate trig functions
    float sa = sin(a);
    float ca = cos(a);
    
    // calculate coordinates
    *x = *x * ca - *y * sa;
    *y = *x * sa + *y * ca;
}

//--------------------------------------------------------------

// body_to_leg converts the leg coordinate x,y,z from body frame to local
//              leg frame given the index of the leg. It applies the body
//              frame rotations and translations.
void body_to_leg(uint8_t idx, float *x, float *y, float *z) {
    // load the leg coordinates and apply body offset
    *x = legs[idx][0] - body_x;
    *y = legs[idx][1] - body_y;
    *z = legs[idx][2] - body_z;

    // apply body rotations
    rot2d(body_roll, x, z);
    rot2d(body_pitch, z, y);
    rot2d(-body_yaw, x, y);

    // apply offsets due to mechanical offsets of leg
    *x -= body_offsets[idx][0];
    *y -= body_offsets[idx][1];
}

//--------------------------------------------------------------

// leg_ik converts the local leg coordinate x,y,z into a triple of angles
//          to be sent to the leg servos.
void leg_ik(float x, float y, float z,
            float *t1_ret, float *t2_ret, float *t3_ret) {
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

//--------------------------------------------------------------

// set_leg sets the servos in the leg such that the tip of the leg is at the
//          local coordinate x,y,z.
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

//--------------------------------------------------------------

// update_body updates all the legs in the body.
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

//--------------------------------------------------------------

void get_centroid(float *x, float *y) {
    float x_sum = 0;
    float y_sum = 0;

    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
        if (i == step_leg) {
            continue;
        }

        x_sum += legs[i][0];
        y_sum += legs[i][1];
    }

    *x = x_sum / 3;
    *y = y_sum / 3;
}

//--------------------------------------------------------------

int8_t get_next_leg_index(void) {
    float max_leg_diff = 0;
    uint8_t max_leg_diff_idx = 0;

    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
        float r = powf(powf(legs[i][0] - legs0[i][0], 2.) + powf(legs[i][1] - legs0[i][1], 2.), 0.5);

        if (r > max_leg_diff) {
            max_leg_diff = r;
            max_leg_diff_idx = i;
        }
    }

    if (max_leg_diff < step_thresh) {
        return -1;
    }

    return max_leg_diff_idx;
}

//--------------------------------------------------------------

inline float sign(float x1, float y1, float x2, float y2, float x3, float y3) {
    return (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1- y3);
}

//--------------------------------------------------------------

bool cg_in_legs(float centroid_x, float centroid_y) {
    float support_base_points[NUM_LEGS - 1][2];
    uint8_t idx = 0;

    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
        if (i == step_leg) {
            continue;
        }

        support_base_points[idx][0] = centroid_x + support_scale_factor * (centroid_x - legs[i][0]);
        support_base_points[idx][1] = centroid_y + support_scale_factor * (centroid_y - legs[i][1]);

        ++idx;
    }

    float d1 = sign(body_x, body_y, support_base_points[0][0], support_base_points[0][1], support_base_points[1][0], support_base_points[1][1]);
    float d2 = sign(body_x, body_y, support_base_points[1][0], support_base_points[1][1], support_base_points[2][0], support_base_points[2][1]);
    float d3 = sign(body_x, body_y, support_base_points[2][0], support_base_points[2][1], support_base_points[0][0], support_base_points[0][1]);

    bool has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    bool has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    return !(has_neg && has_pos);
}

//--------------------------------------------------------------

// update_walk periodically updates the body for walking timesteps.
void update_walk(void) {
    walk_tn += dt;

    float state_tn = walk_tn - step_state_t0;

    float z, centroid_x, centroid_y;

    switch (step_state) {
        case STATE_STEP_IDLE:
            if (walk_tn - step_last_tn >= step_period) {
                step_leg = get_next_leg_index();

                if (step_leg != -1) {
                    step_last_tn = walk_tn;
                    step_state = STATE_STEP_MOVING;
                    step_state_t0 = walk_tn;
                }
            }

            // update body position to move towards the centre
            if (abs(body_x) < 1) {
                body_x = 0;
            }

            if (abs(body_y) < 1) {
                body_y = 0;
            }

            body_x -= body_x * (state_tn / move_time);
            body_y -= body_y * (state_tn / move_time);

            break;
        case STATE_STEP_MOVING:
            get_centroid(&centroid_x, &centroid_y);

            // update body position to move towards centroid
            body_x += (centroid_x - body_x) * (state_tn / move_time);
            body_y += (centroid_y - body_y) * (state_tn / move_time);

            if (cg_in_legs(centroid_x, centroid_y)) {
                step_state = STATE_STEP_STEPPING;
                step_state_t0 = walk_tn;
            }

            break;
        case STATE_STEP_STEPPING:
            // lift leg
            z = 70 * powf(sin(state_tn * M_PI / step_time), 2);
            legs[step_leg][2] = legs0[step_leg][2] + z;

            // if leg is high enough then reset position
            if (z > 40) {
                legs[step_leg][0] = legs0[step_leg][0];
                legs[step_leg][1] = legs0[step_leg][1];
            }

            if (state_tn >= step_time) {
                step_state = STATE_STEP_IDLE;
                step_state_t0 = walk_tn;
                legs[step_leg][2] = legs0[step_leg][2];
            }

            break;
    }

    // write leg positions
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
        // reset leg position when it is high enough
        if (legs[i][2] > 30) {
            continue;
        }

        // apply translational shift to move in the x,y directions
        legs[i][0] -= walk_x_rate * dt;
        legs[i][1] -= walk_y_rate * dt;

        // apply rotational shift to yaw in each direction
        rot2d(walk_yaw_rate*dt, &(legs[i][0]), &(legs[i][1]));
    }

    update_body();
}

//--------------------------------------------------------------

STATIC mp_obj_t cspider_update_walk_rates(mp_obj_t x_rate, mp_obj_t y_rate, mp_obj_t yaw_rate) {
    walk_x_rate = mp_obj_get_float(x_rate);
    walk_y_rate = mp_obj_get_float(y_rate);
    walk_yaw_rate = mp_obj_get_float(yaw_rate);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(cspider_update_walk_rates_obj, cspider_update_walk_rates);

//--------------------------------------------------------------

STATIC mp_obj_t cspider_update_walk() {
    update_walk();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(cspider_update_walk_obj, cspider_update_walk);

//--------------------------------------------------------------

STATIC mp_obj_t cspider_set_legs0(mp_obj_t legs0_list) {
    mp_obj_t *leg_coordinates;
    mp_obj_t *coordinate;

    mp_obj_get_array_fixed_n(legs0_list, NUM_LEGS, &leg_coordinates);

    // // unpack each leg
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
        // get xyz coordinate
        mp_obj_get_array_fixed_n(leg_coordinates[i], 3, &coordinate);

        // load in each float
        for (uint8_t j = 0; j < 3; ++j) {
            float val = mp_obj_get_float(coordinate[j]);

            // save this as the base coordinate value
            legs0[i][j] = val;
            
            // also reset the running leg position while we're here
            legs[i][j] = val;
        }
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cspider_set_legs0_obj, cspider_set_legs0);

//--------------------------------------------------------------

STATIC mp_obj_t cspider_begin_walk(size_t n_args, const mp_obj_t *args) {
    walk_tn = 0;

    step_state_t0 = 0;
    step_state = STATE_STEP_IDLE;
    step_leg = 0;
    step_last_tn = -100;

    dt = mp_obj_get_float(args[0]);

    step_time = mp_obj_get_float(args[1]);
    move_time = mp_obj_get_float(args[2]);
    step_period = mp_obj_get_float(args[3]);
    step_thresh = mp_obj_get_float(args[4]);
    support_scale_factor = mp_obj_get_float(args[5]);

    walk_x_rate = 0;
    walk_y_rate = 0;
    walk_yaw_rate = 0;

    update_body();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cspider_begin_walk_obj, 6, 6, cspider_begin_walk);

//--------------------------------------------------------------

STATIC mp_obj_t cspider_end_walk(void) {
    body_x = 0;
    body_y = 0;

    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
        for (uint8_t j = 0; j < 3; ++j) {
            legs[i][j] = legs0[i][j];
        }
    }

    update_body();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(cspider_end_walk_obj, cspider_end_walk);

//--------------------------------------------------------------

STATIC mp_obj_t cspider_xyz(mp_obj_t x, mp_obj_t y, mp_obj_t z) {
    body_x = mp_obj_get_float(x);
    body_y = mp_obj_get_float(y);
    body_z = mp_obj_get_float(z);

    update_body();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(cspider_xyz_obj, cspider_xyz);

//--------------------------------------------------------------

STATIC mp_obj_t cspider_rpy(mp_obj_t roll, mp_obj_t pitch, mp_obj_t yaw) {
    body_roll = mp_obj_get_float(roll);
    body_pitch = mp_obj_get_float(pitch);
    body_yaw = mp_obj_get_float(yaw);
    
    update_body();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(cspider_rpy_obj, cspider_rpy);

//--------------------------------------------------------------

STATIC mp_obj_t cspider_xyzrpy(size_t n_args, const mp_obj_t *args) {
    body_x = mp_obj_get_float(args[0]);
    body_y = mp_obj_get_float(args[1]);
    body_z = mp_obj_get_float(args[2]);

    body_roll = mp_obj_get_float(args[3]);
    body_pitch = mp_obj_get_float(args[4]);
    body_yaw = mp_obj_get_float(args[5]);

    update_body();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cspider_xyzrpy_obj, 6, 6, cspider_xyzrpy);

//--------------------------------------------------------------

STATIC mp_obj_t cspider_update_body(void) {
    update_body();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(cspider_update_body_obj, cspider_update_body);

//--------------------------------------------------------------

STATIC mp_obj_t cspider_set_x(mp_obj_t x) {
    body_x = mp_obj_get_float(x);
    update_body();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cspider_set_x_obj, cspider_set_x);

//--------------------------------------------------------------

STATIC mp_obj_t cspider_set_y(mp_obj_t y) {
    body_y = mp_obj_get_float(y);
    update_body();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cspider_set_y_obj, cspider_set_y);

//--------------------------------------------------------------

STATIC mp_obj_t cspider_set_z(mp_obj_t z) {
    body_z = mp_obj_get_float(z);
    update_body();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cspider_set_z_obj, cspider_set_z);

//--------------------------------------------------------------

STATIC mp_obj_t cspider_set_roll(mp_obj_t roll) {
    body_roll = mp_obj_get_float(roll);
    update_body();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cspider_set_roll_obj, cspider_set_roll);

//--------------------------------------------------------------

STATIC mp_obj_t cspider_set_pitch(mp_obj_t pitch) {
    body_pitch = mp_obj_get_float(pitch);
    update_body();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cspider_set_pitch_obj, cspider_set_pitch);

//--------------------------------------------------------------

STATIC mp_obj_t cspider_set_yaw(mp_obj_t yaw) {
    body_yaw = mp_obj_get_float(yaw);
    update_body();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cspider_set_yaw_obj, cspider_set_yaw);

//--------------------------------------------------------------

STATIC const mp_map_elem_t cspider_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_cspider) },

    // walk functions
    { MP_OBJ_NEW_QSTR(MP_QSTR_update_walk_rates), (mp_obj_t)&cspider_update_walk_rates_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_update_walk), (mp_obj_t)&cspider_update_walk_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_set_legs0), (mp_obj_t)&cspider_set_legs0_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_begin_walk), (mp_obj_t)&cspider_begin_walk_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_end_walk), (mp_obj_t)&cspider_end_walk_obj },

    // body positioning functions
    { MP_OBJ_NEW_QSTR(MP_QSTR_xyz), (mp_obj_t)&cspider_xyz_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_rpy), (mp_obj_t)&cspider_rpy_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_xyzrpy), (mp_obj_t)&cspider_xyzrpy_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_update_body), (mp_obj_t)&cspider_update_body_obj },

    // single variable functions
    { MP_OBJ_NEW_QSTR(MP_QSTR_set_x), (mp_obj_t)&cspider_set_x_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_set_y), (mp_obj_t)&cspider_set_y_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_set_z), (mp_obj_t)&cspider_set_z_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_set_roll), (mp_obj_t)&cspider_set_roll_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_set_pitch), (mp_obj_t)&cspider_set_pitch_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_set_yaw), (mp_obj_t)&cspider_set_yaw_obj },
};

STATIC MP_DEFINE_CONST_DICT (
    mp_module_cspider_globals,
    cspider_globals_table
);

const mp_obj_module_t mp_module_cspider = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_cspider_globals,
};

