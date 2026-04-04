/*
 * motor_move.h
 *
 *  Created on:
 *      Author:
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    AXIS_PAN = 0,
    AXIS_TILT,
    AXIS_ZOOM,      // future
    AXIS_FOCUS,     // future
    AXIS_COUNT
} motor_axis_t;

typedef enum {
    DIR_POSITIVE = 0,   // CW / Right / Up / In
    DIR_NEGATIVE        // CCW / Left / Down / Out
} motor_dir_t;

typedef enum {
    MOTOR_IDLE = 0,
    MOTOR_ACCEL,
    MOTOR_CRUISE,
    MOTOR_DECEL,
	MOTOR_STOPPING
} motor_state_t;

typedef enum {
    HOME_IDLE = 0,
    HOME_FULL_SCAN,       // pan: rotating full 360 CW
    HOME_MARK_FOUND,      // IR triggered, record position
    HOME_RETURN,          // shortest path back
    HOME_SLOW_APPROACH,   // fine positioning
    HOME_LOCKED,          // step counter zeroed
    HOME_TIMEOUT,
    HOME_ERROR
} home_state_t;

typedef struct {
    motor_state_t   state;
    motor_dir_t     direction;

    volatile int32_t  abs_position;     // absolute step count (signed)
    int32_t           target_position;  // for position moves

    uint32_t current_freq_hz;
    uint32_t target_freq_hz;
    uint32_t min_freq_hz;
    uint32_t max_freq_hz;
    uint32_t ramp_step_hz;              // freq change per ramp tick

    bool     limit_min_hit;
    bool     limit_max_hit;
    bool     homed;

    // Filled by BSP layer — axis need not know about GPIO
    bool     (*read_sensor_min)(void);  // returns true if triggered
    bool     (*read_sensor_max)(void);
    void     (*set_dir_pin)(motor_dir_t);
    void     (*enable_step_timer)(uint32_t freq_hz);
    void     (*disable_step_timer)(void);
} motor_ctrl_t;

// Motor array — all axes
extern motor_ctrl_t motors[AXIS_COUNT];
extern motor_ctrl_t pan_motor;
extern motor_ctrl_t tilt_motor;
// Homing done/error flags — read by main loop
extern volatile uint8_t pan_home_done_flag;
extern volatile uint8_t pan_home_error_flag;
extern volatile uint8_t tilt_home_done_flag;
extern volatile uint8_t tilt_home_error_flag;

// BSP init — call once from main() after MX inits
void motor_bsp_init(void);

// tilt sensor helpers (already in .c, expose for home_cmd use)
bool tilt_down_blocked(void);
bool tilt_up_blocked(void);

void motor_pan_update_ramp(void);
void motor_tilt_update_ramp(void);

#define START_FREQ_HZ 10   // simulate 0 Hz by starting low
#define START_DUTY 100.0f
#define END_DUTY 25.0f
#define RAMP_STEPS 200

extern  uint8_t direction;
extern volatile int32_t right_current_step;	// unified signed pan position
extern volatile int32_t left_current_step;	// kept for build compat, do not use for logic
extern  uint32_t speed_hz;

extern float current_step_pan;
extern float current_step_tilt;
#define TOTAL_RAMP_STEPS 100.0f


/* Function declaration of move and stop for pan and tilt motion -----------------------*/
void move_pan(uint8_t direction, int TARGET_FREQ_HZ);
void move_pan_for_step_calib(uint8_t direction, int TARGET_FREQ_HZ);
void move_tilt(int TARGET_FREQ_HZ);
void stop_pan(int TARGET_FREQ_HZ);
void stop_tilt(int TARGET_FREQ_HZ);
void process_home_command(void);
void move_one_step(uint8_t direction);
void stop_all_motion(void);
//void ptz_execute_intent(void);

#endif /* INC_MOTOR_CONTROL_H_ */
