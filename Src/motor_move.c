/*
 * motor_move.c
 *
 *
 *Timer PWM pulse fires
 *      ↓
 *HAL_TIM_PeriodElapsedCallback (ISR)
 *      ↓
 *motors[AXIS_PAN].abs_position ++ or --   ← ground truth, counts every real pulse
 *      ↓
 *right_current_step = motors[AXIS_PAN].abs_position  ← mirrored for telemetry
 *     ↓
 *main() printf reads right_current_step every 200ms  ← live display
 *
 *  Created on:
 *      Author:
 */

#include <main.h>
#include "pelco_decode.h"
#include "motor_move.h"
//#include "timer_utils.h"
//#include "uart_comm.h"
#include "home_cmd.h"
//#include "error_handler.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
/* variables used in move_pan, move_tilt, stop_pan and stop_tilt function for pan and tilt motion of stepper motors --------------------------------------*/

extern TIM_HandleTypeDef htim2;   // Pan: PA0 -> TIM2_CH1
extern TIM_HandleTypeDef htim5;   // Tilt: PA2 -> TIM5_CH3
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

// Motor instance array — indexed by motor_axis_t
motor_ctrl_t motors[AXIS_COUNT] = {0};

// Legacy named references kept for BSP init clarity
motor_ctrl_t pan_motor  = {0};
motor_ctrl_t tilt_motor = {0};

// current_direction: mirrors motors[AXIS_PAN].direction for move_pan() use
volatile uint8_t current_direction = 0;

// Legacy step mirrors — written by ISR, read by telemetry and home_cmd
extern volatile int32_t right_current_step;
extern volatile int32_t left_current_step;
extern volatile int32_t tilt_current_step;

// Ramp position trackers used by move_pan / move_tilt (software ramp)
// These track ramp progress only — NOT absolute position
float current_step_pan  = 0.0f;
float current_step_tilt = 0.0f;
#define TOTAL_RAMP_STEPS  100.0f   // ramp completes over this many move() calls

#define PAN_TIMER       htim2
#define PAN_CHANNEL     TIM_CHANNEL_1
#define TILT_TIMER      htim5
#define TILT_CHANNEL    TIM_CHANNEL_3

#define PAN_DIR_GPIO_Port   GPIOA
#define PAN_DIR_Pin         GPIO_PIN_5
#define TILT_DIR_GPIO_Port  GPIOA
#define TILT_DIR_Pin        GPIO_PIN_8

/**
  * @brief Process the HOME command using IR sensor at PA6 for pan motor (fast version)
  *
  * CHANGING ALL THE HTIMERx TO HTIMx
  * @retval None
  */
// ---- Homing state machine struct ----
typedef struct {
    home_state_t state;
    bool         active;
    int32_t      marker_position;   // step count when IR first triggered
    uint32_t     steps_in_state;    // counts ISR ticks in this state
    uint32_t     timeout_steps;     // max steps before declaring error
} home_sm_t;

static home_sm_t pan_home_sm  = {0};
static home_sm_t tilt_home_sm = {0};

// Flags set by homing SMs, read by main loop
volatile uint8_t pan_home_done_flag  = 0;
volatile uint8_t pan_home_error_flag = 0;
volatile uint8_t tilt_home_done_flag = 0;
volatile uint8_t tilt_home_error_flag= 0;

// Steps per full pan revolution — calibrate this for your motor/microstepping
// DRV8825 at 1/32: 200 * 32 = 6400 steps/rev
#define PAN_STEPS_PER_REV   6400U
#define PAN_HOMING_FAST_HZ  6000
#define PAN_HOMING_SLOW_HZ  1200
#define PAN_BACKOFF_STEPS    200   // steps to back off after first sensor hit
#define PAN_HOME_TIMEOUT    (PAN_STEPS_PER_REV * 2)  // 2 full revs max

// ---- Ramp engine — called from ISR, one tick per step pulse ----
static void motor_ramp_tick(motor_ctrl_t *m)
{
    if (m->state == MOTOR_ACCEL)
    {
        m->current_freq_hz += m->ramp_step_hz;
        if (m->current_freq_hz >= m->target_freq_hz)
        {
            m->current_freq_hz = m->target_freq_hz;
            m->state = MOTOR_CRUISE;
        }
        if (m->enable_step_timer)
            m->enable_step_timer(m->current_freq_hz);
    }
    else if (m->state == MOTOR_DECEL)
    {
        if (m->current_freq_hz > m->ramp_step_hz + m->target_freq_hz)
            m->current_freq_hz -= m->ramp_step_hz;
        else
            m->current_freq_hz = m->target_freq_hz;

        if (m->enable_step_timer)
            m->enable_step_timer(m->current_freq_hz);

        if (m->current_freq_hz <= m->target_freq_hz)
        {
            if (m->target_freq_hz == 0)
            {
                // Full stop
                m->state = MOTOR_IDLE;
                if (m->disable_step_timer)
                    m->disable_step_timer();
                if (m == &motors[AXIS_PAN])
                    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
                else if (m == &motors[AXIS_TILT])
                    __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_UPDATE);
            }
            else
            {
                // Reached new lower cruise speed — settle into CRUISE
                m->current_freq_hz = m->target_freq_hz;
                m->state = MOTOR_CRUISE;
            }
        }
    }
}

// ---- Pan homing SM — called from TIM2 ISR ----
static void pan_home_tick(home_sm_t *sm, motor_ctrl_t *m)
{
    sm->steps_in_state++;

    switch (sm->state)
    {
        case HOME_FULL_SCAN:
        {
            // Sensor PA6: LOW = blocked = home
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET)
            {
                sm->marker_position = m->abs_position;
                sm->state = HOME_MARK_FOUND;
                sm->steps_in_state = 0;
            }
            if (sm->steps_in_state > sm->timeout_steps)
            {
                sm->state = HOME_ERROR;
            }
            break;
        }

        case HOME_MARK_FOUND:
        {
            // Back off by PAN_BACKOFF_STEPS (motor must keep running CW briefly
            // then we switch to slow approach — handled by step count here)
            // We switch direction externally (from process_home_command state machine)
            // SM just records marker and signals main loop via flag
            pan_home_done_flag = 2;  // 2 = marker found, needs slow approach
            sm->state = HOME_RETURN;
            sm->steps_in_state = 0;
            // Stop motor — main loop will restart slow approach
            if (m->disable_step_timer)
                m->disable_step_timer();
            break;
        }

        case HOME_SLOW_APPROACH:
        {
            // Sensor triggered again at slow speed = locked
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET)
            {
                if (m->disable_step_timer)
                    m->disable_step_timer();
                __disable_irq();
                m->abs_position = 0;
                right_current_step = 0;
                left_current_step  = 0;
                __enable_irq();
                m->homed = true;
                sm->state = HOME_LOCKED;
                sm->active = false;
                pan_home_done_flag = 1;  // 1 = fully locked
            }
            if (sm->steps_in_state > PAN_BACKOFF_STEPS * 4)
            {
                sm->state = HOME_ERROR;
                pan_home_error_flag = 1;
                if (m->disable_step_timer)
                    m->disable_step_timer();
            }
            break;
        }

        case HOME_ERROR:
        case HOME_TIMEOUT:
            sm->active = false;
            pan_home_error_flag = 1;
            if (m->disable_step_timer)
                m->disable_step_timer();
            break;

        default:
            break;
    }
}

// ---- Tilt homing SM — called from TIM5 ISR ----
static void tilt_home_tick(home_sm_t *sm, motor_ctrl_t *m)
{
    sm->steps_in_state++;

    switch (sm->state)
    {
        case HOME_FULL_SCAN:
        {
            // PA7 HIGH = tilt-down sensor blocked = zero position
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET)
            {
                if (m->disable_step_timer)
                    m->disable_step_timer();
                __disable_irq();
                m->abs_position = 0;
                tilt_current_step = 0;
                __enable_irq();
                m->homed = true;
                sm->state = HOME_LOCKED;
                sm->active = false;
                tilt_home_done_flag = 1;
            }
            if (sm->steps_in_state > sm->timeout_steps)
            {
                sm->state = HOME_ERROR;
                tilt_home_error_flag = 1;
                if (m->disable_step_timer)
                    m->disable_step_timer();
            }
            break;
        }

        default:
            break;
    }
}

bool tilt_down_blocked(void)
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET; //PA7 HIGH BLOCKS DOWN
}

bool tilt_up_blocked(void)
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET; //PA4 HIGH BLOCKS UP
}

uint32_t calculate_speed_from_pelco(uint8_t raw_speed, bool is_tilt)
{
    if (raw_speed == 0)
        return 0;

    uint32_t slow, mid, fast, turbo;

    if (is_tilt)
    {
        slow  = 4500;
        mid   = 9000;
        fast  = 12000;
        turbo = 14000;
    }
    else
    {
        slow  = 4000;
        mid   = 6500;
        fast  = 9000;
        turbo = 12000;
    }

    if (raw_speed == 64)
        return turbo;

    if (raw_speed <= 21)
        return slow;
    else if (raw_speed <= 42)
        return mid;
    else
        return fast;
}

/*
void ptz_execute_intent(void)
{
	 if (!ptz_intent_valid)
	        return;

	    uint8_t action = ptz_intent.action;

	    // Explicit stop command — halt everything immediately
	        if (action == PTZ_STOP || action == PTZ_NONE)
	        {
	            stop_all_motion();
	            ptz_intent_valid = 0;
	            return;
	        }

	    uint32_t pan_target  = calculate_speed_from_pelco(ptz_intent.pan_speed, false);
	    uint32_t tilt_target = calculate_speed_from_pelco(ptz_intent.tilt_speed, true);

	    // ---- PAN: no limits, always independent ----
	        bool want_pan_left  = (action == PTZ_PAN_LEFT           ||
	                               action == PTZ_PAN_TILT_UP_LEFT   ||
	                               action == PTZ_PAN_TILT_DOWN_LEFT);
	        bool want_pan_right = (action == PTZ_PAN_RIGHT          ||
	                               action == PTZ_PAN_TILT_UP_RIGHT  ||
	                               action == PTZ_PAN_TILT_DOWN_RIGHT);

	        if (want_pan_left)
	            {
	                direction = 2;
	                move_pan(2, pan_target);
	            }
	            else if (want_pan_right)
	            {
	                direction = 1;
	                move_pan(1, pan_target);
	            }
	            else
	            {
	                stop_pan(0);
	            }

	        // ---- TILT: hard stop at limits, independent of pan ----
	            bool want_up   = (action == PTZ_TILT_UP              ||
	                              action == PTZ_PAN_TILT_UP_LEFT     ||
	                              action == PTZ_PAN_TILT_UP_RIGHT);
	            bool want_down = (action == PTZ_TILT_DOWN            ||
	                              action == PTZ_PAN_TILT_DOWN_LEFT   ||
	                              action == PTZ_PAN_TILT_DOWN_RIGHT);

	            // Read sensors fresh every execution
	            bool up_blocked   = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET);
	            bool down_blocked = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET);

	            if (want_up && !up_blocked)
	            {
	                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	                move_tilt(tilt_target);
	            }
	            else if (want_down && !down_blocked)
	            {
	                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	                move_tilt(tilt_target);
	            }
	            else if (want_up || want_down)
	            {
	                // Tilt was requested but limit hit — stop tilt only, pan keeps going
	                stop_tilt(0);
	            }
	            // Pure pan command: do not touch tilt timer state
	        }
*/

void move_pan(uint8_t direction, int TARGET_FREQ_HZ)
{
    if (TARGET_FREQ_HZ < 100)
        TARGET_FREQ_HZ = 100;

    motor_dir_t new_dir = (direction == 1) ? DIR_POSITIVE : DIR_NEGATIVE;

    HAL_GPIO_WritePin(PAN_DIR_GPIO_Port, PAN_DIR_Pin,
                      (direction == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    current_direction = direction;

    // Direction reversal — decel first, next call cold-starts in new direction
    if (motors[AXIS_PAN].state != MOTOR_IDLE &&
        motors[AXIS_PAN].direction != new_dir)
    {
        motors[AXIS_PAN].target_freq_hz = motors[AXIS_PAN].min_freq_hz;
        motors[AXIS_PAN].state          = MOTOR_DECEL;
        return;
    }

    motors[AXIS_PAN].direction = new_dir;

    if (motors[AXIS_PAN].state != MOTOR_IDLE &&
        motors[AXIS_PAN].target_freq_hz == (uint32_t)TARGET_FREQ_HZ)
        return;

    motors[AXIS_PAN].target_freq_hz = (uint32_t)TARGET_FREQ_HZ;

    if (motors[AXIS_PAN].state == MOTOR_IDLE)
    {
        motors[AXIS_PAN].current_freq_hz = motors[AXIS_PAN].min_freq_hz;
        motors[AXIS_PAN].state           = MOTOR_ACCEL;

        uint32_t timer_clock = HAL_RCC_GetPCLK1Freq() * 2;
        uint32_t prescaler   = PAN_TIMER.Init.Prescaler + 1;
        uint32_t arr         = (timer_clock / prescaler / motors[AXIS_PAN].min_freq_hz) - 1;
        __HAL_TIM_SET_AUTORELOAD(&PAN_TIMER, arr);
        __HAL_TIM_SET_COMPARE(&PAN_TIMER, PAN_CHANNEL, arr / 2);
        __HAL_TIM_ENABLE_IT(&PAN_TIMER, TIM_IT_UPDATE);
        HAL_TIM_PWM_Start(&PAN_TIMER, PAN_CHANNEL);
    }
    else
    {
        if ((uint32_t)TARGET_FREQ_HZ > motors[AXIS_PAN].current_freq_hz)
            motors[AXIS_PAN].state = MOTOR_ACCEL;
        else if ((uint32_t)TARGET_FREQ_HZ < motors[AXIS_PAN].current_freq_hz)
            motors[AXIS_PAN].state = MOTOR_DECEL;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // PAN Motor (TIM2)
    if (htim->Instance == TIM2)
    {
        motor_ctrl_t *m = &motors[AXIS_PAN];

        // Count step: CW (DIR_POSITIVE) = positive, CCW = negative
        if (m->direction == DIR_POSITIVE)
            m->abs_position++;
        else
            m->abs_position--;

        // Pan sensor PA6: LOW = triggered = index/home pulse
        // Reset counter to zero every time sensor is seen — provides absolute reference
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET)
        {
            m->abs_position    = 0;
            right_current_step = 0;
        }
        else
        {
            right_current_step = m->abs_position;
        }

        motor_ramp_tick(m);

        if (pan_home_sm.active)
            pan_home_tick(&pan_home_sm, m);
    }

    // TILT Motor (TIM5)
       if (htim->Instance == TIM5)
       {
           motor_ctrl_t *m = &motors[AXIS_TILT];

           if (m->direction == DIR_POSITIVE)
               m->abs_position++;
           else
               m->abs_position--;

           // Read tilt sensors on every step — ISR-level enforcement
           // This is the only reliable way given ~29ms Pelco frame rate
           bool hit_up   = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET);
           bool hit_down = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET);

           // PA7 = tilt-down sensor = absolute zero reference, reset position on every hit
           if (hit_down)
           {
               m->abs_position   = 0;
               tilt_current_step = 0;
           }
           else
           {
               tilt_current_step = m->abs_position;
           }

           // Hard limit enforcement: stop immediately if limit hit in the direction of travel
           if ((hit_up   && m->direction == DIR_POSITIVE) ||
               (hit_down && m->direction == DIR_NEGATIVE))
           {
               // Hard stop from ISR — cannot call HAL functions, do it manually
               // Clear CCR to 0 to stop PWM output immediately
               TILT_TIMER.Instance->CCR3 = 0;   // TIM5 CH3 compare = 0, output goes low
               __HAL_TIM_DISABLE_IT(&TILT_TIMER, TIM_IT_UPDATE);
               m->state           = MOTOR_IDLE;
               m->current_freq_hz = 0;
               // Do not return — still need to update tilt_current_step above
           }
           else
           {
               motor_ramp_tick(m);
           }

           if (tilt_home_sm.active)
               tilt_home_tick(&tilt_home_sm, m);
       }
}

/**
  * @brief Process the HOME command using IR sensor at PA6 for pan motor (fast version)
  * @retval None
  */

void move_tilt(int TARGET_FREQ_HZ)
{
    if (TARGET_FREQ_HZ < 100)
        TARGET_FREQ_HZ = 100;

    motor_dir_t new_dir =
        (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) ? DIR_POSITIVE : DIR_NEGATIVE;

    // If direction reversed while motor is still moving, force stop first
    if (motors[AXIS_TILT].state != MOTOR_IDLE &&
        motors[AXIS_TILT].direction != new_dir)
    {
        // Hard stop — direction change requires full stop before restart
        // ISR will complete the stop, next call will cold-start in new direction
        motors[AXIS_TILT].target_freq_hz = motors[AXIS_TILT].min_freq_hz;
        motors[AXIS_TILT].state          = MOTOR_DECEL;
        return;
    }

    motors[AXIS_TILT].direction = new_dir;

    if (motors[AXIS_TILT].state != MOTOR_IDLE &&
        motors[AXIS_TILT].target_freq_hz == (uint32_t)TARGET_FREQ_HZ)
        return;

    motors[AXIS_TILT].target_freq_hz = (uint32_t)TARGET_FREQ_HZ;

    if (motors[AXIS_TILT].state == MOTOR_IDLE)
    {
        motors[AXIS_TILT].current_freq_hz = motors[AXIS_TILT].min_freq_hz;
        motors[AXIS_TILT].state           = MOTOR_ACCEL;

        uint32_t timer_clock = HAL_RCC_GetPCLK1Freq() * 2;
        uint32_t prescaler   = TILT_TIMER.Init.Prescaler + 1;
        uint32_t arr         = (timer_clock / prescaler / motors[AXIS_TILT].min_freq_hz) - 1;
        __HAL_TIM_SET_AUTORELOAD(&TILT_TIMER, arr);
        __HAL_TIM_SET_COMPARE(&TILT_TIMER, TILT_CHANNEL, arr / 2);
        __HAL_TIM_ENABLE_IT(&TILT_TIMER, TIM_IT_UPDATE);
        HAL_TIM_PWM_Start(&TILT_TIMER, TILT_CHANNEL);
    }
    else
    {
        if ((uint32_t)TARGET_FREQ_HZ > motors[AXIS_TILT].current_freq_hz)
            motors[AXIS_TILT].state = MOTOR_ACCEL;
        else if ((uint32_t)TARGET_FREQ_HZ < motors[AXIS_TILT].current_freq_hz)
            motors[AXIS_TILT].state = MOTOR_DECEL;
    }
}

/**
  * @brief Process the HOME command using IR sensor at PA6 for pan motor (fast version)
  * @retval None
  */
void stop_pan(int TARGET_FREQ_HZ)
{
    (void)TARGET_FREQ_HZ;
    if (motors[AXIS_PAN].state == MOTOR_IDLE) return;

    // Hard stop — safe for limit hits and command stops
    // At the speeds used (4000–12000 Hz) decel over ~40 steps is negligible
    __HAL_TIM_DISABLE_IT(&PAN_TIMER, TIM_IT_UPDATE);
    __HAL_TIM_SET_COMPARE(&PAN_TIMER, PAN_CHANNEL, 0);
    HAL_TIM_PWM_Stop(&PAN_TIMER, PAN_CHANNEL);
//    HAL_GPIO_WritePin(PAN_STEP_GPIO_Port, PAN_STEP_Pin, GPIO_PIN_RESET);


    motors[AXIS_PAN].state           = MOTOR_IDLE;
    motors[AXIS_PAN].current_freq_hz = 0;
}


/**
  * @brief Process the HOME command using IR sensor at PA6 for pan motor (fast version)
  * @retval None
  */
void stop_tilt(int TARGET_FREQ_HZ)
{
    (void)TARGET_FREQ_HZ;
    if (motors[AXIS_TILT].state == MOTOR_IDLE) return;

    // Hard stop — required for limit hits (decel path cannot be used when ISR
    // may be disabled, and limit requires immediate stop)
    __HAL_TIM_DISABLE_IT(&TILT_TIMER, TIM_IT_UPDATE);
    __HAL_TIM_SET_COMPARE(&TILT_TIMER, TILT_CHANNEL, 0);
    HAL_TIM_PWM_Stop(&TILT_TIMER, TILT_CHANNEL);
//    HAL_GPIO_WritePin(TILT_STEP_GPIO_Port, TILT_STEP_Pin, GPIO_PIN_RESET);

    motors[AXIS_TILT].state           = MOTOR_IDLE;
    motors[AXIS_TILT].current_freq_hz = 0;
}

void stop_all_motion(void)
{
    // Disable UPDATE interrupts first — prevents spurious ramp ticks after stop
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_UPDATE);

    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);

    motors[AXIS_PAN].state            = MOTOR_IDLE;
    motors[AXIS_PAN].current_freq_hz  = 0;
    motors[AXIS_TILT].state           = MOTOR_IDLE;
    motors[AXIS_TILT].current_freq_hz = 0;

    current_step_pan  = 0;
    current_step_tilt = 0;
}

// ---- Timer frequency setter — called from ramp engine inside ISR ----
// Safe: single 32-bit ARR write, no HAL overhead
static void pan_set_timer_freq(uint32_t freq_hz)
{
    if (freq_hz < 100) freq_hz = 100;
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq() * 2;
    uint32_t prescaler   = PAN_TIMER.Init.Prescaler + 1;
    uint32_t arr = (timer_clock / prescaler / freq_hz) - 1;
    __HAL_TIM_SET_AUTORELOAD(&PAN_TIMER, arr);
    __HAL_TIM_SET_COMPARE(&PAN_TIMER, PAN_CHANNEL, arr / 2);
}

static void tilt_set_timer_freq(uint32_t freq_hz)
{
    if (freq_hz < 100) freq_hz = 100;
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq() * 2;
    uint32_t prescaler   = TILT_TIMER.Init.Prescaler + 1;
    uint32_t arr = (timer_clock / prescaler / freq_hz) - 1;
    __HAL_TIM_SET_AUTORELOAD(&TILT_TIMER, arr);
    __HAL_TIM_SET_COMPARE(&TILT_TIMER, TILT_CHANNEL, arr / 2);
}

static void pan_disable_timer(void)
{
    HAL_TIM_PWM_Stop(&PAN_TIMER, PAN_CHANNEL);
    __HAL_TIM_SET_COMPARE(&PAN_TIMER, PAN_CHANNEL, 0);
    // Do NOT disable TIM_IT_UPDATE here — called from within the ISR
    // stop_pan() / stop_all_motion() handle interrupt disable externally
}

static void tilt_disable_timer(void)
{
    HAL_TIM_PWM_Stop(&TILT_TIMER, TILT_CHANNEL);
    __HAL_TIM_SET_COMPARE(&TILT_TIMER, TILT_CHANNEL, 0);
}

// Call this once from main() after all MX_Init calls, before UART receive
void motor_bsp_init(void)
{
    // PAN motor struct
    motors[AXIS_PAN].state           = MOTOR_IDLE;
    motors[AXIS_PAN].direction       = DIR_POSITIVE;
    motors[AXIS_PAN].abs_position    = 0;
    motors[AXIS_PAN].current_freq_hz = 0;
    motors[AXIS_PAN].target_freq_hz  = 0;
    motors[AXIS_PAN].min_freq_hz     = 500;
    motors[AXIS_PAN].max_freq_hz     = 12000;
    motors[AXIS_PAN].ramp_step_hz    = 200;   // Hz added per step pulse during accel
    motors[AXIS_PAN].homed           = false;
    motors[AXIS_PAN].enable_step_timer  = pan_set_timer_freq;
    motors[AXIS_PAN].disable_step_timer = pan_disable_timer;

    // TILT motor struct
    motors[AXIS_TILT].state           = MOTOR_IDLE;
    motors[AXIS_TILT].direction       = DIR_POSITIVE;
    motors[AXIS_TILT].abs_position    = 0;
    motors[AXIS_TILT].current_freq_hz = 0;
    motors[AXIS_TILT].target_freq_hz  = 0;
    motors[AXIS_TILT].min_freq_hz     = 500;
    motors[AXIS_TILT].max_freq_hz     = 13000;
    motors[AXIS_TILT].ramp_step_hz    = 200;
    motors[AXIS_TILT].homed           = false;
    motors[AXIS_TILT].enable_step_timer  = tilt_set_timer_freq;
    motors[AXIS_TILT].disable_step_timer = tilt_disable_timer;
}
