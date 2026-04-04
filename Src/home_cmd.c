/*
 * home_cmd.c
 *
 *  Created on:
 *      Author: Avikalp
 */

#include <main.h>

#include "home_cmd.h"
#include "motor_move.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim2; // Required for HAL_TIM_PWM_Stop()
extern TIM_HandleTypeDef htim5;
extern volatile int32_t right_current_step;
extern volatile int32_t left_current_step;
extern volatile int32_t tilt_current_step;
extern motor_ctrl_t motors[];
volatile uint8_t home_request = 0;

/* Preset table — persists in RAM until power cycle */
preset_t preset_table[MAX_PRESETS] = {0};

static bool pan_sensor_lock_sequence(uint8_t approach_dir);

/*  Internal speed constants                                           */
/* ------------------------------------------------------------------ */
#define TILT_CRUISE_HZ      9000
#define TILT_SLOW_HZ        3000
#define TILT_DECEL_STEPS    2000
#define PAN_CRUISE_HZ       6000
#define PAN_SLOW_HZ         1200
#define PAN_DECEL_STEPS     400
#define GOTO_TIMEOUT_MS     20000

void goto_home_position(void)
{
    char msg[96];

    if (!motors[AXIS_PAN].homed || !motors[AXIS_TILT].homed)
    {
        snprintf(msg, sizeof(msg), ">> GOTO HOME: not homed\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }

     //-- Compute pan shortest path to 0
    __disable_irq();
    int32_t pan_pos  = motors[AXIS_PAN].abs_position;
    int32_t tilt_pos = motors[AXIS_TILT].abs_position;
    __enable_irq();

    int32_t pan_norm = pan_pos % PAN_STEPS_PER_REV;
    if (pan_norm < 0) pan_norm += PAN_STEPS_PER_REV;

    int32_t pan_delta;
    uint8_t pan_dir;

    if (pan_norm == 0)
    {
        pan_delta = 0;
        pan_dir   = 0;
    }
    else if (pan_norm <= (PAN_STEPS_PER_REV / 2))
    {
        pan_delta = pan_norm;
        pan_dir   = 2;                                          /* CCW = DIR_NEGATIVE */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        motors[AXIS_PAN].direction = DIR_NEGATIVE;
    }
    else
    {
        pan_delta = PAN_STEPS_PER_REV - pan_norm;
        pan_dir   = 1;                                          /* CW = DIR_POSITIVE */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        motors[AXIS_PAN].direction = DIR_POSITIVE;
    }
    // Determine if we need sensor-based final lock
        // Near-home = step target lands within SLOT_HALF_STEPS of 0 (CCW approach)
        // or within SLOT_HALF_STEPS from the CW side (pan_norm > REV - SLOT_HALF_STEPS)
        bool use_sensor_lock = (pan_norm <= SLOT_HALF_STEPS) ||
                               (pan_norm >= (PAN_STEPS_PER_REV - SLOT_HALF_STEPS)) ||
                               (pan_delta <= SLOT_HALF_STEPS);

    /* Tilt delta */
    int32_t tilt_delta = (int32_t)TILT_REAL_HOME_STEPS - tilt_pos;

    /* Set tilt direction */
    if (tilt_delta > 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        motors[AXIS_TILT].direction = DIR_POSITIVE;
    }
    else if (tilt_delta < 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        motors[AXIS_TILT].direction = DIR_NEGATIVE;
    }

    snprintf(msg, sizeof(msg),
             ">> GOTO HOME: pan_pos=%ld(delta=%ld,dir=%u) tilt_pos=%ld(delta=%ld)\r\n",
             (long)pan_pos, (long)pan_delta, pan_dir,
             (long)tilt_pos, (long)tilt_delta);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    /* ----------------------------------------------------------------
     * Simultaneous move polling loop
     * ---------------------------------------------------------------- */
     bool pan_done  = (pan_delta == 0 && !use_sensor_lock);
     bool tilt_done = (tilt_delta == 0);

     // If pan_delta==0 but use_sensor_lock: pan is already inside slot,
         // must do back-off + re-approach. pan_done stays false.
         // If already cleanly at 0 with sensor clear — verify:
         if (pan_delta == 0 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET)
         {
             // Sensor not triggered at step 0 — position is good, no lock needed
             pan_done = true;
         }

    __disable_irq();
    int32_t pan_start_pos  = motors[AXIS_PAN].abs_position;
//    int32_t tilt_start_pos = motors[AXIS_TILT].abs_position;
    __enable_irq();

    uint32_t t_start = HAL_GetTick();

       /* ----------------------------------------------------------------
        * Phase A: simultaneous coarse move (tilt + pan step counting)
        * Stop pan when within SLOT_HALF_STEPS of target if using sensor lock,
        * otherwise stop on delta completion as before.
        * ---------------------------------------------------------------- */
       while ((!pan_done || !tilt_done) &&
              ((HAL_GetTick() - t_start) < GOTO_TIMEOUT_MS))
       {
           __disable_irq();
           int32_t p_pos = motors[AXIS_PAN].abs_position;
           int32_t t_pos = motors[AXIS_TILT].abs_position;
           __enable_irq();

           /* PAN coarse phase */
           if (!pan_done)
           {
               int32_t traveled  = p_pos - pan_start_pos;
               if (traveled < 0) traveled = -traveled;
               int32_t remaining = pan_delta - traveled;

               // If approaching CW (dir=1) and we're now close enough, stop and
               // reverse for CCW sensor approach — overshoot handled below
               int32_t stop_threshold = use_sensor_lock ? (SLOT_HALF_STEPS + 100) : 0;

               if (remaining <= stop_threshold)
               {
                   stop_pan(0);
                   pan_done = true;  // coarse done; sensor lock runs after loop
               }
               else
               {
                   // PA6 already triggered mid-coarse — we entered the slot early
                   if (use_sensor_lock &&
                       HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET)
                   {
                       stop_pan(0);
                       pan_done = true;  // sensor lock runs after loop
                   }
                   else
                   {
                       uint32_t spd = (remaining < PAN_DECEL_STEPS) ? PAN_SLOW_HZ : PAN_CRUISE_HZ;
                       direction = pan_dir;
                       move_pan(pan_dir, spd);
                   }
               }
           }

           /* TILT — unchanged */
           if (!tilt_done)
           {
               int32_t t_remaining = (int32_t)TILT_REAL_HOME_STEPS - t_pos;
               int32_t abs_rem     = (t_remaining < 0) ? -t_remaining : t_remaining;
               bool arrived = (tilt_delta > 0) ? (t_pos >= TILT_REAL_HOME_STEPS) :
                              (tilt_delta < 0) ? (t_pos <= TILT_REAL_HOME_STEPS) : true;
               if (arrived)
               {
                   stop_tilt(0);
                   tilt_done = true;
                   snprintf(msg, sizeof(msg), ">> TILT GOTO DONE. pos=%ld\r\n", (long)t_pos);
                   HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
               }
               else
               {
                   uint32_t spd = (abs_rem < TILT_DECEL_STEPS) ? TILT_SLOW_HZ : TILT_CRUISE_HZ;
                   move_tilt(spd);
               }
           }

           HAL_Delay(1);
       }

       if (!tilt_done)
       {
           stop_all_motion();
           snprintf(msg, sizeof(msg), ">> GOTO HOME ERROR: tilt timeout\r\n");
           HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
           return;
       }

       /* ----------------------------------------------------------------
        * Phase B: Pan sensor lock (runs only if use_sensor_lock == true)
        * Mirrors process_home_command() phases 2 and 3 exactly.
        * If CW approach: motor is now near slot from CW side.
        *   Must overshoot to clear slot CW, then re-approach CCW.
        * If CCW approach (normal): motor entered slot from CCW side.
        *   Just do back-off CW then slow CCW re-approach.
        * ---------------------------------------------------------------- */
       if (!use_sensor_lock)
       {
           snprintf(msg, sizeof(msg), ">> GOTO HOME COMPLETE.\r\n");
           HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
           preset_save(1);
           return;
       }

       snprintf(msg, sizeof(msg), ">> PAN SENSOR LOCK: back-off phase\r\n");
       HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

       // If approaching was CW (pan_dir==1), we may be on the CW edge of the slot.
       // Overshoot further CW past the slot before CCW re-approach.
       if (pan_dir == 1)
       {
           HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
           motors[AXIS_PAN].direction = DIR_POSITIVE;
           uint32_t overshoot_start = HAL_GetTick();
           while ((HAL_GetTick() - overshoot_start) < 2000)
           {
               move_pan(1, PAN_HOME_BACKOFF_HZ);
               HAL_Delay(1);
               // Stop once sensor clears — we've exited the slot CW
               if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET)
                   break;
           }
           stop_pan(0);
           HAL_Delay(50);
       }

       // Back-off CW until sensor clears (if sensor still triggered from CCW entry)
       if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET)
       {
           HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
           motors[AXIS_PAN].direction = DIR_POSITIVE;
           uint32_t backoff_start = HAL_GetTick();
           while ((HAL_GetTick() - backoff_start) < 1000)
           {
               move_pan(1, PAN_HOME_BACKOFF_HZ);
               HAL_Delay(1);
               if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET)
                   break;
           }
           stop_pan(0);
           HAL_Delay(50);
       }

       // Slow CCW re-approach — identical to process_home_command() phase 3
       snprintf(msg, sizeof(msg), ">> PAN SENSOR LOCK: slow CCW approach\r\n");
       HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
       motors[AXIS_PAN].direction = DIR_NEGATIVE;

       uint32_t lock_start = HAL_GetTick();
       bool pan_locked = false;
       while ((HAL_GetTick() - lock_start) < 5000)
       {
           move_pan(2, PAN_HOME_SLOW_HZ);
           HAL_Delay(1);
           if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET)
           {
               stop_pan(0);
               __disable_irq();
               motors[AXIS_PAN].abs_position = 0;
               right_current_step = 0;
               __enable_irq();
               pan_locked = true;
               snprintf(msg, sizeof(msg), ">> PAN SENSOR LOCKED. pos=0\r\n");
               HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
               break;
           }
       }

       if (!pan_locked)
       {
           stop_all_motion();
           snprintf(msg, sizeof(msg), ">> PAN SENSOR LOCK ERROR: timeout\r\n");
           HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
           return;
       }

       snprintf(msg, sizeof(msg), ">> GOTO HOME COMPLETE.\r\n");
       HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    preset_save(1);   // pan=0, tilt=30000 — canonical home, shared reference
}
/**
  * @brief Process the HOME command using IR sensor at PA6 for pan motor (fast version)
  * @retval None
  */

void process_home_command(void)
{
    char msg[96];
    extern UART_HandleTypeDef huart3;

   /* // ================================================================
    // PHASE 0a: TILT UP — move up until PA4 triggers (tilt-up sensor)
    // ================================================================
//    snprintf(msg, sizeof(msg), ">> TILT HOME: moving up to PA4...\r\n");
//    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);  // DIR = up
//    motors[AXIS_TILT].direction = DIR_POSITIVE;
//
//    uint32_t tilt_up_start = HAL_GetTick();
//    while ((HAL_GetTick() - tilt_up_start) < 15000)
//    {
//        move_tilt(9000);
//        HAL_Delay(1);
//
//        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET)  // PA4 HIGH = tilt-up blocked
//        {
//            stop_all_motion();
//            snprintf(msg, sizeof(msg), ">> TILT UP LOCKED at PA4. Starting down...\r\n");
//            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//            goto tilt_down;
//        }
//    }
//    stop_all_motion();
//    snprintf(msg, sizeof(msg), ">> TILT UP ERROR: PA4 timeout\r\n");
//    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//    return;  // abort — no point homing pan if tilt failed
*/

    // ================================================================
    // PHASE 0: TILT HOME — move down until PA7 triggers (tilt-down sensor)
    // ================================================================
//tilt_down:
    snprintf(msg, sizeof(msg), ">> TILT HOME: moving down...\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Set DIR down, start slow
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);  // DIR = down
    motors[AXIS_TILT].direction = DIR_NEGATIVE;

    uint32_t tilt_start = HAL_GetTick();
    while ((HAL_GetTick() - tilt_start) < 15000)
    {
        move_tilt(9000);//(6000);
        HAL_Delay(1);  // small yield — 1ms is fine in startup blocking context

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET)  // PA7 HIGH = tilt-down blocked
        {
            stop_all_motion();
            __disable_irq();
            motors[AXIS_TILT].abs_position = 0;
            tilt_current_step = 0;
            __enable_irq();
            motors[AXIS_TILT].homed = true;

            snprintf(msg, sizeof(msg), ">> TILT HOME LOCKED. pos=0\r\n");
            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            goto pan_home;
        }
    }
    stop_all_motion();
    snprintf(msg, sizeof(msg), ">> TILT HOME ERROR: timeout\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

pan_home:
    // ================================================================
    // PHASE 1: PAN HOME — fast CCW scan until PA6 triggers
    // ================================================================
    snprintf(msg, sizeof(msg), ">> PAN HOME: fast scan CCW...\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    direction = 1;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    motors[AXIS_PAN].direction = DIR_NEGATIVE;

    uint32_t pan_start = HAL_GetTick();
    while ((HAL_GetTick() - pan_start) < 15000)
    {
        move_pan(2, 5000);
        HAL_Delay(1);

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET)  // PA6 LOW = triggered
            break;
    }

    if ((HAL_GetTick() - pan_start) >= 15000)
    {
        stop_all_motion();
        snprintf(msg, sizeof(msg), ">> PAN HOME ERROR: timeout\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }

    // ================================================================
    // PHASE 2: PAN BACK OFF — CW until sensor clears
    // ================================================================
    direction = 1;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    motors[AXIS_PAN].direction = DIR_POSITIVE;

    uint32_t backoff = HAL_GetTick();
    while ((HAL_GetTick() - backoff) < 1000)
    {
        move_pan(1, 1500);
        HAL_Delay(1);
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET)
            break;
    }
    stop_all_motion();
    while (motors[AXIS_PAN].state != MOTOR_IDLE)
            HAL_Delay(1);

    // ================================================================
    // PHASE 3: PAN SLOW CcW re-approach for precise lock
    // ================================================================
    direction = 2;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    motors[AXIS_PAN].direction = DIR_NEGATIVE;

    uint32_t slow_start = HAL_GetTick();
    while ((HAL_GetTick() - slow_start) < 5000)
    {
        move_pan(2, 1200);
        HAL_Delay(1);

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET)
        {
            stop_all_motion();
            __disable_irq();
            motors[AXIS_PAN].abs_position = 0;
            right_current_step = 0;
            left_current_step  = 0;
            __enable_irq();
            motors[AXIS_PAN].homed = true;

            snprintf(msg, sizeof(msg), ">> PAN HOME LOCKED. pos=0\r\n");
            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            return;
        }
    }

    stop_all_motion();
    snprintf(msg, sizeof(msg), ">> PAN HOME ERROR: slow approach timeout\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}


/* ------------------------------------------------------------------ */
/*  Preset operations                                                  */
/* ------------------------------------------------------------------ */

void preset_save(uint8_t id)
{
    char msg[64];
    if (id == 0 || id > MAX_PRESETS)
    {
        snprintf(msg, sizeof(msg), ">> PRESET SAVE: invalid id=%u\r\n", id);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }

    uint8_t idx = id - 1;

    __disable_irq();
    preset_table[idx].pan  = motors[AXIS_PAN].abs_position;
    preset_table[idx].tilt = motors[AXIS_TILT].abs_position;
    preset_table[idx].zoom = 0;   /* placeholder for zoom axis */
    preset_table[idx].valid = true;
    __enable_irq();

    snprintf(msg, sizeof(msg), ">> PRESET %u SAVED: pan=%ld tilt=%ld\r\n",
             id,
             (long)preset_table[idx].pan,
             (long)preset_table[idx].tilt);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void preset_clear(uint8_t id)
{
    char msg[64];
    if (id == 0 || id > MAX_PRESETS) return;
    uint8_t idx = id - 1;
    preset_table[idx].valid = false;
    snprintf(msg, sizeof(msg), ">> PRESET %u CLEARED\r\n", id);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

/*
 * preset_goto — blocking positional move to a saved preset.
 * Uses the same simultaneous polling loop as goto_home_position().
 */
void preset_goto(uint8_t id)
{
    char msg[96];

    if (id == 0 || id > MAX_PRESETS)
    {
        snprintf(msg, sizeof(msg), ">> PRESET GOTO: invalid id=%u\r\n", id);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }

    uint8_t idx = id - 1;
    if (!preset_table[idx].valid)
    {
        snprintf(msg, sizeof(msg), ">> PRESET %u: not set\r\n", id);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }

    if (!motors[AXIS_PAN].homed || !motors[AXIS_TILT].homed)
    {
        snprintf(msg, sizeof(msg), ">> PRESET GOTO: system not homed\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }

    __disable_irq();
    int32_t pan_target  = preset_table[idx].pan;
    int32_t tilt_target = preset_table[idx].tilt;
    int32_t pan_pos     = motors[AXIS_PAN].abs_position;
    int32_t tilt_pos    = motors[AXIS_TILT].abs_position;
    __enable_irq();

    /* Pan: shortest path to target
     * delta on the circular [0, REV) space */
    int32_t cur_norm = pan_pos  % PAN_STEPS_PER_REV; if (cur_norm < 0) cur_norm += PAN_STEPS_PER_REV;
    int32_t tgt_norm = pan_target % PAN_STEPS_PER_REV; if (tgt_norm < 0) tgt_norm += PAN_STEPS_PER_REV;

    int32_t diff = tgt_norm - cur_norm;
    /* wrap into (-REV/2, REV/2] */
    if (diff >  (int32_t)(PAN_STEPS_PER_REV / 2)) diff -= PAN_STEPS_PER_REV;
    if (diff < -(int32_t)(PAN_STEPS_PER_REV / 2)) diff += PAN_STEPS_PER_REV;

    int32_t pan_delta = diff;
    uint8_t pan_dir;

    // After pan_dir computation, before the log print:
    bool use_sensor_lock = (pan_target == 0) &&
                           ((cur_norm  <= SLOT_HALF_STEPS) ||
                            (cur_norm  >= (PAN_STEPS_PER_REV - SLOT_HALF_STEPS)) ||
                            (pan_delta <= SLOT_HALF_STEPS));

    if (pan_delta == 0)
    {
        pan_dir = 0;
    }
    else if (pan_delta > 0)
    {
        pan_dir = 1;    /* CW */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        motors[AXIS_PAN].direction = DIR_POSITIVE;
    }
    else
    {
        pan_delta = -pan_delta;
        pan_dir   = 2;  /* CCW */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        motors[AXIS_PAN].direction = DIR_NEGATIVE;
    }

    int32_t tilt_delta = tilt_target - tilt_pos;
    if (tilt_delta > 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        motors[AXIS_TILT].direction = DIR_POSITIVE;
    }
    else if (tilt_delta < 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        motors[AXIS_TILT].direction = DIR_NEGATIVE;
    }

    snprintf(msg, sizeof(msg),
             ">> GOTO PRESET %u: pan_tgt=%ld(delta=%ld) tilt_tgt=%ld(delta=%ld)\r\n",
             id, (long)pan_target, (long)pan_delta,
             (long)tilt_target, (long)tilt_delta);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    bool pan_done  = (pan_delta == 0);
    bool tilt_done = (tilt_delta == 0);

    __disable_irq();
    int32_t pan_start  = motors[AXIS_PAN].abs_position;
    __enable_irq();

    uint32_t t0 = HAL_GetTick();

    while ((!pan_done || !tilt_done) &&
           ((HAL_GetTick() - t0) < GOTO_TIMEOUT_MS))
    {
        __disable_irq();
        int32_t p = motors[AXIS_PAN].abs_position;
        int32_t t = motors[AXIS_TILT].abs_position;
        __enable_irq();

        /* PAN */
        if (!pan_done)
        {
            int32_t traveled  = p - pan_start;
            if (traveled < 0) traveled = -traveled;
            int32_t remaining = pan_delta - traveled;
            if (remaining <= 0)
            {
                stop_pan(0);
                pan_done = true;
            }
            else
            {
                uint32_t spd = (remaining < PAN_DECEL_STEPS) ? PAN_SLOW_HZ : PAN_CRUISE_HZ;
                direction = pan_dir;
                move_pan(pan_dir, spd);
            }
        }

        /* TILT */
        if (!tilt_done)
        {
            int32_t remaining = tilt_target - t;
            int32_t abs_rem   = (remaining < 0) ? -remaining : remaining;

            bool arrived = (tilt_delta > 0) ? (t >= tilt_target) :
                           (tilt_delta < 0) ? (t <= tilt_target) : true;
            if (arrived)
            {
                stop_tilt(0);
                tilt_done = true;
            }
            else
            {
                uint32_t spd = (abs_rem < TILT_DECEL_STEPS) ? TILT_SLOW_HZ : TILT_CRUISE_HZ;
                move_tilt(spd);
            }
        }

        HAL_Delay(1);
    }

    if (!pan_done || !tilt_done)
    {
        stop_all_motion();
        snprintf(msg, sizeof(msg), ">> PRESET GOTO ERROR: timeout\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }
    // Pan sensor lock for preset targets near home
    if (pan_done && use_sensor_lock)
    {
    	// Same Phase B sequence as goto_home_position() — extract as a helper
    	pan_sensor_lock_sequence(pan_dir);   // see below
    }

    snprintf(msg, sizeof(msg), ">> PRESET %u REACHED.\r\n", id);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

// home_cmd.c — internal helper, not exposed in header
static bool pan_sensor_lock_sequence(uint8_t approach_dir)
{
    char msg[64];

    // If CW approach — overshoot past slot first
    if (approach_dir == 1)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        motors[AXIS_PAN].direction = DIR_POSITIVE;
        uint32_t t = HAL_GetTick();
        while ((HAL_GetTick() - t) < 2000)
        {
            move_pan(1, PAN_HOME_BACKOFF_HZ);
            HAL_Delay(1);
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) break;
        }
        stop_pan(0);
        HAL_Delay(50);
    }

    // Back-off CW if still in slot
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        motors[AXIS_PAN].direction = DIR_POSITIVE;
        uint32_t t = HAL_GetTick();
        while ((HAL_GetTick() - t) < 1000)
        {
            move_pan(1, PAN_HOME_BACKOFF_HZ);
            HAL_Delay(1);
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) break;
        }
        stop_pan(0);
        HAL_Delay(50);
    }

    // Slow CCW re-approach
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    motors[AXIS_PAN].direction = DIR_NEGATIVE;
    uint32_t t = HAL_GetTick();
    while ((HAL_GetTick() - t) < 5000)
    {
        move_pan(2, PAN_HOME_SLOW_HZ);
        HAL_Delay(1);
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET)
        {
            stop_pan(0);
            __disable_irq();
            motors[AXIS_PAN].abs_position = 0;
            right_current_step = 0;
            __enable_irq();
            snprintf(msg, sizeof(msg), ">> PAN SENSOR LOCKED. pos=0\r\n");
            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            return true;
        }
    }
    stop_pan(0);
    snprintf(msg, sizeof(msg), ">> PAN SENSOR LOCK ERROR\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    return false;
}
