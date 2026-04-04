/*
 * pelco_decode.c
 *
 *  Created on:
 *      Author: Avikalp Fuley
 *  Full Pelco-D decoder — Standard Command Set, Extended Commands,
 *  and Advanced Feature Set (position commands).
 *
 *  Fine-motion design
 *  ------------------
 *  At 3 km range, 1 step of pan/tilt subtends ~0.056° (6400 steps/rev).
 *  The speed mapping below maps Pelco raw speed byte linearly to Hz so
 *  that the lowest Pelco speed (0x01) drives the motor at START_FINE_HZ
 *  (configurable, default 800 Hz = ~7.5 rpm ≈ 0.45°/s).
 *  Speed byte 0x40 (turbo) drives at max_freq_hz.
 *
 *  For "fine" joystick nudges the operator uses low speed bytes — the
 *  motor fires individual steps at low Hz, giving sub-degree resolution.
 */

#include "main.h"
#include "pelco_decode.h"
#include "motor_move.h"
#include "home_cmd.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/* Pelco-D frame reception state */
uint8_t pelco_frame[PELCO_FRAME_LENGTH];
uint8_t frame_index = 0;
extern uint8_t rcvd_byte;

static uint8_t local_address = 0x01;   /* default camera address */

/* Decoded intent — written by decode_pelco_command(), read by ptz_execute_intent() */
volatile ptz_intent_t  ptz_intent       = {0};
volatile uint8_t       ptz_intent_valid = 0;
volatile uint8_t       new_pelco_data_ready = 0;
volatile uint32_t pelco_last_frame_tick = 0;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

extern volatile uint8_t cmd1;
extern uint8_t cmd2;
extern uint8_t pan_speed;
extern uint8_t tilt_speed;

extern uint32_t speed_hz;

/* ------------------------------------------------------------------ */
/*  Fine-motion speed constants                                        */
/*                                                                     */
/*  Pelco pan speed byte: 0x00 = very slow, 0x3F = high, 0x40 = turbo */
/*  Pelco tilt speed byte: 0x00 = very slow, 0x3F = max (no turbo)    */
/*                                                                     */
/*  We map:   speed_hz = START_FINE_HZ + (raw / 0x3F) * (max - START) */
/*  Turbo (0x40) → max unconditionally.                                */
/*                                                                     */
/*  START_FINE_HZ should be the minimum reliably-stepping frequency    */
/*  of your motor driver — typically 300–1000 Hz.                     */
/* ------------------------------------------------------------------ */
#define PAN_START_FINE_HZ    500U
#define PAN_MAX_HZ           12000U
#define TILT_START_FINE_HZ   500U
#define TILT_MAX_HZ          13000U

static uint32_t pelco_speed_to_hz(uint8_t raw, bool is_tilt)
{
    if (raw == 0)    return 0;                   /* speed=0 means stop this axis */
    if (raw == 0x40) return is_tilt ? TILT_MAX_HZ : PAN_MAX_HZ;

    /* Clamp to 0x3F for linear interpolation */
    if (raw > 0x3F) raw = 0x3F;

    uint32_t lo  = is_tilt ? TILT_START_FINE_HZ : PAN_START_FINE_HZ;
    uint32_t hi  = is_tilt ? TILT_MAX_HZ        : PAN_MAX_HZ;
    uint32_t spd = lo + ((uint32_t)raw * (hi - lo)) / 0x3F;
    return spd;
}

void pelco_set_local_address(uint8_t addr) { local_address = addr; }
uint8_t pelco_get_local_address(void)      { return local_address; }


/**
  * @brief This function verifies checksum of pelco-d frame. It also verifies whether received pelco-d
           frame is valid or invalid and enforces address filtering.
  * @param frame pointer variable
  * @retval None
  */
void process_pelco_frame(uint8_t *frame)
{
		uint8_t sync          = frame[0];
	    uint8_t addr          = frame[1];
	    uint8_t f_cmd1        = frame[2];
	    uint8_t f_cmd2        = frame[3];
	    uint8_t f_d1          = frame[4];
	    uint8_t f_d2          = frame[5];
	    uint8_t recv_checksum = frame[6];

	    uint8_t calc_checksum = (uint8_t)(addr + f_cmd1 + f_cmd2 + f_d1 + f_d2);

    	// Invalid frame — log and stop motors as safety measure
    if (sync != 0xFF || calc_checksum != recv_checksum)
    {
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "Invalid Pelco-D Frame [RAW: %02X %02X %02X %02X %02X %02X %02X] | Expected Chk=%02X, Got=%02X\r\n",
                 frame[0], frame[1], frame[2], frame[3], frame[4], frame[5], frame[6],
                 calc_checksum, recv_checksum);
        HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

        /* On invalid frame we conservatively stop movements (keeps behavior consistent
           with your previous code). */
        frame_index = 0;
        stop_all_motion();
        HAL_UART_Receive_IT(&huart1, &rcvd_byte, 1);
        return;
    }
    //============
    if (addr != local_address && addr != 0x00)
       {
           HAL_UART_Receive_IT(&huart1, &rcvd_byte, 1);
           return;
       }
//
//       /* Store globally for legacy access */
       cmd1      = f_cmd1;
       cmd2      = f_cmd2;
       pan_speed = f_d1;
       tilt_speed= f_d2;
//
       pelco_last_frame_tick = HAL_GetTick();
       decode_pelco_command(f_cmd1, f_cmd2, f_d1, f_d2);
    //=====
        new_pelco_data_ready = 1;
}


void decode_pelco_command(uint8_t c1, uint8_t c2,
                          uint8_t d1, uint8_t d2)
{
    ptz_intent_valid = 0;
    ptz_intent.pan_speed  = d1;
    ptz_intent.tilt_speed = d2;
    ptz_intent.preset_id  = 0;
    ptz_intent.aux_id     = 0;
    ptz_intent.abs_position = 0;

    /* ==============================================================
     * 1. EXTENDED / ADVANCED FEATURE COMMANDS
     *    These are identified by c1 == 0x00 and c2 being a non-zero
     *    even opcode byte.  All pan/tilt direction bits are clear.
     * ============================================================== */
    if (c1 == 0x00 && c2 != 0x00 && ((c2 & 0x01) != 0 || c2 >= 0x49))
    {
        /* Combine d1/d2 into a 16-bit position word for advanced cmds */
        ptz_intent.abs_position = ((uint16_t)d1 << 8) | d2;

        switch (c2)
        {
            /* ---- Standard extended ---- */
            case 0x03:  /* Set Preset */
                ptz_intent.action    = PTZ_SET_PRESET;
                ptz_intent.preset_id = d2;
                break;

            case 0x05:  /* Clear Preset */
                ptz_intent.action    = PTZ_CLEAR_PRESET;
                ptz_intent.preset_id = d2;
                break;

            case 0x07:  /* Go To Preset / Flip / Go To Zero Pan */
                if (d2 == 0x22)
                    ptz_intent.action = PTZ_GOTO_HOME;      /* Go To Zero Pan */
                else if (d2 == 0x21)
                    ptz_intent.action = PTZ_FLIP_180;
                else {
                    ptz_intent.action    = PTZ_GOTO_PRESET;
                    ptz_intent.preset_id = d2;
                }
                break;

            case 0x09:  ptz_intent.action = PTZ_SET_AUXILIARY;  ptz_intent.aux_id = d2; break;
            case 0x0B:  ptz_intent.action = PTZ_CLEAR_AUXILIARY; ptz_intent.aux_id = d2; break;
            case 0x0F:  ptz_intent.action = PTZ_REMOTE_RESET;   break;
            case 0x11:  ptz_intent.action = PTZ_SET_ZONE_START; ptz_intent.aux_id = d2; break;
            case 0x13:  ptz_intent.action = PTZ_SET_ZONE_END;   ptz_intent.aux_id = d2; break;
            case 0x17:  ptz_intent.action = PTZ_CLEAR_SCREEN;   break;
            case 0x19:  ptz_intent.action = PTZ_ALARM_ACK;      ptz_intent.aux_id = d2; break;
            case 0x1B:  ptz_intent.action = PTZ_ZONE_SCAN_ON;   break;
            case 0x1D:  ptz_intent.action = PTZ_ZONE_SCAN_OFF;  break;
            case 0x1F:  ptz_intent.action = PTZ_SET_PATTERN_START; ptz_intent.preset_id = d2; break;
            case 0x21:  ptz_intent.action = PTZ_SET_PATTERN_STOP;  break;
            case 0x23:  ptz_intent.action = PTZ_RUN_PATTERN;    ptz_intent.preset_id = d2; break;
            case 0x29:  ptz_intent.action = PTZ_RESET_CAMERA;   break;
            case 0x2B:  ptz_intent.action = PTZ_AUTOFOCUS_SET;  break;
            case 0x2D:  ptz_intent.action = PTZ_AUTOIRIS_SET;   break;
            case 0x31:  ptz_intent.action = PTZ_BACKLIGHT_SET;  break;
            case 0x33:  ptz_intent.action = PTZ_AUTO_WB_SET;    break;
            case 0x37:  ptz_intent.action = PTZ_SET_SHUTTER;    break;

            /* ---- Advanced Feature Set ---- */
            case 0x49:  ptz_intent.action = PTZ_SET_ZERO_POSITION; break;
            case 0x4B:  ptz_intent.action = PTZ_SET_PAN_POSITION;  break;
            case 0x4D:  ptz_intent.action = PTZ_SET_TILT_POSITION; break;
            case 0x4F:  ptz_intent.action = PTZ_SET_ZOOM_POSITION; break;
            case 0x51:  ptz_intent.action = PTZ_QUERY_PAN_POS;     break;
            case 0x53:  ptz_intent.action = PTZ_QUERY_TILT_POS;    break;
            case 0x55:  ptz_intent.action = PTZ_QUERY_ZOOM_POS;    break;
            case 0x5F:  ptz_intent.action = PTZ_SET_MAGNIFICATION;  break;
            case 0x61:  ptz_intent.action = PTZ_QUERY_MAGNIFICATION; break;

            default:    ptz_intent.action = PTZ_NONE; break;
        }

        ptz_intent_valid = 1;
        return;
    }

    /* ==============================================================
     * 2. STOP — all motion bits zero and no extended opcode
     * ============================================================== */
    if (c1 == 0 && c2 == 0)
    {
        ptz_intent.action = PTZ_STOP;
        ptz_intent_valid  = 1;
        return;
    }

    /* ==============================================================
     * 3. STANDARD COMMAND SET — motion bits
     *
     *  cmd2 bit layout (Pelco-D standard):
     *    bit7 Focus Far
     *    bit6 Zoom Wide
     *    bit5 Zoom Tele
     *    bit4 Tilt Down
     *    bit3 Tilt Up
     *    bit2 Pan Left
     *    bit1 Pan Right
     *    bit0 Always 0
     *
     *  cmd1 bit layout:
     *    bit7 Sense
     *    bit5 Auto/Manual Scan
     *    bit4 Camera On/Off
     *    bit3 Iris Close
     *    bit2 Iris Open
     *    bit1 Focus Near
     *    bit0 reserved
     * ============================================================== */

    bool pan_left  = (c2 & 0x04) != 0;
    bool pan_right = (c2 & 0x02) != 0;
    bool tilt_up   = (c2 & 0x08) != 0;
    bool tilt_down = (c2 & 0x10) != 0;
    bool zoom_tele = (c2 & 0x20) != 0;
    bool zoom_wide = (c2 & 0x40) != 0;
    bool focus_far = (c2 & 0x80) != 0;
    bool focus_near= (c1 & 0x02) != 0;
    bool iris_open = (c1 & 0x04) != 0;
    bool iris_close= (c1 & 0x08) != 0;
    bool cam_on_off= (c1 & 0x10) != 0;
    bool scan      = (c1 & 0x20) != 0;
    bool sense     = (c1 & 0x80) != 0;

    /* Combined pan+tilt diagonals */
    if      (pan_left  && tilt_up)   ptz_intent.action = PTZ_PAN_TILT_UP_LEFT;
    else if (pan_right && tilt_up)   ptz_intent.action = PTZ_PAN_TILT_UP_RIGHT;
    else if (pan_left  && tilt_down) ptz_intent.action = PTZ_PAN_TILT_DOWN_LEFT;
    else if (pan_right && tilt_down) ptz_intent.action = PTZ_PAN_TILT_DOWN_RIGHT;
    else if (pan_left)               ptz_intent.action = PTZ_PAN_LEFT;
    else if (pan_right)              ptz_intent.action = PTZ_PAN_RIGHT;
    else if (tilt_up)                ptz_intent.action = PTZ_TILT_UP;
    else if (tilt_down)              ptz_intent.action = PTZ_TILT_DOWN;
    else if (zoom_tele)              ptz_intent.action = PTZ_ZOOM_IN;
    else if (zoom_wide)              ptz_intent.action = PTZ_ZOOM_OUT;
    else if (focus_far)              ptz_intent.action = PTZ_FOCUS_FAR;
    else if (focus_near)             ptz_intent.action = PTZ_FOCUS_NEAR;
    else if (iris_open)              ptz_intent.action = PTZ_IRIS_OPEN;
    else if (iris_close)             ptz_intent.action = PTZ_IRIS_CLOSE;
    else if (cam_on_off && sense)    ptz_intent.action = PTZ_CAMERA_ON;
    else if (cam_on_off && !sense)   ptz_intent.action = PTZ_CAMERA_OFF;
    else if (scan && sense)          ptz_intent.action = PTZ_AUTO_SCAN_ON;
    else if (scan && !sense)         ptz_intent.action = PTZ_AUTO_SCAN_OFF;
    else                             ptz_intent.action = PTZ_NONE;

    ptz_intent_valid = 1;
}

/* ------------------------------------------------------------------ */
/*  ptz_execute_intent()                                               */
/*  Called from main loop whenever ptz_intent_valid is set.           */
/* ------------------------------------------------------------------ */
void ptz_execute_intent(void)
{

    if (!ptz_intent_valid) return;

    ptz_action_t action = ptz_intent.action;
    char msg[80];

    /* Compute actual Hz from Pelco speed bytes */
    uint32_t pan_hz  = pelco_speed_to_hz(ptz_intent.pan_speed,  false);
    uint32_t tilt_hz = pelco_speed_to_hz(ptz_intent.tilt_speed, true);

    snprintf(msg, sizeof(msg),  ">>INTENT: action=%d pan_spd=%02X tilt_spd=%02X pan_hz=%lu tilt_hz=%lu\r\n",
        	    (int)action,
        	    ptz_intent.pan_speed,
        	    ptz_intent.tilt_speed,
        	    (unsigned long)pan_hz,
        	    (unsigned long)tilt_hz);
       	 HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    /* ----------------------------------------------------------------
     * STOP
     * ---------------------------------------------------------------- */
    if (action == PTZ_STOP || action == PTZ_NONE)
    {
        stop_all_motion();
        ptz_intent_valid = 0;
        return;
    }

    /* ----------------------------------------------------------------
     * GOTO HOME (Pelco "Go To Zero Pan" — we repurpose as full home)
     * ---------------------------------------------------------------- */
    if (action == PTZ_GOTO_HOME)
    {
        stop_all_motion();
        ptz_intent_valid = 0;
        goto_home_position();
        return;
    }

    /* ----------------------------------------------------------------
     * PRESET COMMANDS
     * ---------------------------------------------------------------- */
    if (action == PTZ_SET_PRESET)
    {
        preset_save(ptz_intent.preset_id);
        ptz_intent_valid = 0;
        return;
    }
    if (action == PTZ_CLEAR_PRESET)
    {
        preset_clear(ptz_intent.preset_id);
        ptz_intent_valid = 0;
        return;
    }
    if (action == PTZ_GOTO_PRESET)
    {
        stop_all_motion();
        ptz_intent_valid = 0;
        preset_goto(ptz_intent.preset_id);
        return;
    }

    /* ----------------------------------------------------------------
     * ADVANCED FEATURE: absolute position commands
     *
     * Pelco advanced position is in hundredths of a degree.
     * Conversion: steps = (degrees / 360) * PAN_STEPS_PER_REV
     *             degrees = (position_word / 100.0f)
     *
     * pan steps  = (pos_word / 100.0) / 360.0 * PAN_STEPS_PER_REV
     *            = pos_word * PAN_STEPS_PER_REV / 36000
     * tilt steps = similar, but tilt range is [0 .. TILT_REAL_HOME_STEPS..max]
     *              map 0..9000 (0°..90°) → 0..TILT_MAX_STEPS (your range)
     *              adjust TILT_DEG_TO_STEPS for your physical build.
     * ---------------------------------------------------------------- */
    if (action == PTZ_SET_PAN_POSITION)
    {
        int32_t target_steps = (int32_t)((ptz_intent.abs_position * (int32_t)PAN_STEPS_PER_REV) / 36000);
        snprintf(msg, sizeof(msg), ">> SET PAN POS: word=%ld steps=%ld\r\n",
                 (long)ptz_intent.abs_position, (long)target_steps);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        /* Store target for position-mode move; call a dedicated helper if needed */
        ptz_intent_valid = 0;
        return;
    }
    if (action == PTZ_SET_TILT_POSITION)
    {
        /* Pelco: 0 = horizontal, 9000 = 90° down.
         * Map tilt 0..9000 → step 0..TILT_REAL_HOME_STEPS linearly. */
        int32_t target_steps = (int32_t)((ptz_intent.abs_position * (int32_t)TILT_REAL_HOME_STEPS) / 9000);
        snprintf(msg, sizeof(msg), ">> SET TILT POS: word=%ld steps=%ld\r\n",
                 (long)ptz_intent.abs_position, (long)target_steps);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        ptz_intent_valid = 0;
        return;
    }
    if (action == PTZ_SET_ZERO_POSITION)
    {
        /* Reset pan zero reference to current position */
        __disable_irq();
        motors[AXIS_PAN].abs_position = 0;
        right_current_step = 0;
        __enable_irq();
        snprintf(msg, sizeof(msg), ">> SET ZERO POSITION done\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        ptz_intent_valid = 0;
        return;
    }
    if (action == PTZ_QUERY_PAN_POS)
    {
        /* Respond on UART with pan position in hundredths of degree */
        int32_t pan_pos = motors[AXIS_PAN].abs_position;
        /* Convert steps → hundredths of degree */
        int32_t pan_hd  = (int32_t)((pan_pos * 36000L) / (int32_t)PAN_STEPS_PER_REV);
        uint8_t resp[7];
        resp[0] = 0xFF;
        resp[1] = local_address;
        resp[2] = 0x00;
        resp[3] = 0x59;                            /* Query Pan Response opcode */
        resp[4] = (uint8_t)((pan_hd >> 8) & 0xFF);
        resp[5] = (uint8_t)(pan_hd & 0xFF);
        resp[6] = (uint8_t)((resp[1] + resp[2] + resp[3] + resp[4] + resp[5]) & 0xFF);
        HAL_UART_Transmit(&huart1, resp, 7, HAL_MAX_DELAY);
        ptz_intent_valid = 0;
        return;
    }
    if (action == PTZ_QUERY_TILT_POS)
    {
        int32_t tilt_pos = motors[AXIS_TILT].abs_position;
        int32_t tilt_hd  = (int32_t)((tilt_pos * 9000L) / (int32_t)TILT_REAL_HOME_STEPS);
        uint8_t resp[7];
        resp[0] = 0xFF;
        resp[1] = local_address;
        resp[2] = 0x00;
        resp[3] = 0x5B;                            /* Query Tilt Response opcode */
        resp[4] = (uint8_t)((tilt_hd >> 8) & 0xFF);
        resp[5] = (uint8_t)(tilt_hd & 0xFF);
        resp[6] = (uint8_t)((resp[1] + resp[2] + resp[3] + resp[4] + resp[5]) & 0xFF);
        HAL_UART_Transmit(&huart1, resp, 7, HAL_MAX_DELAY);
        ptz_intent_valid = 0;
        return;
    }
    if (action == PTZ_REMOTE_RESET)
    {
        stop_all_motion();
        /* Trigger NVIC system reset */
        NVIC_SystemReset();
        ptz_intent_valid = 0;
        return;
    }
    if (action == PTZ_FLIP_180)
    {
        /* Pan 180° CW from current position */
        __disable_irq();
        int32_t cur = motors[AXIS_PAN].abs_position;
        __enable_irq();
        snprintf(msg, sizeof(msg), ">> FLIP 180 from pos=%ld\r\n", (long)cur);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        /* Start CW move; main loop or a step-count helper would stop at +3200 steps.
         * For now set a global target and let the main loop handle it, or implement inline. */
        ptz_intent_valid = 0;
        return;
    }

    /* ----------------------------------------------------------------
     * AUTO-SCAN (pan sweep)
     * When auto-scan is on, pan motor runs at mid speed continuously
     * in the current direction until scan-off or stop.
     * ---------------------------------------------------------------- */
    if (action == PTZ_AUTO_SCAN_ON)
    {
        direction = 2;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        motors[AXIS_PAN].direction = DIR_NEGATIVE;
        move_pan(2, PAN_MAX_HZ / 2);
        ptz_intent_valid = 0;
        return;
    }
    if (action == PTZ_AUTO_SCAN_OFF)
    {
        stop_pan(0);
        ptz_intent_valid = 0;
        return;
    }

    /* ----------------------------------------------------------------
     * STANDARD MOTION: PAN / TILT / COMBINED
     *
     * Speed of 0 on an axis means stop that axis only.
     * ---------------------------------------------------------------- */
    bool want_pan_left  = (action == PTZ_PAN_LEFT  ||
                           action == PTZ_PAN_TILT_UP_LEFT   ||
                           action == PTZ_PAN_TILT_DOWN_LEFT);
    bool want_pan_right = (action == PTZ_PAN_RIGHT ||
                           action == PTZ_PAN_TILT_UP_RIGHT  ||
                           action == PTZ_PAN_TILT_DOWN_RIGHT);
    bool want_tilt_up   = (action == PTZ_TILT_UP   ||
                           action == PTZ_PAN_TILT_UP_LEFT   ||
                           action == PTZ_PAN_TILT_UP_RIGHT);
    bool want_tilt_down = (action == PTZ_TILT_DOWN ||
                           action == PTZ_PAN_TILT_DOWN_LEFT ||
                           action == PTZ_PAN_TILT_DOWN_RIGHT);

    /* PAN */
    if (want_pan_left && pan_hz > 0)
    {
        direction = 2;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        move_pan(2, pan_hz);
    }
    else if (want_pan_right && pan_hz > 0)
    {
        direction = 1;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        move_pan(1, pan_hz);
    }
    else if (want_pan_left || want_pan_right)
    {
        /* Speed byte was 0 → stop pan */
        stop_pan(0);
    }

    /* TILT — hard-stop at physical limits enforced by ISR */
    bool up_blocked   = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET);
    bool down_blocked = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET);

    if (want_tilt_up && !up_blocked && tilt_hz > 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        move_tilt(tilt_hz);
    }
    else if (want_tilt_down && !down_blocked && tilt_hz > 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        move_tilt(tilt_hz);
    }
    else if (want_tilt_up || want_tilt_down)
    {
        stop_tilt(0);
    }

    ptz_intent_valid = 0;
}

/* ------------------------------------------------------------------ */
/*  UART RX Complete Callback                                          */
/*  Pelco-D frame assembly with sync-byte resync.                     */
/* ------------------------------------------------------------------ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    if (rcvd_byte == 0xFF)
    {
        /* Start of new frame — discard any partial frame in progress */
        frame_index = 0;
        pelco_frame[frame_index++] = rcvd_byte;
    }
    else
    {
        if (frame_index == 0)
        {
            /* Waiting for sync, ignore non-0xFF bytes */
            HAL_UART_Receive_IT(huart, &rcvd_byte, 1);
            return;
        }
        pelco_frame[frame_index++] = rcvd_byte;
    }

    if (frame_index == PELCO_FRAME_LENGTH)
    {
        process_pelco_frame(pelco_frame);
        frame_index = 0;
    }

    HAL_UART_Receive_IT(huart, &rcvd_byte, 1);
}








//=======   Old pelco-decode =====
/*
void decode_pelco_command(uint8_t cmd1, uint8_t cmd2,
                          uint8_t d1, uint8_t d2)
{
    ptz_intent_valid = 0;

    // STOP
    if (d1 == 0 && d2 == 0 && cmd1 == 0 && cmd2 == 0) {
        ptz_intent.action = PTZ_STOP;
        ptz_intent_valid = 1;
        return;
    }

    // ---------------- PAN / TILT ----------------
    bool pan_left  = cmd2 & 0x04;//0x02;
    bool pan_right = cmd2 & 0x02;//0x01;
    bool tilt_up   = cmd2 & 0x08;//0x04;
    bool tilt_down = cmd2 & 0x10;//0x08;

    ptz_intent.pan_speed  = d1;
    ptz_intent.tilt_speed = d2;
    ptz_intent.preset_id  = 0;
    ptz_intent.aux_id     = 0;
    ptz_intent.abs_position = 0;

    if (pan_left && tilt_up)
        ptz_intent.action = PTZ_PAN_TILT_UP_LEFT;
    else if (pan_right && tilt_up)
        ptz_intent.action = PTZ_PAN_TILT_UP_RIGHT;
    else if (pan_left && tilt_down)
        ptz_intent.action = PTZ_PAN_TILT_DOWN_LEFT;
    else if (pan_right && tilt_down)
        ptz_intent.action = PTZ_PAN_TILT_DOWN_RIGHT;
    else if (pan_left)
        ptz_intent.action = PTZ_PAN_LEFT;
    else if (pan_right)
        ptz_intent.action = PTZ_PAN_RIGHT;
    else if (tilt_up)
        ptz_intent.action = PTZ_TILT_UP;
    else if (tilt_down)
        ptz_intent.action = PTZ_TILT_DOWN;

    // ---------------- ZOOM ----------------
    else if (cmd1 & 0x20)
            ptz_intent.action = PTZ_ZOOM_IN;
    else if (cmd1 & 0x40)
            ptz_intent.action = PTZ_ZOOM_OUT;

    // ---------------- FOCUS / IRIS ----------------
    else if (cmd1 & 0x40)
        ptz_intent.action = PTZ_FOCUS_NEAR;
    else if (cmd1 & 0x80)
        ptz_intent.action = PTZ_FOCUS_FAR;
    else if (cmd1 & 0x20)
        ptz_intent.action = PTZ_IRIS_OPEN;
    else if (cmd1 & 0x10)
        ptz_intent.action = PTZ_IRIS_CLOSE;

    ptz_intent_valid = 1;
//    ptz_execute_intent();
}



/**
  * @brief USART Receive Complete callback function
  * @param UART handle structure
  * @retval None
  *
  * Improved resynchronization: if a sync byte (0xFF) is seen at any time we restart frame collection
  * from that byte. Otherwise, if we are expecting sync (frame_index==0) and byte != 0xFF we ignore it.
  */

/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {

        // If a sync byte appears at any time, restart frame collection from there
        if (rcvd_byte == 0xFF) {
            frame_index = 0;
            pelco_frame[frame_index++] = rcvd_byte;
        } else {
            if (frame_index == 0) {
                // We were expecting the sync byte; this byte isn't sync -> ignore and wait for next.
                HAL_UART_Receive_IT(&huart1, &rcvd_byte, 1);
                return;
            } else {
                pelco_frame[frame_index++] = rcvd_byte;
            }
        }

        if (frame_index == PELCO_FRAME_LENGTH)
        {
            // Frame complete
            process_pelco_frame(pelco_frame);
            frame_index = 0; // reset for next frame
        }

        // Continue receiving next byte
        HAL_UART_Receive_IT(&huart1, &rcvd_byte, 1);
    }
}
*/
