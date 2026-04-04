/*
 * pelco_decode.h
 *
 *  Author: Avikalp Fuley
 *
 *  Full Pelco-D decoder — Standard, Extended, and Advanced Feature Set.
 */

#ifndef INC_PELCO_DECODE_H_
#define INC_PELCO_DECODE_H_

#include "main.h"
#include "motor_move.h"
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/*  Frame                                                              */
/* ------------------------------------------------------------------ */
#define PELCO_FRAME_LENGTH  7

extern uint8_t  pelco_frame[PELCO_FRAME_LENGTH];
extern uint8_t  frame_index;
extern uint8_t  rcvd_byte;
extern volatile uint32_t pelco_last_frame_tick;
extern volatile uint8_t new_pelco_data_ready;

/* ------------------------------------------------------------------ */
/*  Action enum — every decoded intent                                 */
/* ------------------------------------------------------------------ */
typedef enum {
    PTZ_NONE = 0,

    /* Basic motion */
    PTZ_PAN_LEFT,
    PTZ_PAN_RIGHT,
    PTZ_TILT_UP,
    PTZ_TILT_DOWN,
    PTZ_PAN_TILT_UP_LEFT,
    PTZ_PAN_TILT_UP_RIGHT,
    PTZ_PAN_TILT_DOWN_LEFT,
    PTZ_PAN_TILT_DOWN_RIGHT,

    /* Zoom / Focus / Iris */
    PTZ_ZOOM_IN,
    PTZ_ZOOM_OUT,
    PTZ_FOCUS_NEAR,
    PTZ_FOCUS_FAR,
    PTZ_IRIS_OPEN,
    PTZ_IRIS_CLOSE,

    /* Aux / Wiper / Light */
    PTZ_WIPER_ON,
    PTZ_WIPER_OFF,
    PTZ_LIGHT_ON,
    PTZ_LIGHT_OFF,

    /* Auto-scan */
    PTZ_AUTO_SCAN_ON,
    PTZ_AUTO_SCAN_OFF,
    PTZ_CAMERA_ON,
    PTZ_CAMERA_OFF,

    /* Extended commands (opcode-based) */
    PTZ_SET_PRESET,
    PTZ_CLEAR_PRESET,
    PTZ_GOTO_PRESET,
    PTZ_GOTO_HOME,          /* opcode 0x07 + data2=0x22  (Go To Zero Pan) */
    PTZ_FLIP_180,           /* opcode 0x07 + data2=0x21 */
    PTZ_SET_AUXILIARY,
    PTZ_CLEAR_AUXILIARY,
    PTZ_REMOTE_RESET,
    PTZ_SET_ZONE_START,
    PTZ_SET_ZONE_END,
    PTZ_ZONE_SCAN_ON,
    PTZ_ZONE_SCAN_OFF,
    PTZ_SET_PATTERN_START,
    PTZ_SET_PATTERN_STOP,
    PTZ_RUN_PATTERN,
    PTZ_CLEAR_SCREEN,
    PTZ_ALARM_ACK,
    PTZ_AUTOFOCUS_SET,
    PTZ_AUTOIRIS_SET,
    PTZ_BACKLIGHT_SET,
    PTZ_AUTO_WB_SET,
    PTZ_SET_SHUTTER,
    PTZ_RESET_CAMERA,

    /* Advanced Feature Set */
    PTZ_SET_ZERO_POSITION,
    PTZ_SET_PAN_POSITION,
    PTZ_SET_TILT_POSITION,
    PTZ_SET_ZOOM_POSITION,
    PTZ_QUERY_PAN_POS,
    PTZ_QUERY_TILT_POS,
    PTZ_QUERY_ZOOM_POS,
    PTZ_SET_MAGNIFICATION,
    PTZ_QUERY_MAGNIFICATION,

    PTZ_STOP
} ptz_action_t;

/* ------------------------------------------------------------------ */
/*  Intent struct — written by decoder, consumed by executor          */
/* ------------------------------------------------------------------ */
typedef struct {
    ptz_action_t action;
    uint8_t      pan_speed;     /* raw Pelco byte 0x00–0x40 */
    uint8_t      tilt_speed;    /* raw Pelco byte 0x00–0x3F */
    uint8_t      preset_id;     /* for preset / pattern commands */
    uint8_t      aux_id;        /* auxiliary / zone id */
    uint16_t     position_msb;  /* for advanced position commands */
    uint16_t     position_lsb;
    int32_t      abs_position;  /* combined 16-bit position word */
} ptz_intent_t;

extern volatile ptz_intent_t ptz_intent;
extern volatile uint8_t      ptz_intent_valid;

/* ------------------------------------------------------------------ */
/*  Live position — continuously updated by ISR, read by telemetry    */
/* ------------------------------------------------------------------ */
extern volatile int32_t  right_current_step;   /* pan  */
extern volatile int32_t  tilt_current_step;    /* tilt */

/* ------------------------------------------------------------------ */
/*  API                                                                */
/* ------------------------------------------------------------------ */
void process_pelco_frame(uint8_t *frame);
void decode_pelco_command(uint8_t cmd1, uint8_t cmd2, uint8_t d1, uint8_t d2);
void ptz_execute_intent(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void pelco_set_local_address(uint8_t addr);
uint8_t pelco_get_local_address(void);

#endif /* INC_PELCO_DECODE_H_ */



//====> Old pelco decode.h <======
/*
#ifndef INC_PELCO_PROTOCOL_H_
#define INC_PELCO_PROTOCOL_H_

#include <main.h>
#include <stdint.h>
#include <string.h>
#include "motor_move.h"
#include <stdbool.h>   // <--- Add this

extern volatile uint8_t new_pelco_data_ready;
extern uint8_t rcvd_byte;
#define PELCO_FRAME_LENGTH 7

typedef enum {
    PTZ_NONE = 0,

    PTZ_PAN_LEFT,
    PTZ_PAN_RIGHT,

    PTZ_TILT_UP,
    PTZ_TILT_DOWN,

    PTZ_PAN_TILT_UP_LEFT,
    PTZ_PAN_TILT_UP_RIGHT,

    PTZ_PAN_TILT_DOWN_LEFT,
    PTZ_PAN_TILT_DOWN_RIGHT,

    PTZ_ZOOM_IN,
    PTZ_ZOOM_OUT,

    PTZ_FOCUS_NEAR,
    PTZ_FOCUS_FAR,

    PTZ_IRIS_OPEN,
    PTZ_IRIS_CLOSE,

    PTZ_WIPER_ON,
    PTZ_WIPER_OFF,

    PTZ_LIGHT_ON,
    PTZ_LIGHT_OFF,

    PTZ_GOTO_HOME,
    PTZ_PRESET_SET,
    PTZ_PRESET_GOTO,

    PTZ_STOP
} ptz_action_t;

typedef struct {
    ptz_action_t action;
    uint8_t pan_speed;    // raw Pelco speed byte
    uint8_t tilt_speed;
    uint8_t preset_id;    // for preset commands
} ptz_intent_t;

extern volatile ptz_intent_t ptz_intent;
extern volatile uint8_t ptz_intent_valid;
void pelco_set_local_address(uint8_t addr);
void process_pelco_frame(uint8_t *frame);
//void interpret_pelco_movement(uint8_t cmd2, uint8_t d1, uint8_t d2);
void decode_pelco_command(uint8_t cmd1, uint8_t cmd2, uint8_t d1, uint8_t d2);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
uint8_t pelco_get_local_address(void);
void pelco_enable_broadcast(bool enable, uint8_t baddr);

#endif /* INC_PELCO_PROTOCOL_H_ */
