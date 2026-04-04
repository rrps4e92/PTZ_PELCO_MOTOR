/*
 * home_cmd.h
 *
 *  Created on: 13-Jun-2025
 *      Author: Vedant Khamkar
 */

#ifndef INC_HOMING_H_
#define INC_HOMING_H_

#include <main.h>
#include <stdint.h>
#include <stdbool.h>

#define TILT_REAL_HOME_STEPS    30000
#define PAN_STEPS_PER_REV       6400
#define MAX_PRESETS             250
#define SLOT_HALF_STEPS     450     // ~7° — tunable. Full slot ~711–853 steps
#define PAN_HOME_BACKOFF_HZ 1500
#define PAN_HOME_SLOW_HZ    1200

typedef struct {
    int32_t  pan;       /* abs_position at save time */
    int32_t  tilt;      /* abs_position at save time */
    int32_t  zoom;      /* future — reserved 0 for now */
    bool     valid;
} preset_t;

extern preset_t preset_table[MAX_PRESETS];
extern volatile uint8_t home_request;
extern volatile uint8_t home_request;

void process_home_command(void);
void goto_home_position(void);  /* Preset operations */
void preset_save(uint8_t id);
void preset_goto(uint8_t id);
void preset_clear(uint8_t id);
#endif /* INC_HOMING_H_ */
