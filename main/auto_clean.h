#pragma once

/*
 * auto_clean.h — Auto cleaning state machine + path recorder
 */

#include <stdbool.h>
#include "motor.h"

/* ─── Panel grid configuration ───────────────────────────── */
typedef struct {
    int  panel_cols;        /* number of columns in the array     */
    int  panel_rows;        /* number of rows in the array        */
    int  panel_w_cm;        /* width of one panel (cm)            */
    int  panel_h_cm;        /* height of one panel (cm)           */
    int  gap_between_cm;    /* horizontal gap between panels (cm) */
    int  row_gap_cm;        /* vertical gap between rows (cm)     */
    bool wash_enabled;      /* activate pump during clean         */
    bool blow_enabled;      /* activate blower during clean       */
    int  passes;            /* cleaning passes per strip (1–3)    */
} clean_grid_t;

/* ─── State ───────────────────────────────────────────────── */
typedef enum {
    CLEAN_IDLE    = 0,
    CLEAN_RUNNING = 1,
    CLEAN_DONE    = 2,
    CLEAN_ERROR   = 3,
    CLEAN_PAUSED  = 4,
} clean_state_t;

/* ─── Grid API ────────────────────────────────────────────── */
void          auto_clean_set_grid(const clean_grid_t *g);
void          auto_clean_get_grid(clean_grid_t *g);

/* ─── Control ─────────────────────────────────────────────── */
bool          auto_clean_start(void);
void          auto_clean_stop(void);
void          auto_clean_pause(void);
void          auto_clean_resume(void);

/* ─── Status ──────────────────────────────────────────────── */
clean_state_t auto_clean_state(void);
bool          auto_clean_is_running(void);
int           auto_clean_strips_done(void);
int           auto_clean_strips_total(void);
int           auto_clean_pct(void);
int           auto_clean_current_panel(void);
int           auto_clean_current_strip(void);

/* ─── Path recording API ─────────────────────────────────── */
bool auto_path_record_start(void);
void auto_path_record_cmd(motor_dir_t dir);
void auto_path_record_peripheral(bool is_pump, bool on);
void auto_path_record_stop(void);
void auto_path_clear(void);
bool auto_path_play_start(void);

bool auto_path_recording(void);
bool auto_path_playback(void);
int  auto_path_len(void);