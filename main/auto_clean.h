#pragma once

#include "motor.h"
#include <stdbool.h>

/* ─── Grid configuration ─────────────────────────────────── */

typedef struct {
    int  panel_cols;        /* number of panels side-by-side horizontally  */
    int  panel_rows;        /* number of panels stacked vertically         */
    int  panel_w_cm;        /* width  of one panel (the axis rover sweeps) */
    int  panel_h_cm;        /* height of one panel (rover steps along it)  */
    int  gap_between_cm;    /* gap between adjacent panels                 */
    bool wash_enabled;      /* activate pump during cleaning               */
    bool blow_enabled;      /* activate blower during cleaning             */
} clean_grid_t;

/* ─── State machine ──────────────────────────────────────── */

typedef enum {
    CLEAN_IDLE    = 0,
    CLEAN_RUNNING = 1,
    CLEAN_DONE    = 2,
    CLEAN_ERROR   = 3,
} clean_state_t;

/* ─── Grid API ───────────────────────────────────────────── */

void          auto_clean_set_grid   (const clean_grid_t *g);
void          auto_clean_get_grid   (clean_grid_t *g);

/* ─── Auto-clean API ─────────────────────────────────────── */

bool          auto_clean_start      (void);
void          auto_clean_stop       (void);

clean_state_t auto_clean_state      (void);
bool          auto_clean_is_running (void);
int           auto_clean_strips_done(void);
int           auto_clean_strips_total(void);
int           auto_clean_pct        (void);

/* ─── Path recording / playback API ─────────────────────── */

/**
 * Start capturing every manual move command into an internal path buffer.
 * Returns false if auto-clean or playback is already running.
 */
bool  auto_path_record_start    (void);

/**
 * Call this from web_server whenever a manual /move command arrives.
 * The recording engine timestamps and stores each segment.
 */
void  auto_path_record_cmd      (motor_dir_t dir);

/**
 * Call this when pump/blower state changes during recording.
 * is_pump=true → pump event, false → blower event.
 */
void  auto_path_record_peripheral(bool is_pump, bool on);

/**
 * Finalise recording.  After this, auto_path_play_start() may be called.
 */
void  auto_path_record_stop     (void);

/**
 * Erase the stored path.
 */
void  auto_path_clear           (void);

/**
 * Start replaying the recorded path.
 * Returns false if no path is stored or something is already running.
 */
bool  auto_path_play_start      (void);

/* State queries */
bool  auto_path_recording       (void);
bool  auto_path_playback        (void);
int   auto_path_len             (void);