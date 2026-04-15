/*
 * auto_clean.h  —  Fully automatic solar panel cleaning engine
 *
 * Boustrophedon (snake) pattern:
 *   Strip 0: FORWARD (top→bottom), DOWN the panel
 *   Lateral shift RIGHT by ROVER_WIDTH_CM using BACKWARD repositioning
 *   Strip 1: FORWARD again (top→bottom)
 *   ... repeat across panel width
 *   After all strips on one panel column, advance to next column of panels
 *
 * Movement rules:
 *   FORWARD  = used for every cleaning sweep (top→bottom)
 *   BACKWARD = used ONLY for lateral repositioning (rewind to top)
 *   LEFT/RIGHT turning = used for heading correction only
 *   DRIFT_LEFT / DRIFT_RIGHT = used for minor straight-line corrections
 *
 * Pump:   ON during all forward cleaning sweeps
 * Blower: ON during forward sweeps, OFF during backward rewind
 */

#pragma once
#include <stdbool.h>

/* ─── Grid configuration (set via web UI / set_grid endpoint) ─ */
typedef struct {
    int  panel_cols;        /* number of panel columns in the array */
    int  panel_rows;        /* number of panel rows in the array */
    int  panel_w_cm;        /* panel width  (lateral, perpendicular to cleaning direction) */
    int  panel_h_cm;        /* panel height (along cleaning direction, top→bottom) */
    int  gap_between_cm;    /* gap between adjacent panels in the array */
    bool wash_enabled;      /* true = pump on during sweeps */
    bool blow_enabled;      /* true = blower on during sweeps */
} clean_grid_t;

/* ─── State ───────────────────────────────────────────────── */
typedef enum {
    CLEAN_IDLE = 0,
    CLEAN_RUNNING,
    CLEAN_PAUSED,
    CLEAN_DONE,
    CLEAN_ERROR,
} clean_state_t;

/* ─── Public API ──────────────────────────────────────────── */

/* Configure the grid (called from web_server set_grid handler) */
void auto_clean_set_grid(const clean_grid_t *g);
void auto_clean_get_grid(clean_grid_t *g);

/* Start / stop */
bool auto_clean_start(void);   /* returns false if already running */
void auto_clean_stop(void);    /* request graceful stop */

/* Query */
clean_state_t auto_clean_state(void);
bool          auto_clean_is_running(void);
int           auto_clean_strips_done(void);   /* strips completed so far */
int           auto_clean_strips_total(void);  /* total strips to clean */
int           auto_clean_pct(void);           /* 0–100 */