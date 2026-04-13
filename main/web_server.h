#pragma once

#include "esp_err.h"
#include <stdbool.h>

/* ============================================================
 *  web_server.h  —  HTTP control interface
 * ============================================================ */

esp_err_t web_server_start(void);
void      web_server_stop(void);

/** Returns current tail-light state (for main.c accident handler) */
bool      web_server_tail_state(void);