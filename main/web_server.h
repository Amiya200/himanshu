/*
 * web_server.h  —  HTTP control interface
 */
#pragma once
#include "esp_err.h"
#include <stdbool.h>

esp_err_t web_server_start(void);
void      web_server_stop(void);
bool      web_server_tail_state(void);