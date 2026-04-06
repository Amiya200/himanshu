#pragma once
#include "esp_err.h"

/**
 * @brief Start the HTTP server and register all URI handlers.
 */
esp_err_t web_server_start(void);

/**
 * @brief Stop the HTTP server.
 */
void web_server_stop(void);