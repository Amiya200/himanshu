#pragma once

/**
 * @brief Initialize Telegram module (creates send task/queue).
 */
void telegram_init(void);

/**
 * @brief Enqueue a message to send via Telegram.
 *        Non-blocking — message is copied into an internal queue.
 *        If WiFi is unavailable the message is discarded gracefully.
 */
void telegram_send(const char *message);