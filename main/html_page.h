#pragma once

/**
 * @brief Returns pointer to the static HTML page string.
 *        Stored in RODATA (flash), not RAM.
 */
const char *html_get_page(void);