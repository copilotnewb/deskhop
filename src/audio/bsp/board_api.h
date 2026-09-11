/* Minimal TinyUSB board API shim for DeskHop's embedded Audio Host app. */
#pragma once

#include <stdbool.h>

/* DeskHop owns its LED; do not let TinyUSB's example change it. */
static inline void board_led_write(bool state) {
    (void)state;
}
