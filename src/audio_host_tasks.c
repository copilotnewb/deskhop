/* DeskHop wrappers around TinyUSB's Audio Host example application tasks. */
#include "main.h"

/* Avoid including TinyUSB's example app.h here: it declares a
 * led_blinking_task(void) symbol that conflicts with DeskHop's LED task API.
 * These are the only two application entry points DeskHop needs. */
void audio_app_task(void);
void defer_queue_task(void);

void audio_host_task(device_t *state) {
    (void)state;
    audio_app_task();
}

void audio_host_defer_task(device_t *state) {
    (void)state;
    defer_queue_task();
}
