/* DeskHop wrappers around TinyUSB's Audio Host example application tasks. */
#include "main.h"
#include "app.h"

void audio_host_task(device_t *state) {
    (void)state;
    audio_app_task();
}

void audio_host_defer_task(device_t *state) {
    (void)state;
    defer_queue_task();
}
