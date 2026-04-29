/*
 * This file is part of DeskHop (https://github.com/hrvach/deskhop).
 * Copyright (c) 2026
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * See the file LICENSE for the full license text.
 */

#include "main.h"

static bool macro_keyboard_triggered(const macro_config_t *macro, hid_keyboard_report_t *keyboard) {
    if (macro->trigger_mod_all && ((keyboard->modifier & macro->trigger_mod_all) != macro->trigger_mod_all))
        return false;

    if (macro->trigger_mod_any && ((keyboard->modifier & macro->trigger_mod_any) == 0))
        return false;

    return true;
}

static bool macro_mouse_triggered(const macro_config_t *macro, mouse_values_t *mouse) {
    if (macro->trigger_buttons && ((mouse->buttons & macro->trigger_buttons) != macro->trigger_buttons))
        return false;

    return true;
}

static uint8_t macro_trigger_modifiers(const macro_config_t *macro, hid_keyboard_report_t *keyboard) {
    uint8_t modifiers = macro->trigger_mod_all;

    if (macro->trigger_mod_any)
        modifiers |= keyboard->modifier & macro->trigger_mod_any;

    return modifiers;
}

uint8_t macro_suppressed_modifiers(device_t *state) {
    return state->macro_suppressed_modifiers;
}

static void evaluate_macro_state(device_t *state, uint8_t mouse_buttons, uint8_t *filtered_buttons) {
    hid_keyboard_report_t keyboard;
    uint8_t active = 0;
    uint8_t suppressed = 0;

    combine_kbd_states(state, &keyboard);

    for (int i = 0; i < MACRO_SLOT_COUNT; i++) {
        macro_config_t *macro = &state->config.macros[i];
        uint8_t slot_mask = 1 << i;

        if (!macro->enabled || macro->mode == MACRO_MODE_DISABLED)
            continue;

        mouse_values_t mouse = {.buttons = mouse_buttons};

        if (!macro_keyboard_triggered(macro, &keyboard) || !macro_mouse_triggered(macro, &mouse))
            continue;

        active |= slot_mask;
        suppressed |= macro_trigger_modifiers(macro, &keyboard);
        *filtered_buttons &= ~macro->trigger_buttons;

        if (!(state->macro_active & slot_mask))
            state->macro_next_run[i] = 0;
    }

    if (suppressed != state->macro_suppressed_modifiers) {
        state->macro_suppressed_modifiers = suppressed;
        send_key(&keyboard, state);
    }

    state->macro_active = active;
}

void update_macro_state(device_t *state, mouse_values_t *mouse) {
    uint8_t filtered_buttons = mouse->buttons;

    evaluate_macro_state(state, mouse->buttons, &filtered_buttons);
    mouse->buttons = filtered_buttons;
}

static void queue_macro_mouse_button(device_t *state, macro_config_t *macro, bool pressed) {
    uint8_t buttons = state->mouse_buttons & ~macro->trigger_buttons;
    mouse_report_t report = {
        .buttons = pressed ? (buttons | macro->output_buttons) : buttons,
        .x = state->pointer_x,
        .y = state->pointer_y,
        .mode = ABSOLUTE,
    };

    if (state->relative_mouse || state->gaming_mode) {
        report.mode = RELATIVE;
    }

    output_mouse_report(&report, state);
}

void macro_task(device_t *state) {
    uint64_t now = time_us_64();
    uint8_t filtered_buttons = state->physical_mouse_buttons;

    evaluate_macro_state(state, state->physical_mouse_buttons, &filtered_buttons);

    for (int i = 0; i < MACRO_SLOT_COUNT; i++) {
        uint8_t slot_mask = 1 << i;
        macro_config_t *macro = &state->config.macros[i];

        if (state->macro_buttons_down & slot_mask) {
            if (!(state->macro_active & slot_mask) || now >= state->macro_release_time[i]) {
                queue_macro_mouse_button(state, macro, false);
                state->macro_buttons_down &= ~slot_mask;
            }
            continue;
        }

        if (!(state->macro_active & slot_mask))
            continue;

        if (macro->mode != MACRO_MODE_MOUSE_REPEAT || !macro->output_buttons)
            continue;

        if (now < state->macro_next_run[i])
            continue;

        if (queue_get_level(&state->mouse_queue) > MACRO_MAX_MOUSE_QUEUE_LEVEL) {
            state->macro_next_run[i] = now + _MS(macro->interval_ms ? macro->interval_ms : MACRO_DEFAULT_INTERVAL_MS);
            continue;
        }

        queue_macro_mouse_button(state, macro, true);
        state->macro_buttons_down |= slot_mask;
        state->macro_release_time[i] = now + _MS(MACRO_CLICK_DURATION_MS);
        state->macro_next_run[i] = now + _MS(macro->interval_ms ? macro->interval_ms : MACRO_DEFAULT_INTERVAL_MS);
    }
}
