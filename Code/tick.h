#ifndef CSYNC_TICK_H
#define CSYNC_TICK_H

#include <stdint.h>

/* The 1kHz tick. tick_count itself is NOT exported: it is volatile,
 * 16-bit and written by an interrupt, so every reader needs the
 * double-read that now_ticks() does. Exporting the variable would be
 * exporting the obligation to remember that. */
/* ONLY safe to read directly from a LEVEL-0 interrupt handler, which
 * cannot be preempted by TCF0. Anywhere else -- the main loop, or a
 * level-1 handler -- use now_ticks(), which double-reads. */
extern volatile uint16_t tick_count;

/* THE QUARTER-MILLISECOND COUNTER. Same variable, undivided: this is
 * every TCF0 overflow rather than every fourth, so one count is 250us.
 *
 * IT EXISTS FOR ONE READER -- the field-rate check, which needs to
 * separate 65Hz from 70Hz. Those are 15.38ms and 14.29ms, only 1.1ms
 * apart, and on a millisecond grid both round to the same pair of
 * integers, so the check could not tell them apart and strobed on the
 * boundary. Quarter-milliseconds put four counts between them.
 *
 * NO now_qticks() IS OFFERED, DELIBERATELY. The same level-0-only rule
 * as tick_count applies and there is no double-reading accessor to fall
 * back on, so this is readable ONLY from a level-0 interrupt handler.
 * The Vsync edge ISR is the sole caller and is level 0. If a main-loop
 * reader is ever wanted, it needs the retry loop now_ticks() uses --
 * write that accessor rather than reaching for this.
 *
 * WRAPS EVERY 16.384s, not 65.536s. That is far shorter than the tick's
 * and it is fine here because the only interval measured against it is
 * one field -- under 25ms, so a wrap can never be missed -- but it
 * means this must not be used for the UI's multi-second holds. */
extern volatile uint16_t qtick_count;

uint16_t now_ticks(void);
uint16_t elapsed_since(uint16_t now, uint16_t mark);
void     tcf0_init(void);

/* DEFINED IN main.c, CALLED FROM THE TCF0 ISR at PWM_HZ -- four times
 * per millisecond tick.
 *
 * The direction of this call is the point. TCF0 has one owner and it is
 * this file; the Blink LED pin has one owner and it is main.c. Software
 * PWM needs both, so the timebase calls out to the pin's owner rather
 * than this file reaching across to the pin, or main.c reaching across
 * to the timer. Anything that runs here is on the interrupt path at
 * 4kHz and must stay short. */
void     tick_pwm_service(void);

#endif
