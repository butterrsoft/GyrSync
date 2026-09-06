#ifndef CSYNC_EVSYS_H
#define CSYNC_EVSYS_H

#include <stdint.h>

/* THE CHANNEL MAP. Five channels used, one free.
 *
 * RE-DERIVED FROM evsys_init(), NOT CARRIED FORWARD. The previous
 * version of this table was wrong on four of its six lines: it still
 * described the pre-rotation LUT assignment, and it named CHANNEL1 as
 * the mode-valid gate long after the gate became a choice of truth
 * table. A map that a reader trusts and that disagrees with the code
 * below it is worse than no map, so this one is written from the
 * register writes rather than from what the map used to say.
 *
 *   CHANNEL0  Hsync, PORTC EVGEN0   -> TCB0 capture (the line period)
 *                                      LUT2 IN1  (H into the combiner)
 *                                      LUT1 IN0  (the restart edge sense)
 *                                      LUT0 IN2  (the divider's clock)
 *   CHANNEL1  unused
 *   CHANNEL2  LUT0 out              -> TCB1 capture, Mode S's reshaper.
 *                                      LUT0 rather than LUT2 because
 *                                      SEQ0's output appears at the EVEN
 *                                      LUT of the pair.
 *   CHANNEL3  LUT1 out              -> TCE0 restart, the H offset
 *   CHANNEL4  what the COMBINER sees as Vsync -> LUT2 IN0
 *   CHANNEL5  the RAW Vsync                   -> LUT0 IN0, TCB1 trigger,
 *                                                EVOUTD (pin 13)
 *
 * 4 and 5 are the pair that move at run time, and CHANNEL5's SOURCE
 * moves too -- it is the Vsync pin normally and pin 13 while Mode V
 * regenerates. Everything else is set once in evsys_init() and never
 * again.
 *
 * CHANNEL1 IS GENUINELY FREE, and is left so deliberately rather than
 * being reused for something small. It is the only spare routing
 * resource on the part. */

typedef enum {
    EV_TCB1_DIVIDED_HSYNC,   /* CHANNEL2 -- Mode S reshape */
    EV_TCB1_HSYNC,           /* CHANNEL0 -- sync width measurement */
    EV_TCB1_VSYNC_RAW        /* CHANNEL5 -- Mode FO field delay */
} ev_tcb1_src_t;

void evsys_init(void);

/* CHANNEL4 and CHANNEL5, together, from one call. Both arguments are
 * passed rather than read from globals, so a caller cannot route the
 * pair from a state this module happens to disagree with.
 *
 * The V input's sense may change as a result, so the caller still owns
 * setting pol_pending -- that is a polarity concern, not an event one,
 * and burying it here would hide it from the code that acts on it. */
/* v_out asks for the Vsync level to appear on pin 13 via EVOUTD.
 *
 * IT IS A REQUEST, NOT A COMMAND, and this function may refuse it. When
 * regen is set, CHANNEL5 is pointed at pin 13's OWN level, so an EVOUTD
 * driven from that channel would be a combinational latch: PD7 into
 * PORTD EVGEN1 into CHANNEL5 into EVOUTD and back into PD7. The caller
 * is not trusted to sequence around that -- the teardown and the
 * channel switch happen HERE, in that order, in one function, because
 * an ordering rule split across two callers is a rule that holds until
 * someone adds a third. */
void ev_vsync_route(uint8_t regen, uint8_t fo_active, uint8_t v_out);

void ev_tcb1_capture_source(ev_tcb1_src_t s);

#endif
