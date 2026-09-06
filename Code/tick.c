/* TCF0 -- the 1kHz housekeeping timebase. SINGLE OWNER OF TCF0.
 *
 * Nothing outside this file names TCF0. That is the point of the
 * split: the register set and the one variable derived from it are
 * reachable only through now_ticks() and elapsed_since().
 *
 * TCF0 has no alternative PORTMUX routing, so WO0EN/WO1EN staying
 * clear is the only thing keeping it off PORTA -- the DDC lines. That
 * constraint now lives in one file with the register writes it
 * constrains, rather than in a comment several thousand lines away. */

#include "tick.h"

#include <avr/io.h>
#include <avr/interrupt.h>

/* ===================================================================
 * Housekeeping timebase
 * ===================================================================
 *
 * TCF0 in frequency-generation mode. It overflows at 4kHz and the
 * millisecond tick is a division of that -- see the note below the
 * constants. Ticks are counted in software.
 *
 * Section 12 of the spec asks for a TCB here, to avoid both the TCF
 * read erratum and the RTC's loose internal oscillator. Both TCBs are
 * spoken for -- TCB0 measures Hsync, TCB1 carries the one-shots from
 * increment 3 -- so this is the arrangement that satisfies what that
 * asked for rather than how it asked for it. The erratum is that
 * TCF0.CNT may return a wrong value because its high bytes can change
 * after the low bytes are read; counting overflow INTERRUPTS never
 * reads CNT at all, so it cannot bite. And the tick rides the 20MHz
 * clock, so unlike the RTC nothing needs rescaling against a measured
 * oscillator.
 *
 * A 1kHz interrupt against a 15-31kHz sync signal is not a hazard: the
 * sync path is entirely combinational logic in the CCL, so no interrupt
 * latency anywhere in this firmware can glitch the output.
 *
 * 1ms ticks in a uint16_t wrap every 65.536s -- longer than any
 * interval this design measures, the 25s factory-reset hold included,
 * so the usual wraparound-safe subtraction covers everything.
 *
 * qtick_count counts the same overflows undivided, so 250us a count and
 * a 16.384s wrap. Only the field-rate check reads it and the only
 * interval it measures is a single field, so the shorter wrap costs it
 * nothing. See tick.h for why no accessor is offered.
 */
#define TICK_HZ            1000UL

/* TCF0 NOW OVERFLOWS AT 4kHz, NOT 1kHz, AND THE MILLISECOND TICK IS A
 * DIVISION OF IT.
 *
 * The Blink LED has no hardware PWM available to it. TCF0's own
 * outputs reach only PA0/PA1 and stay disabled for it (see tcf0_init);
 * both TCBs are committed, TCB0 to the Hsync capture and TCB1 to the
 * three one-shot jobs; and TCE0's period IS the line, so its output
 * would stop whenever the source did -- which is precisely when the
 * indicator matters most. Software PWM therefore needs a periodic
 * interrupt, and this is the only one in the design that is both
 * regular and unrelated to the sync signal.
 *
 * 4kHz with a 16-level PWM frame gives 250Hz, comfortably above flicker
 * fusion, and divides EXACTLY by four back to the 1kHz tick: 20MHz /
 * 4000 = 5000 with no remainder, and 4000 / 1000 = 4. tick_count
 * therefore counts the same milliseconds it always did, and every hold,
 * strobe and timeout in the UI is unaffected. The asserts below are
 * what keep that true if either rate is ever edited. */
#define PWM_HZ             4000UL
#define TCF_CMP_FOR_PWM    ((F_CPU / PWM_HZ) - 1)    /* 4999 */
#define PWM_PER_TICK       ((uint8_t)(PWM_HZ / TICK_HZ))

_Static_assert(TCF_CMP_FOR_PWM < 0x1000000UL, "TCF CMP exceeds 24 bits");
_Static_assert(F_CPU % PWM_HZ == 0,  "PWM_HZ must divide F_CPU exactly");
_Static_assert(PWM_HZ % TICK_HZ == 0, "PWM_HZ must be a whole multiple of TICK_HZ");

volatile uint16_t tick_count;
volatile uint16_t qtick_count;

/* EXPORTED DELIBERATELY, AND ONLY FOR LEVEL-0 INTERRUPT CONTEXT.
 *
 * now_ticks() double-reads because the main loop can be interrupted
 * between the two bytes. An ISR at level 0 cannot be interrupted by
 * TCF0, which is also level 0, so the plain read is safe there and
 * the double-read would be waste in a path that matters. The Vsync
 * ISR takes the frame interval this way.
 *
 * Declared in tick.h with this condition stated, so a future caller
 * at level 1 or in the main loop has to read the condition before it
 * can reach the variable. */

/* Ordinary priority. The TCB0 capture holds level 1 (see main(), where
 * CPUINT.LVL1VEC is written), so it preempts this ISR properly -- better
 * than the ISR_NOBLOCK this used to carry, because NOBLOCK only
 * re-enables interrupts after the prologue has already run, and it opens
 * the question of an ISR re-entering itself. Level 1 preemption is
 * immediate and has neither drawback.
 *
 * That promotion was described here and in the capture's own comment for
 * some time before it was actually written to LVL1VEC. It is written
 * now. If the capture is ever demoted, both comments and the frame-
 * boundary hand-off through v_frame_req have to move with it. */
ISR(TCF0_INT_vect)
{
    TCF0.INTFLAGS = TCF_OVF_bm;

    /* Every overflow: one PWM sub-tick. Lives in main.c, which owns the
     * Blink LED pin; this file owns TCF0 and nothing else. Passing the
     * work out through a call rather than reaching for the pin here is
     * what keeps both statements true -- LTO inlines it back in, so the
     * split costs nothing at run time. */
    tick_pwm_service();

    /* Every overflow: one quarter-millisecond. Incremented here rather
     * than derived from tick_count and subdiv, because a reader would
     * then have to sample two variables the ISR updates separately and
     * could land between them. One counter, one store. */
    qtick_count++;

    /* Every fourth: one millisecond. */
    static uint8_t subdiv;
    if (++subdiv < PWM_PER_TICK) return;
    subdiv = 0;

    tick_count++;
}

/* One timestamp per loop pass, or none. A mark captured with a fresh
 * read and then compared against a `now` sampled earlier in the same
 * pass gives a negative difference, which wraps to an enormous positive
 * one, and the interval is judged already over -- an intermittent,
 * timing-dependent failure of whatever that interval controls. Every
 * caller below passes the `now` sampled at the top of the loop. */
uint16_t now_ticks(void)
{
    /* Read twice and retry if they disagree, rather than taking a lock.
     * A 16-bit read is two instructions and the tick ISR can land
     * between them, so it does need protecting -- but this used to do
     * it with cli(), and that was the single biggest contributor to
     * genlock jitter. The main loop calls this every pass, so interrupts
     * were disabled for a few hundred nanoseconds a very large fraction
     * of the time, and any Hsync arriving inside one of those windows
     * had its phase sample delayed by exactly that much.
     *
     * The retry loop costs a few cycles more in the common case and
     * blocks nothing. A torn read cannot survive it: if the ISR fires
     * anywhere across either read the two values differ and it goes
     * round again. */
    uint16_t a, b;
    do {
        a = tick_count;
        b = tick_count;
    } while (a != b);
    return a;
}

uint16_t elapsed_since(uint16_t now, uint16_t mark)
{
    return (uint16_t)(now - mark);
}

void tcf0_init(void)
{
    /* WO0EN and WO1EN stay CLEAR, deliberately. TCF0's only routing is
     * PA0/PA1 -- the DDC pins -- and unlike TCE0 there is no alternative
     * to move it to. The waveform generator runs because FRQ mode needs
     * it, but without these bits it reaches no pin. */
    TCF0.CTRLB   = TCF_CLKSEL_CLKPER_gc | TCF_WGMODE_FRQ_gc;
    TCF0.CMP     = TCF_CMP_FOR_PWM;
    TCF0.INTCTRL = TCF_OVF_bm;
    TCF0.CTRLA   = TCF_ENABLE_bm;
}
