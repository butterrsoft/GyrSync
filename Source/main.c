/*
 * CSync dongle -- AVR32EB14. Composite sync combiner/converter.
 *
 * Built and verified with avr-gcc 16.1.0 (ZakKemble build), which has
 * avr32eb14/eb20 device support. Clean under -Wall -Wextra; also builds
 * for avr32eb20 unchanged, via `make MCU=avr32eb20`.
 *
 * ONE BUILD. There are no compile-time variants: no EXTRA, no
 * diagnostic switches, no alternative make targets that produce a
 * different image. `make flash` is the whole of it. The diagnostics
 * this file used to carry did their job and were removed with the
 * questions they answered -- and with them the class of fault they kept
 * causing, where a build selected on a command line reached the bench
 * without anything on the part to say which one it was.
 *
 * =====================================================================
 * WHERE THINGS LIVE
 * =====================================================================
 *
 * THE PIN MAP IS IN pins.h, AND ONLY THERE. It used to be duplicated in
 * this comment, and the copy drifted -- it described the pre-rotation
 * wiring, with DDC on pins 4/5 and Csync on pin 9, long after the
 * oscillator took PA0 and moved all of it. A map that disagrees with
 * itself is worse than no map, so there is one.
 *
 *   pins.h    every pin this project touches, named once
 *   clock.h   CLKCTRL, the PLL, the external oscillator
 *   tick.h    TCF0, the 4kHz interrupt and the millisecond tick
 *   evsys.h   all six event channels, and every event user
 *   ccl.h     all four LUTs, every truth table, CCLROUTEA
 *
 * Each of those modules is the SINGLE OWNER of its registers. That is
 * not tidiness: nearly every bug this firmware has had at the hardware
 * level was two writers on one register, and the split is what makes a
 * second writer a link error rather than a bench round.
 *
 * =====================================================================
 * PERIPHERAL ALLOCATION
 * =====================================================================
 *
 *   TCB0   Hsync capture, FRQ mode: one period measurement per line,
 *          restarted by the event system. Level 1 -- the only interrupt
 *          in this design that cannot be late, because Mode V counts
 *          lines and a lost count moves the emitted Vsync.
 *   TCB1   one timer, three jobs, one owner -- see tcb1_reevaluate().
 *          Mode S's pulse reshaping, the Hsync width measurement, and
 *          Mode FO's sub-line field delay.
 *   TCE0   the H offset. Restarted by Hsync in hardware, pulse placed
 *          by CMP0/CMP1 with the 8x high-resolution extension.
 *   TCF0   the 4kHz housekeeping interrupt: the millisecond tick and
 *          the Blink LED's software PWM.
 *   TWI0   the DDC/EDID client, at 0x50 with 0x30 also ACKed.
 *   CCL    the combiner and everything feeding it.
 *   RTC    unused.
 *
 * =====================================================================
 * THE Rev. A0 WRITE-LOSS ERRATUM, AND WHY NO WORK-AROUND APPEARS BELOW
 * =====================================================================
 *
 * A store to an address >= 0x40 followed by a store below 0x40 loses
 * the second write. Everything below 0x40 on this part is VPORTA/C/D/F
 * (0x00-0x17), GPR (0x1C) and the CPU block (0x30, which holds SREG and
 * the stack pointer). Every peripheral this firmware touches lives at
 * or above 0x40 -- PORT is at 0x0400, CCL at 0x01C0, and so on.
 *
 * So the rule is simply: NEVER TOUCH VPORT OR GPR. Use PORTx.OUTSET and
 * PORTx.OUTCLR, which are above the boundary and cannot be affected.
 * Nothing here needs VPORT's single-cycle access anyway -- the sync
 * path is combinational logic in the CCL, not the CPU.
 *
 * The one remaining sub-0x40 write is restoring SREG, which avr-gcc
 * emits as OUT -- itself the erratum's documented work-around. That is
 * a compiler behaviour rather than a guarantee, so `make lst` exists to
 * check it, and it should be re-checked whenever the toolchain changes.
 *
 * Checked on avr-gcc 16.1.0 for this build: every SREG access is
 * OUT 0x3f, CCP is OUT 0x34, the stack pointer is OUT 0x3d/0x3e, and
 * the only ST instructions in the image are the C runtime's .data copy
 * and .bss clear (targeting RAM at 0x7400+) and two inside
 * eeprom_update_byte (targeting the mapped EEPROM at 0x1400+ and
 * NVMCTRL at 0x1000+). Every one of those is at or above 0x40, so the
 * erratum's trigger condition does not occur anywhere in the build.
 */

#define F_CPU 20000000UL
#include "pins.h"
#include "tick.h"
#include "clock.h"
#include "evsys.h"
#include "ccl.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/cpufunc.h>   /* _PROTECTED_WRITE */
#include <string.h>        /* memcpy, for the EDID shadow rebuild */

/* THERE IS NO GENERATED EDID HEADER ANY MORE. edid_data.h is deleted.
 * The six images are the six .bin files, pulled straight into .rodata
 * by the assembler -- see the .incbin block in the DDC/EDID section
 * below, which is also where the build-time size rules live. */

/* ===================================================================
 * Modes
 * =================================================================== */

#define MODE_1  1
#define MODE_2  2
#define MODE_3  3
#define MODE_4  4
#define MODE_5  5
#define MODE_S  6

/* ===================================================================
 * Hsync acceptance windows, in TCB0 counts at CLK_PER
 * ===================================================================
 *
 * The fast and slow windows are carried over from the ATtiny824
 * unchanged. They are about 30% wide in frequency, so a half-percent of
 * oscillator error is nowhere near their edges and there is nothing to
 * recalibrate.
 *
 * Mode 4's window is the exception and is the one thing here that MUST
 * be re-measured on this part. It spans 24450-25450Hz, four percent
 * wide, which is close enough to the oscillator's own error that the
 * ATtiny824 build had to derive it from a bench measurement of that
 * chip -- ~19.897MHz rather than the nominal 20MHz -- before Mode 4
 * would accept a real 24960Hz source reliably.
 *
 * So Mode 4's edges are computed from F_PER_MEASURED rather than
 * written as counts, and nothing else in the file depends on the value.
 *
 * ON THE EXTERNAL OSCILLATOR THIS IS EXACT AND NEEDS NO CALIBRATION.
 * clock_init() runs the PLL from the 20MHz crystal module on pin 4, so
 * CLK_PER is 20MHz to the oscillator's 50ppm -- four hundred times
 * tighter than the four-percent window, and 20000000 is simply right.
 *
 * IT IS NOT EXACT ON THE FALLBACK PATH. If the oscillator is absent or
 * dead, clock_init() resets once and the second boot runs the PLL from
 * OSCHF instead, whose error is comparable to the width of this window
 * -- which is exactly why the ATtiny824 build had to measure its part
 * at ~19.897MHz before Mode 4 would accept a real 24960Hz source. A
 * unit that has fallen back may therefore reject Mode 4 sources it
 * should take. That is a consequence of a missing part rather than
 * something to trim this constant for: with the oscillator fitted,
 * leave it at 20000000.
 */
#define F_PER_MEASURED    20000000UL   /* <-- re-measure on the bench */

/* Boundaries land OUTSIDE the stated range in both directions: floor on
 * the low count, ceiling on the high one. An exactly-computed edge
 * rejects a legitimate source that misses it by a handful of counts. */
#define COUNTS_FLOOR(hz)  ((uint16_t)( F_PER_MEASURED / (hz)))
#define COUNTS_CEIL(hz)   ((uint16_t)((F_PER_MEASURED + (hz) - 1) / (hz)))

#define HSYNC_FAST_MIN_COUNTS   615   /* 32500Hz */
#define HSYNC_FAST_MAX_COUNTS   656   /* 30500Hz */
#define HSYNC_SLOW_MIN_COUNTS  1176   /* 17000Hz */
#define HSYNC_SLOW_MAX_COUNTS  1333   /* 15000Hz */
#define HSYNC_M4_MIN_COUNTS    COUNTS_FLOOR(25450UL)
#define HSYNC_M4_MAX_COUNTS    COUNTS_CEIL(24450UL)

_Static_assert(HSYNC_M4_MIN_COUNTS > HSYNC_FAST_MAX_COUNTS,
               "Mode 4 window overlaps the fast window");
_Static_assert(HSYNC_M4_MAX_COUNTS < HSYNC_SLOW_MIN_COUNTS,
               "Mode 4 window overlaps the slow window");

#define W_FAST 0x01
#define W_SLOW 0x02
#define W_M4   0x04

/* Which windows each mode accepts. One table to check against the mode
 * documentation, rather than a branch chain that recomputes the same
 * three tests and then picks between them. */
static const uint8_t mode_windows[MODE_S + 1] = {
    0,                        /* index 0 unused */
    W_SLOW,                   /* MODE_1  15-17kHz */
    W_FAST,                   /* MODE_2  30.5-32.5kHz */
    W_FAST | W_SLOW,          /* MODE_3  both */
    W_M4,                     /* MODE_4  narrow ~25kHz */
    W_FAST | W_SLOW | W_M4,   /* MODE_5  all three */
    W_FAST | W_SLOW,          /* MODE_S  both; divides the fast one,
                               * from increment 3 */
};

/* ===================================================================
 * Mode S divide-by-2, and the CCL role swap
 * ===================================================================
 *
 * Mode S accepts both the slow and the fast window. On a slow source it
 * passes Hsync through like any other mode. On a fast one it halves it,
 * so an unbidden 31kHz fallback still shows on a 15kHz CRT.
 *
 * THE DIVIDER OUTPUT IS A TIMING REFERENCE, NOT A SYNC SIGNAL. A
 * flip-flop dividing by two produces a 50% square wave -- about 32us
 * high and 32us low at the resulting 15.7kHz -- which no sync separator
 * will lock to. Each rising edge of it therefore fires a one-shot that
 * emits a fixed-width pulse, and that pulse is what reaches the
 * combiner. This is easy to miss because the divider looks perfect on a
 * logic analyser and fails only at the display.
 *
 * RISING EDGES ONLY, which is worth stating because section 4 of the
 * spec says "each of its edges triggers a one-shot". Taken literally
 * that undoes the division: the square wave has 15.7k rising and 15.7k
 * falling edges per second, so firing on both puts 31.5k pulses per
 * second back on the output. TCB1 is left with EVCTRL.EDGE clear, which
 * triggers on positive edges only and gives one pulse per output line.
 *
 * 4.0us is close to the 4.7us analogue standard and comfortably inside
 * what a 15kHz separator expects. It is a fixed count rather than a
 * fraction of the period because at this point the output rate is known
 * to be in the 15-17kHz window by construction.
 *
 * The counter runs at CLK_PER, so one count is 50ns.
 */
#define RESHAPE_PULSE_COUNTS     80          /* 80 x 50ns = 4.0us */

_Static_assert(RESHAPE_PULSE_COUNTS * 50UL < 10000UL,
               "reshape pulse is wider than a sync pulse should ever be");

/* Which shape the CCL is currently built in. The boundary between the
 * two is NOT the mode -- it is Mode S's own fast/slow decision, so the
 * swap happens at the rate threshold rather than at a mode change.
 *
 * REWRITTEN FOR THE ROTATION. This block described the pre-rotation
 * assignment, where LUT1 was the combiner and LUT2 held the two
 * alternating jobs. Since the oscillator took PA0 and DDC took PC3, the
 * only CCL output pin left is PD6, so LUT2 is the OUTPUT STAGE and is
 * the combiner in every role. What alternates is LUT0/LUT1 and LUT3.
 * ccl.c's header carries the full table.
 *
 *   CCL_ROLE_PASSTHRU  LUT2 combines, taking Hsync straight from the
 *                      pin by event. LUT1 is the restart edge sense and
 *                      LUT3 the reconstruction XOR, both built but only
 *                      consulted by TCE0. Every mode except
 *                      Mode-S-on-a-fast-source.
 *   CCL_ROLE_RECON     the same, except LUT2 takes its H input from
 *                      LUT3 by LINK -- the rebuilt, offset pulse
 *                      instead of the pin. Engaged only when the H
 *                      offset is off centre.
 *   CCL_ROLE_DIVIDE    LUT0+LUT1 become SEQ0's J-K, halving a fast
 *                      source; LUT2 takes the reshaped pulse from TCB1
 *                      on IN2; LUT3 is disabled. Mode S on a fast
 *                      source only, and the H offset stands down here
 *                      -- the divider wins because without it there is
 *                      no picture at all, whereas without the offset
 *                      there is merely a mispositioned one.
 */
/* ccl_role_t moved to ccl.h with the module that acts on it. */

/* ===================================================================
 * H offset (increment 4b)
 * ===================================================================
 *
 * The reconstruction is the XOR of TCE0's WO0 and WO1 -- a pulse
 * spanning CMP0 to CMP1, so BOTH its edges come from compare matches
 * and neither is at BOTTOM. That is what earns the high-resolution
 * extension on both edges rather than one, and diag1 confirmed on
 * silicon that the two are stable relative to each other within 10-20ns.
 *
 * ENGAGED ONLY WHEN THE OFFSET IS OFF-CENTRE, and only while the
 * genlock holds lock. At the neutral setting, and any time lock is
 * lost, LUT1 goes back to the pin. A genlock failure therefore costs
 * the offset, not the picture -- which is the right way round for a
 * device whose whole job is to produce Csync.
 *
 * STEP SIZE is derived from the measured period rather than fixed, so a
 * step means the same fraction of a line at any rate, as section 6.4
 * requires. One step is the period in hi-res units divided by 1024:
 * 9 units at 15.7kHz, which is 56ns, about two thirds of a pixel on a
 * 640-wide active area. 114 steps either side of centre is +/-10.1% of
 * a line.
 *
 * The spec asks for +/-12% and this is a little less. The history is
 * that the range came down and the resolution went up together: 61
 * steps of 244ns originally, then 121 of 119ns, then 181 of 56ns at
 * +/-8%, now 229 of 56ns at +/-10% -- the step held while the travel
 * was extended. Sub-pixel throughout, and 229 still fits the byte the
 * setting is stored in.
 *
 * 228 divides exactly by the UI stride of 6, so the sweep lands on both
 * ends AND on centre, where the offset switches itself off.
 */
#define H_OFFSET_STEPS    229
#define H_OFFSET_DEFAULT  114

#define HOFF_STEP_SHIFT   10    /* step = per_hires >> 10 */

/* Steps moved per short press in Mode H. One step is 56ns -- two thirds
 * of a pixel -- which is the right STORED resolution but far too fine to
 * sweep by hand: 180 presses end to end. Six steps is 337ns a press,
 * about four pixels, and 180 divides exactly by 6 so the range lands on
 * both ends AND on centre, where the offset switches itself off.
 *
 * Keeping the stored value fine and the UI coarse means the feel can be
 * retuned without touching the placement arithmetic or the EEPROM
 * format. */
#define HOFF_UI_STRIDE    6

/* MEASURED CHARACTERISTIC: THE NEGATIVE SIDE'S FLOOR WAS THE
 * OSCILLATOR, AND IS NOT ANY MORE. Both halves are worth keeping --
 * the first is why the crystal was fitted, the second is what it did.
 *
 * ON OSCHF. Emitted edge spread against how far after the restart the
 * pulse is placed, measured on Csync with delayed sweep and infinite
 * persistence, source jitter independently confirmed at 5ns:
 *
 *     6.25us delay ->  50ns      (positive offsets live here)
 *    56.25us       -> 100ns
 *    61.5us        -> 135ns      (negative offsets live here)
 *
 * The 50ns is the restart quantising to CLK_PER and is unavoidable. The
 * excess grew with the delay -- about 87ns by 56us, some 1540 ppm of
 * the interval -- and it was not the placement, which is exact, nor the
 * compare registers, which do not move. It was the clock counting the
 * delay out: running CLK_MAIN straight from OSCHF with no PLL gave the
 * same figures, so not the 16x multiplication either, but OSCHF itself
 * -- an internal RC oscillator whose frequency wanders over the 1125
 * cycles a negative offset has to count.
 *
 * That bounded the negative direction on that hardware. Every negative
 * offset, however small, waits nearly a whole line -- there is no
 * benign region near centre -- so the whole of that side sat at
 * 110-135ns while the positive side sat at 50ns.
 *
 * ON THE CRYSTAL, WHICH IS NOW FITTED. A 20MHz oscillator module on
 * pin 4 jitters in picoseconds, so the same walk over 1125 cycles comes
 * to tens of picoseconds against the 87ns above. The negative side now
 * sits at the same 50ns quantisation floor as the positive one, and the
 * asymmetry this block used to describe as shipped is gone.
 *
 * WHAT IT COST, since the note that used to be here said it was not
 * worth paying. EXTCLK exists only on PA0, which was SDA -- so fitting
 * the oscillator moved DDC to PC2/PC3, took PC3 away from the CCL, and
 * forced Csync to PD6 with the LUT roles rotated: LUT2 combining, LUT3
 * reconstructing, LUT1 converting the edge. A board revision and a
 * substantial rewire. It was done, and the roles above are its result.
 *
 * A NOTE ON THE ASYMMETRY, so it is not revisited.
 *
 * The two directions are not symmetric and no machinery makes them so.
 * One way delays sync from an edge that has already arrived, so the
 * pulse lands early in its own line and carries only the 50ns restart
 * quantisation. The other needs sync EARLIER than the edge being
 * referenced, which does not exist -- so the pulse goes near the end of
 * the PREVIOUS line, and the gap to the content it precedes is a whole
 * source line period. Whatever the source's period does line to line
 * lands on the picture. Section 3.2 justified that case on the source
 * being exactly periodic; a real graphics card wanders.
 *
 * Moving the NEUTRAL setting into the delay range makes both directions
 * land in their own line. It was built, measured, and rejected -- for a
 * reason unrelated to jitter. Only the sync is regenerated here; the
 * picture information passes from source to display untouched. So a
 * neutral that is a delay MOVES THE PICTURE by default, and correcting
 * that means adjusting the source's modeline or the display's geometry.
 * If either is being adjusted, the H offset had no reason to exist: it
 * earns its place only by correcting position without touching them.
 *
 * So the neutral stays at zero delay, one direction is stable, and the
 * other carries the source's wander.
 */

/* Narrowest emitted pulse, after any truncation by the next restart.
 *
 * THIS SETS THE NEGATIVE-SIDE DEAD ZONE, and the mechanism is worth
 * stating because it looks like a bug from the outside.
 *
 * The emitted pulse occupies [d, d+w] and the next restart is at
 * per_hires. For it to fit at all, d must not exceed per_hires minus the
 * width that survives. Since d = per_hires + offset, a negative offset
 * smaller than that width leaves no room -- the position is pinned and
 * only the pulse narrows, so the picture does not move. The first few
 * presses off centre therefore do nothing visible.
 *
 * A FULL-WIDTH pulse would need the offset to exceed 4.7us, which is
 * fourteen presses of dead zone. Allowing truncation to 1.2us brings it
 * to under four, which is about the least that still leaves a pulse a
 * sync separator will trigger on reliably. Below roughly 1us the
 * separator starts hunting and the cure is worse than the symptom.
 *
 * It cannot be removed. Placing the pulse earlier than the reference
 * edge means going to the previous line, and there has to be room
 * before the next restart for the pulse to exist in. Only the positive
 * direction is free of it, because there the whole line lies ahead. */
#define HOFF_MIN_WIDTH    (8u * 24u)      /* 1.2us */


/* ===================================================================
 * Line-period bounds
 * ===================================================================
 *
 * WHAT USED TO BE HERE. Increment 4a built a free-running genlock: TCE0
 * kept at the line rate by a feed-forward-plus-PI loop, its phase
 * sampled in the capture ISR, its period steered by dithering PER
 * between two adjacent 50ns steps. It worked, and it was removed -- the
 * loop's own cost was 400ns of emitted edge spread against the 50ns of
 * restart quantisation it existed to avoid. TCE0 is restarted by Hsync
 * in hardware now, and the H offset engine further down carries the
 * whole argument.
 *
 * Fifteen GENLOCK_* constants and about a hundred and twenty lines
 * describing that loop -- dither ratios, lock windows, outlier
 * rejection, why the proportional term was not optional -- outlived it
 * by several increments. THIRTEEN OF THE FIFTEEN WERE REFERENCED BY
 * NOTHING AT ALL, so the file read as though a control loop were still
 * in the signal path. They are gone.
 *
 * THE TWO SURVIVORS WERE NEVER ABOUT GENLOCK, which is why they
 * survived and why they are renamed. They bound the measured line
 * period: anything outside is not a rate this product serves, and
 * clamping stops a transient dragging the running average somewhere it
 * cannot walk back from. period_average() and hoffset_apply() are the
 * only readers. */
#define LINE_PER_MIN           600   /* ~33kHz */
#define LINE_PER_MAX          1400   /* ~14kHz */

/* ===================================================================
 * CCL truth tables
 * ===================================================================
 *
 * LUT1's truth index is IN2:IN1:IN0 = gate:Hsync:Vsync.
 *
 * The output is gate AND XNOR(Vsync, Hsync): high when the two syncs
 * agree, held low -- the muted state -- whenever the gate is low.
 *
 *   index 4 (gate=1, H=0, V=0) -> 1
 *   index 7 (gate=1, H=1, V=1) -> 1        = 0x90
 *
 * Inverting BOTH inputs of an XNOR leaves it unchanged, so the absolute
 * polarity of either sync is irrelevant -- only whether they AGREE.
 * That is why flipping one input's sense covers all four polarity
 * combinations, and why the whole polarity feature is a choice between
 * two truth-table values with no extra LUT, pin, timer or event
 * channel. Inverting one input of an XNOR makes it an XOR:
 *
 *   index 5 (gate=1, H=0, V=1) -> 1
 *   index 6 (gate=1, H=1, V=0) -> 1        = 0x60
 *
 * The output is idle-high and pulses low on either sync in both cases,
 * i.e. standard negative composite sync whatever the source did.
 */


/* --- H offset state ------------------------------------------------ */
static uint8_t  hoffset_running;
static uint8_t  h_offset = H_OFFSET_DEFAULT;
static uint16_t last_width_counts;      /* sync pulse, measured by TCB1 */

/* THE CAPTURE IS BACK IN AN INTERRUPT, and this time for a reason that
 * polling cannot satisfy.
 *
 * Polling was fine while nothing counted lines: a missed capture only
 * delayed a period measurement that was averaged anyway. Mode V counts
 * lines, and every lost capture moves the emitted Vsync -- which is a
 * jittering picture. The same loss also destabilised the frame length
 * and, earlier, glitched the mode-valid pin during DDC reads.
 *
 * An interrupt cannot miss one. It stays minimal: read the period,
 * advance the line, place the pulse if it is due, flag the main loop.
 *
 * NOTE FOR ANYONE REMOVING THIS: the ISR and the INTCTRL enable must
 * arrive and depart TOGETHER. An enabled interrupt with no handler
 * vectors to __bad_interrupt, which jumps to address zero -- the first
 * capture then resets the part, forever, with no other symptom.
 */
/* Lines since the frame boundary. WRITTEN ONLY IN THIS INTERRUPT -- see
 * the note on v_frame_req below. */
static uint16_t v_line;
static uint16_t v_prev_lines;
static void v_line_tick(void);
static void v_frame_start(void);

/* The main loop reads this to decide whether the regeneration may
 * engage at all. It is the only piece of Mode V's state that crosses out
 * of the capture interrupt -- and it is only ever tested against zero,
 * where a torn read of a 16-bit line count cannot turn zero into
 * non-zero or the reverse in any way that matters for one loop pass. So
 * it needs volatile and nothing more. */
static volatile uint16_t v_frame_lines;

/* Frame-referenced line counting, for an interlaced source. Declared
 * here, beside the interrupt that reads it, rather than in the Mode V
 * block that owns everything else.
 *
 * v_ilace is adopted at a frame boundary rather than the moment the
 * main loop decides it, so a change cannot land halfway through a
 * count; v_ilace_want is where the main loop leaves it.
 *
 * v_next_is_mid is a PREDICTION. The Vsync ISR cannot work out a
 * field's parity for itself -- that needs the phase weighed against the
 * line period, which is main-loop work -- so the main loop names the
 * next field instead, on the grounds that fields alternate. */
static uint8_t v_ilace;
static uint8_t v_ilace_want;
static uint8_t v_next_is_mid;

/* THE REFERENCE IS HELD BY ALTERNATION, NOT RE-MEASURED EVERY FIELD.
 *
 * Fields alternate, so once the right one has been identified the next
 * is simply the one after next. Re-deciding it from the measurement
 * every frame looked equivalent and was not: src_track_interlace's
 * mid-line window covers only the middle quarter of a line, and a source
 * whose phase sits near either edge of it will misread the odd field.
 *
 * The interlace DETECTOR survives that -- it has four-alternation
 * hysteresis, which is why the indicator reads steady. The reference
 * selection had none, so one misread field either skipped a reference
 * (the counter ran an extra frame into the runaway guard) or inserted a
 * spare one (the count came back a field short and was rejected). Both
 * destroy the believed frame length, and with it the position of the
 * second pulse -- leaving the two pulses 262 and 263 lines apart on
 * alternate frames instead of 262.5 each. A one-line shift at field
 * rate, which is a picture that jumps, and enough to stop a set locking,
 * which is a picture that rolls.
 *
 * So the measurement is consulted only while ACQUIRING, when there is no
 * frame length to lose. After that the alternation carries it, and a
 * misread field costs nothing. v_bad_frames drops back to acquisition if
 * the alternation itself is ever wrong, which is what a genuinely lost
 * Vsync edge would do. */
static uint8_t v_field_toggle;   /* flips on every field */
static uint8_t v_ref_parity;     /* the alternation carrying the reference */
static uint8_t v_bad_frames;     /* consecutive rejected frame counts */
#define V_BAD_FRAMES_MAX 4

/* WHERE THE REFERENCE IS, RELATIVE TO WHERE IT SHOULD BE, in lines.
 *
 * v_line is zeroed by the first capture that sees v_frame_req set by
 * the Vsync pin interrupt. The capture is level 1 and the Vsync
 * interrupt is level 0, so on a PROGRESSIVE source -- where the Vsync
 * edge is on the line grid and coincident with an Hsync -- the capture
 * can preempt the Vsync interrupt and read the flag before it has been
 * written. The reset is then deferred a whole line, and the emission,
 * which is placed a number of lines after that reset, moves with it.
 *
 * MEASURED, not deduced. A diagnostic build once held pin 11 high for
 * any frame whose line count disagreed with the last, and on the bench
 * it showed a uniform 50ms pulse -- three frames at 16.65ms --
 * coincident with every visible tic in the picture. Three frames is the
 * signature of exactly this: the deferred frame counts L+1, the next
 * counts L-1, and the third is back to L.
 *
 * The interlaced path does not suffer it, because its reference is the
 * MID-LINE field, whose Vsync edge sits half a line from any Hsync.
 * The comment in the capture interrupt says so and is still live; a
 * progressive source simply has no mid-line field to choose.
 *
 * WHAT DECIDES THE RACE IS STILL UNKNOWN. Two candidates have been
 * measured and eliminated: DDC traffic delaying the level 0 interrupt
 * (turning EDID off changes nothing) and source jitter flipping which
 * edge lands first (Hsync jitter measures 5ns, which could only matter
 * if the two edges were already within 5ns of each other). Do not
 * guess at a third here. This accumulator does not need to know: it
 * removes the emission's DEPENDENCE on the attribution rather than
 * fixing the attribution.
 *
 * CUMULATIVE, AND THAT IS THE WHOLE POINT. The per-frame difference is
 * the wrong quantity. A frame counting L+1 says the reference has moved
 * one line late; the L-1 frame that follows says it has moved back, and
 * correcting THAT one would introduce an error where there was none.
 * Only the running total describes where the reference actually is.
 *
 * Clamped because a believed frame length that is wrong by one would
 * otherwise accumulate a line every frame and walk the picture off the
 * screen. With the stability test now requiring exact agreement that
 * should not happen, so the clamp is a bound on the damage rather than
 * a working part -- but it is the difference between a defect and a
 * runaway. */
static int8_t v_slip;
#define V_SLIP_MAX 4

/* A line count this far past any legitimate frame means the reference
 * was missed. On an interlaced source only one field in two resets the
 * counter, so losing that field's edge would let it run until it
 * wrapped. The widest legitimate frame here is a 45Hz field on a 17kHz
 * interlaced source, about 760 lines. */
#define V_LINE_RUNAWAY 1000

/* THE FRAME BOUNDARY IS A REQUEST, NOT AN ACTION, and promoting this
 * capture to level 1 is what forces that.
 *
 * The Vsync interrupt used to reset the line count and compute the
 * emission point itself. At level 0 for both that was safe, because
 * neither interrupt could interrupt the other. At level 1 this one CAN
 * land in the middle of the Vsync ISR, and v_line, v_start_line and
 * v_frame_lines are all 16-bit: a capture arriving between the two
 * stores of a 16-bit write gives a torn read, and a torn v_start_line
 * puts that frame's Vsync somewhere arbitrary. Which is a picture that
 * jumps once every few seconds -- the hardest kind of fault to catch,
 * because it looks like a source problem.
 *
 * Masking this interrupt across the Vsync ISR is the obvious fix and is
 * the WRONG one. The mask would be about 1.5us against a 63.6us line, so
 * one frame in forty loses the very count the mask exists to protect,
 * and the emission point moves by a line -- which is precisely the
 * jitter being chased.
 *
 * So the Vsync ISR sets an 8-bit flag and nothing else, and the frame
 * boundary is done HERE, on the next capture. Every variable Mode V
 * touches then lives in one interrupt context and there is nothing left
 * to tear. The cost is that the count resets on the first Hsync after
 * Vsync rather than at the Vsync edge, which is the resolution the whole
 * scheme works at anyway -- and it is the same quantisation the +/-1
 * line frame-length test already absorbs. */
static volatile uint8_t  v_frame_req;
static volatile uint8_t  v_frame_req_mid;   /* was that field mid-line? */

static volatile uint16_t cap_period;
static volatile uint8_t  cap_ready;

ISR(TCB0_INT_vect)
{
    uint16_t period = TCB0.CCMP;      /* clears CAPT; already restarted */
    cap_period = period;
    cap_ready  = 1;

    if (v_frame_req) {
        v_frame_req = 0;

        /* ON AN INTERLACED SOURCE ONLY ONE OF THE TWO FIELDS RESETS THE
         * COUNTER -- the mid-line one, whose Vsync edge sits squarely
         * half a line from any Hsync, so the capture that follows it is
         * never in doubt. The on-grid field's edge is coincident with an
         * Hsync and the race between them can be decided differently
         * from one frame to the next. Using it as the reference would
         * put a whole line of jitter into the very measurement the
         * reconstruction is built on. */
        if (!v_ilace) {
            v_frame_start();
        } else {
            v_field_toggle ^= 1;

            if (!v_frame_lines) {
                /* Acquiring. Nothing to lose, so take the measurement
                 * and remember which alternation it landed on. */
                if (v_frame_req_mid) {
                    v_ref_parity = v_field_toggle;
                    v_frame_start();
                }
            } else if (v_field_toggle == v_ref_parity) {
                v_frame_start();
            }
        }
    }

    v_line++;

    /* A missed reference must not be allowed to run away. On an
     * interlaced source the counter is reset by one field in two, so
     * losing that field's edge means nothing resets it until the count
     * wraps at 65535 -- by which time the frame length has been rejected
     * for hundreds of frames and the picture is gone. Invalidating the
     * length forces the stability test to re-earn it from scratch. */
    if (v_line > V_LINE_RUNAWAY) {
        v_line        = 0;
        v_prev_lines  = 0;
        v_frame_lines = 0;
    }

    v_line_tick();
}

/* Old note, kept because the reasoning still applies to everything the
 * main loop does with the captured value:
 *
 * It was moved into an interrupt so the genlock could sample TCE0.CNT
 * at a known point after the Hsync edge. Nothing needs that now -- the
 * restart is done by the event system, in hardware -- and the period
 * measurement this feeds is used for window checking and for scaling
 * the offset step, neither of which cares whether a line is missed. */


/* ===================================================================
 * Timing constants, in 1ms ticks
 * =================================================================== */

#define POL_SAMPLE_TICKS         20   /* idle-level sample window.
                                       *
                                       * Polarity is a majority vote on
                                       * a DC property present 98-99% of
                                       * the time, not a hunt for a rare
                                       * event, so it does not need to
                                       * span a frame. 20ms at one
                                       * sample per loop pass is
                                       * thousands of samples with a
                                       * 99:1 margin, and keeping it
                                       * short is what lets the mute
                                       * come down to 40ms. */
#define SETTLE_TICKS_SOURCE     220   /* Csync and the Mode-Valid LED
                                       * muted after the SOURCE changes
                                       * its line rate, as opposed to
                                       * after something this firmware
                                       * did. Long because the mute has
                                       * to outlast a graphics card's
                                       * whole modeline transition --
                                       * measured at 200ms worst case --
                                       * and it retriggers on each step
                                       * within it. Both are muted: the
                                       * LED blinking through a change
                                       * is the reported symptom, the
                                       * output unmuting into a
                                       * transient mode is the part
                                       * that reaches the display. */

#define BOOT_SETTLE_TICKS       250   /* Csync and the Mode-Valid LED
                                       * muted from reset until
                                       * acquisition has finished
                                       * churning. ONE CURTAIN OVER THE
                                       * WHOLE OF IT, rather than a
                                       * settle per event.
                                       *
                                       * THE FAULT THIS CLOSES. A
                                       * profile with the H offset off
                                       * centre strobed the Mode-Valid
                                       * LED at every power-on: two or
                                       * three mutes of about 60ms with
                                       * brief valid windows between
                                       * them, worst at 15kHz. Isolated
                                       * on the bench by centring the
                                       * offset, which removed it.
                                       *
                                       * EVERY ONE OF THOSE MUTES WAS
                                       * INDIVIDUALLY CORRECT. The CCL
                                       * really was being reconfigured
                                       * each time and muting across a
                                       * reconfiguration is right. What
                                       * was wrong is that they arrived
                                       * as a strobe. A display's PLL
                                       * cares far more about sync
                                       * appearing, vanishing and
                                       * reappearing than about sync
                                       * arriving late once, so the fix
                                       * is to hide the churn rather
                                       * than to remove any one settle.
                                       *
                                       * WHAT HAPPENS UNDERNEATH, traced
                                       * with pin 11 over four
                                       * diagnostic builds. The source
                                       * terms never drop -- hsync_valid
                                       * and field_rate_ok are innocent
                                       * and the watchdog never fires.
                                       * h_pin_idle_high moves once, off
                                       * its BSS zero, which is real
                                       * work. pol_flip then moves TWICE
                                       * more, 0->1 at 25ms and 1->0 at
                                       * 85ms, each inside a mute that
                                       * was already up and each
                                       * followed 40ms later by that
                                       * mute lifting. So
                                       * polarity_measure() raises them,
                                       * and want_flip is being computed
                                       * against acquisition state that
                                       * is still moving: the CCL role,
                                       * and fo_active, both of which
                                       * force a term of want_flip and
                                       * both of which settle at their
                                       * own pace.
                                       *
                                       * 250 AGAINST A WORST OBSERVED
                                       * 220. The longest boot churn
                                       * measured was low 0-60, high
                                       * 60-160, low 160-220, at 15kHz
                                       * with the H offset engaged and
                                       * Mode FO disengaged. That is
                                       * 30ms of margin and no more, so
                                       * it is a number to re-measure
                                       * rather than to trust: if any
                                       * event behind this curtain ever
                                       * scales with the source -- the
                                       * interlace detector counts
                                       * FRAMES, for one -- a slower
                                       * source could outrun it, and the
                                       * symptom would return quietly as
                                       * a single late pulse rather than
                                       * as the obvious strobe.
                                       *
                                       * Longer than SETTLE_TICKS_SOURCE
                                       * deliberately: a modeline change
                                       * mid-run has a settled firmware
                                       * underneath it, and boot does
                                       * not. */

#define SETTLE_TICKS             40   /* Csync muted while the firmware
                                       * decides something: the 20ms
                                       * polarity measurement, during
                                       * which the output is still being
                                       * combined with the PREVIOUS
                                       * modeline's polarity, plus the
                                       * rebuild that may follow it. */
#define HSYNC_WATCHDOG_TICKS      3   /* captures stopped arriving.
                                       *
                                       * MUST BE AT LEAST 2, and the
                                       * reason is tick quantisation
                                       * rather than the signal. A tick
                                       * boundary can fall immediately
                                       * after a capture, so an elapsed
                                       * count of N guarantees only
                                       * (N-1) ms of real time. At 1 the
                                       * watchdog therefore fires with
                                       * no time elapsed at all, once
                                       * per tick, forever -- see the
                                       * note in the main loop for what
                                       * that then wrecks.
                                       *
                                       * 3 guarantees at least 2ms, i.e.
                                       * 30 missed lines at 15kHz and 62
                                       * at 31kHz. Sync is gone, not
                                       * late, and a real dropout is
                                       * still caught within 3ms. */
/* TWO CONSTANTS, BECAUSE THERE ARE TWO JOBS.
 *
 * The whole history of this debounce is one constant being asked to do
 * both, and failing whichever way it was turned: long enough to outlast
 * bounce was long enough to swallow a real second press (the merge),
 * short enough to pass real presses was short enough for bounce to look
 * like one (the split). Then trusting a single sample fixed both and
 * let electrical noise in instead. Three failures, one cause.
 *
 * B1_QUALIFY_MS -- NOISE. How long the pin must hold the new level
 * CONTINUOUSLY before the edge is believed. Sampled at loop rate, which
 * is many times per millisecond, so this is thousands of consecutive
 * samples and any glitch shorter than it resets the count. Set to 8,
 * deliberately identical to the old DEBOUNCE_TICKS, because that build
 * ran on the bench without phantom presses and matching it exactly is
 * the only claim about noise that can be made without a scope on the
 * pin. It is NOT a bounce filter and must stay short: it is the floor
 * on the gap between two real presses, and at 8ms no finger can trip it.
 *
 * B1_LOCKOUT_MS -- BOUNCE. Dead time after an edge has been ACCEPTED,
 * during which nothing is looked at. This is what outlasts contact
 * settling, and it can be generous precisely because it no longer has
 * to pass real presses -- it starts after the edge is already believed,
 * so a second press inside it is delayed rather than lost.
 *
 * THE SEPARATION IS THE FIX. Neither number can reintroduce the other's
 * failure, which is what no single value could manage. */
#ifndef B1_QUALIFY_MS
#define B1_QUALIFY_MS             8
#endif
#ifndef B1_LOCKOUT_MS
#define B1_LOCKOUT_MS            25
#endif

/* ===================================================================
 * Held-button timeline (section 8.1 unlocked, 8.1a locked)
 * ===================================================================
 *
 * Each indication says what releasing NOW will do. The two Mode S codes
 * and the gap around them mark the MS window; the fast 150ms blinks mark
 * the lockout window; and the LED going DARK at the end is the last
 * warning before something destructive.
 *
 * The locked timeline is the same shape on a shorter scale, so the
 * gesture is learned once: nothing at all, then fast blinks, then dark.
 */
#define HOLD_PROFILE1_MS      2000     /*  2-4s   profile 1               */
#define HOLD_PROFILE2_MS      4000     /*  4-6s   profile 2               */
#define HOLD_PROFILE3_MS      6000     /*  6-8s   profile 3               */
#define HOLD_MS_ENTER_MS      8000     /*  8-11s  Mode S                  */
#define HOLD_CS_ENTER_MS     11000     /* 11-13s  Mode CS                 */
#define HOLD_ED_ENTER_MS     13000     /* 13-16s  Mode ED                 */
#define HOLD_LOCK_MS         16000     /* 16-25s  button lockout          */
#define HOLD_FACTORY_MS      25000     /* 25s+    factory reset           */

/* THE ED WINDOW IS THREE SECONDS WIDE WHERE THE OTHERS ARE TWO, AND
 * THAT IS DELIBERATE. It sits at the deepest point of the ladder, where
 * nobody is still counting, and the thing immediately above it is the
 * lockout -- the most irritating window on the timeline to land in by
 * accident. Overshooting Mode ED costs a lockout and an unlock gesture;
 * overshooting Mode CS only costs a second attempt. So the margin is
 * spent on the side where a miss is expensive. Lockout still gets four
 * seconds, which is twice what it needs.
 *
 * THE LOCKOUT WINDOW IS NINE SECONDS WIDE, 16s TO 25s, AND FACTORY
 * RESET IS THE ONLY THING BELOW IT. It was 16s to 20s. Widening it
 * costs nothing -- nobody navigates TO the lockout by counting, they
 * arrive at it by watching the fast blinks start -- and it buys the
 * one thing the timeline could not otherwise give: distance in front
 * of the only gesture on the ladder that destroys settings.
 *
 * The blinks run the whole nine seconds, so the warning that something
 * irreversible is close is now the LED going dark at 25s rather than a
 * four-second window a slow hand can cross. Nothing else moves: the
 * locked ladder is a separate, shorter timeline and keeps its 10s.
 *
 * Both new windows are navigable by WATCHING rather than counting:
 * solid through the CS window, dark through the ED window, fast blinks
 * from the lockout. Two crisp transitions land exactly where the
 * counting has broken down. */
_Static_assert(HOLD_PROFILE1_MS < HOLD_PROFILE2_MS
            && HOLD_PROFILE2_MS < HOLD_PROFILE3_MS
            && HOLD_PROFILE3_MS < HOLD_MS_ENTER_MS
            && HOLD_MS_ENTER_MS < HOLD_CS_ENTER_MS
            && HOLD_CS_ENTER_MS < HOLD_ED_ENTER_MS
            && HOLD_ED_ENTER_MS < HOLD_LOCK_MS
            && HOLD_LOCK_MS     < HOLD_FACTORY_MS,
            "held-button timeline must be strictly increasing");

#define HOLD_L_UNLOCK_MS      3000     /* locked: 3-10s unlocks */
#define HOLD_L_FACTORY_MS    10000

/* Inside MS (section 8.2) */
#define HOLD_MODE_H_MS        2000
#define HOLD_MODE_V_MS        4000
#define HOLD_MODE_FO_MS       6000
#define MS_DOUBLE_MS           500     /* deferral window for a 2nd press */

/* Adjustment modes (section 8.3) */
/* Mode ED's in-mode ladder. Four bands where H/V/FO have three, because
 * the axis change needs one -- which is why 8s+ is not "reset and exit"
 * as it is in the other modes. Reset here changes the ACTIVE AXIS and
 * stays put; the only exits are save at 4-6s and the idle timeout. */
#define ED_REVERSE_MS         2000     /* solid    : reverse direction */
#define ED_SAVE_MS            4000     /* dark     : save both, exit   */
#define ED_RESET_MS           6000     /* solid    : reset active axis */
#define ED_AXIS_MS            8000     /* pulsing  : position <-> size */
#define ED_IDLE_MS           30000     /* 30s, not 20s                 */

#define ADJ_REVERSE_MS        2000
#define ADJ_SAVE_MS           4000
#define ADJ_RESET_MS          6000
#define ADJ_IDLE_MS          20000
#define ADJ_STROBE_MS         1200

#define POL_PERIOD_SHIFT          6   /* re-measure if the line period
                                       * moves by more than 1/64, ~1.6%
                                       * -- wider than measurement
                                       * noise, narrower than any real
                                       * modeline step */
#define POL_STREAK_REQUIRED       8   /* consecutive out-of-tolerance
                                       * periods before believing it.
                                       *
                                       * One sample is not enough, and
                                       * an interlaced source proves it:
                                       * the half line ending each field
                                       * is a genuinely short Hsync
                                       * period, once per field, every
                                       * field. A single-sample trigger
                                       * fires on it 120 times a second
                                       * forever. Any in-tolerance
                                       * period resets the count, so an
                                       * isolated anomaly can never
                                       * accumulate, while a real
                                       * modeline change -- where EVERY
                                       * period differs -- reaches 8 in
                                       * about half a millisecond. */

/* ===================================================================
 * EEPROM
 * ===================================================================
 *
 * 512 bytes here against 128 on the ATtiny824, so the layout was sized
 * for the whole feature set up front rather than grown a byte at a
 * time. All 25 are in use now. Unwritten bytes read 0xFF and every load
 * path range-checks, so growing into the remaining space costs nothing
 * and needs no magic bump -- which matters, because a bump discards the
 * settings that ARE in use.
 *
 * Each setting is written by persist_byte(), which commits only the
 * byte that changed plus the magic, written LAST. An interrupted write
 * can therefore never leave a valid magic vouching for a stale
 * companion.
 *
 * The magic is a LAYOUT VERSION, not a checksum. Bump it only when the
 * meaning of an existing byte changes. It starts fresh for the EB
 * rather than continuing the ATtiny824's sequence, since no EB part has
 * ever held the old layout and there is nothing to be compatible with.
 *
 * The bytes are one array rather than separate EEMEM symbols, because
 * the linker does not guarantee predictable relative placement between
 * separate EEMEM symbols -- confirmed with nm on the ATtiny824 build,
 * where the order came out looking alphabetical rather than as
 * declared. An array's layout is fixed by construction.
 */
#define MODE_MAGIC        0xE5   /* bumped: Mode CS method 3 inserted, 3-6 shifted to 4-7 */

/* Magic bumped whenever the layout or a setting's range changes, so a
 * part carrying an older layout discards it rather than reading stale
 * values as valid. Two changes have earned a bump so far: the H
 * offset's step count, which has moved several times on its way to the
 * 229 it holds now, and inserting Mode CS's method 3 rather than
 * appending it. Factory reset (section 8.1b) restores the same defaults
 * a freshly programmed part boots with. */
#define EE_MAGIC           0
#define EE_MODE            1
#define EE_B1_LOCKED       2
#define EE_PROFILE         3
#define EE_EDID_P1         4     /* per profile, +0 +1 +2 */
#define EE_HOFF_P1         7
#define EE_VOFF_P1        10
#define EE_FOFF_P1        13

/* ALLOCATED AHEAD OF THEIR CODE, AND NOW WRITTEN BY IT.
 *
 * Mode ED needs six bytes and Mode CS needs three. Landing them in two
 * separate increments would have meant two magic bumps and two wipes of
 * the user's settings, so the layout was grown ONCE and the bytes sat
 * unwritten until the code that owns them existed. It does now: Mode
 * ED's save gesture writes EE_EDPOS/EE_EDSIZE, Mode CS's writes
 * EE_CSMETH, and factory_reset() writes all three per profile.
 *
 * WHAT THAT COST, AND WHY IT WAS STILL THE RIGHT ORDER. The reservation
 * bought exactly one bump instead of two, and the bump it could not
 * avoid was spent elsewhere -- inserting Mode CS's method 3 rather than
 * appending it moved the meaning of a stored in-range value, which is
 * precisely what section 14 requires a bump for. MODE_MAGIC is 0xE5.
 *
 * The rule these slots were reserved under still applies to whatever is
 * added next: an erased byte is 0xFF, every loader below range-checks,
 * and 0xFF is out of range for any range -- so growing INTO unwritten
 * bytes needs no bump. Changing the meaning of a written one does. */
#define EE_EDPOS_P1       16     /* Mode ED  horizontal position, per profile */
#define EE_EDSIZE_P1      19     /* Mode ED  horizontal size,     per profile */
#define EE_CSMETH_P1      22     /* Mode CS  combine method,      per profile */
#define EE_BYTES          25

#define PROFILE_COUNT      3
#define PROFILE_DEFAULT    1

/* Offsets are stored as unsigned step indices with the middle as the
 * neutral setting, exactly as on the ATtiny824. */
#define V_OFFSET_STEPS    61
#define V_OFFSET_DEFAULT  30
#define F_OFFSET_STEPS    31
#define F_OFFSET_DEFAULT  15

static uint8_t EEMEM eeprom_data[EE_BYTES];

static void persist_byte(uint8_t idx, uint8_t value)
{
    eeprom_update_byte(&eeprom_data[idx], value);
    eeprom_update_byte(&eeprom_data[EE_MAGIC], MODE_MAGIC);
}

/* ===================================================================
 * State
 * =================================================================== */

static uint8_t  current_mode  = MODE_1;
static uint8_t  current_profile = PROFILE_DEFAULT;
static uint8_t  edid_enabled;
static uint8_t  b1_locked;

static uint8_t  hsync_valid;          /* period inside the mode's window */
static uint8_t  mode_valid;           /* what the gate pin is driving */
static uint16_t last_period_counts;
static uint8_t  last_windows;
static uint16_t last_hsync_tick;

/* ccl_role lives in ccl.c now; read it with ccl_current_role(). */


static uint8_t  pol_flip;             /* 1 -> LUT1 uses XOR, not XNOR */
static uint8_t  h_pin_idle_high = 1;  /* the source's own Hsync sense */

/* Which edge of the incoming pulse restarts TCE0. One function decides
 * it, so the two places that write the edge sense's TRUTH1 cannot
 * disagree. */
static uint8_t  pol_pending = 1;      /* measure once at start-up */

/* A RE-MEASURE THAT DOES NOT MUTE.
 *
 * pol_pending both measures and blanks the output, which is right when
 * the combiner's inputs have just changed identity -- a mode change, a
 * role change -- because what it was emitting is no longer meaningful.
 *
 * But a source flipping its sync polarity changes NOTHING that any
 * existing trigger watches. The period is identical, so the modeline
 * detector stays quiet, and the stale polarity holds until something
 * unrelated happens to force a measurement. That is exactly the
 * "sometimes it works" behaviour: whether the picture is right depends
 * on whether anything else disturbed the unit since the polarity moved.
 *
 * So this runs the same measurement on a timer without blanking.
 * polarity_measure only applies a change when the answer actually
 * differs, so a recheck that finds nothing costs a truth-table
 * comparison and no visible interruption. */
#define POL_RECHECK_MS   250
static uint8_t  pol_recheck;
static uint16_t pol_recheck_at;

/* Separate from pol_pending, and it has to be.
 *
 * The genlock's re-acquisition used to be triggered by pol_pending, on
 * the reasoning that the one event justifying a restart -- a modeline
 * change -- announces itself as a polarity re-measure. That held until
 * something else started asking for a polarity re-measure: switching
 * the CCL role changes the H input's idle level from the pin's to a
 * regenerated one, so it must re-measure, and it set the same flag.
 *
 * The result was a standoff. Engaging the reconstruction set
 * pol_pending; pol_pending restarted the genlock; the restart cleared
 * lock; and the reconstruction is only used while locked, so the role
 * fell straight back and set the flag again. It cycled once per line
 * and lock was never held for the 64 lines it needs, so the offset
 * never took effect at all.
 *
 * It also explains something from before 4b: pol_pending stays set for
 * the whole 20ms polarity measurement, and nothing cleared it in the
 * restart branch, so every one of those measurements was restarting
 * the genlock on every line throughout. */
static uint8_t  genlock_reacquire;
static uint8_t  pol_measuring;
static uint16_t pol_start;
static uint16_t pol_h_high, pol_h_low;
static uint16_t pol_v_high, pol_v_low;

/* Vsync presence, for the section 8.2 sync report.
 *
 * Hsync has the capture timer and its watchdog; Vsync has neither, and
 * the polarity counters are no substitute -- they run for 20ms, which is
 * about one field, so a single missed pulse reads as absence.
 *
 * Watching for ANY edge and allowing 100ms without one covers every
 * field rate down to 10Hz, well below anything this will ever see, and
 * needs no assumption about polarity or duty. */
static volatile uint16_t field_qtick;     /* qtick at the last frame edge */
static volatile uint16_t field_period_q;  /* last interval, quarter-ms */
static uint8_t  field_bad;
static uint8_t  field_rate_ok = 1;  /* absent Vsync passes */

static uint16_t vs_change;
static uint8_t  vsync_present;

/* ===================================================================
 * Interlace detection (section 9)
 * ===================================================================
 *
 * An interlaced source alternates: one field's Vsync sits ON the line
 * grid, the next lands in the MIDDLE of a line. So the test is simply
 * where the Vsync edge falls within a line, and whether that answer
 * toggles frame to frame. A progressive source reports the same phase
 * every frame and so reports no interlace.
 *
 * PHASE COMES FREE from TCB0. In frequency mode the counter restarts on
 * every Hsync edge, so its value at the Vsync edge IS the elapsed time
 * since the last line started -- provided it is read AT the edge, which
 * is why this needs an interrupt rather than the polling that serves
 * presence detection.
 *
 * THE MIDDLE QUARTER, not the middle half. Testing between 3/8 and 5/8
 * of a line keeps the decision boundaries away from 25% and 75%, which
 * is where a real Vsync phase can plausibly sit -- and there a few
 * microseconds of interrupt jitter flips the answer every frame.
 *
 * FOUR ALTERNATIONS TO LATCH, EIGHT AGREEMENTS TO RELEASE. Interlace
 * alternates every frame, so a sustained run says so in four. Latching
 * on a single disagreement means one jittery reading pins a progressive
 * source as interlaced -- and if the jitter recurs more often than the
 * release window it never clears, which is why Mode FO would not lock
 * out on a progressive source.
 */
/* ===================================================================
 * The two regenerated Vsync paths, and why they cannot both be on
 * ===================================================================
 *
 * Mode V substitutes a Vsync it emits on pin 13, quantised to the line
 * grid. Mode FO leaves the source's Vsync where it is and delays ONE
 * FIELD of it by a fraction of a line.
 *
 * v_regen requires a PROGRESSIVE source and fo_active requires an
 * INTERLACED one, so the two are mutually exclusive by construction --
 * not by a rule written somewhere that could be forgotten. That is what
 * section 7 means when it says the two are exclusive on the ATtiny824,
 * and it is why the awkward case (a line-quantised pulse that has to
 * have half a line put back into it before a field offset means
 * anything) does not arise here at all.
 *
 * The code below still asks which source LUT0 is looking at rather than
 * assuming the pin, because the combination is the one thing section 7
 * asks for later and the question will become live. */
static uint8_t v_regen;            /* 1 -> the V source is pin 13 */

static uint8_t fo_active;          /* 1 -> LUT0's field delay is in path */

/* The source's OWN Vsync sense, kept separately from the value the
 * combiner is given -- exactly as h_pin_idle_high is for Hsync. LUT0
 * needs it to know which way round to build its truth table, and that
 * is a property of the source, not of the path Csync currently takes. */
static uint8_t v_pin_idle_high = 1;

/* Set where the SYNC WIDTH could have changed, and nowhere else.
 *
 * The first attempt hung the width burst off pol_pending, which was
 * wrong in a way worth recording because it locks up outright rather
 * than merely misbehaving. Engaging Mode FO changes what the combiner
 * sees, so it sets pol_pending; pol_pending would start a width burst;
 * the burst takes TCB1 away, so Mode FO stands down; standing down
 * changes the routing back, which sets pol_pending again. Twenty
 * milliseconds a cycle, forever, with Csync muted throughout -- because
 * pol_pending is gated into mode_valid.
 *
 * A flag that means "the width may have changed" cannot be the same flag
 * as "the combiner's inputs may have changed sense". They share most of
 * their causes and differ in exactly the one that matters. */
static uint8_t width_pending = 1;   /* measure once at start-up */

/* Set while the polarity window has borrowed TCB1 back to measure the
 * sync width. Mode FO stands down for its duration -- see fo_update(). */
static uint8_t width_burst;

/* Mode FO reloads TCB1's delay once per field, deferred a couple of
 * milliseconds from the Vsync edge that arms it -- see
 * fo_load_service(), which explains why it cannot be done at the edge. */
static uint8_t  fo_load_pending;
static uint16_t fo_load_at;

static uint8_t  src_prev_mid;
static uint8_t  src_alt_count;
static uint8_t  src_prog_count;
static uint8_t  src_interlaced;

static volatile uint16_t vs_phase;      /* TCB0.CNT at the leading edge */
static volatile uint8_t  vs_frame_flag; /* a frame boundary was latched */
static volatile uint8_t  vs_any_edge;   /* any edge, for presence */

/* Runs in the main loop from the flag the ISR sets, because none of it
 * is urgent and the ISR must stay short enough not to disturb DDC. */
static void src_track_interlace(uint16_t phase)
{
    uint16_t p = last_period_counts;
    if (!p) return;

    uint8_t midline = (phase > (uint16_t)((p >> 2) + (p >> 3))
                    && phase < (uint16_t)((p >> 1) + (p >> 3))) ? 1 : 0;

    if (midline != src_prev_mid) {
        if (src_alt_count < 4 && ++src_alt_count >= 4) src_interlaced = 1;
        src_prog_count = 0;
    } else {
        src_alt_count = 0;
        if (src_prog_count < 8 && ++src_prog_count >= 8) src_interlaced = 0;
    }
    src_prev_mid = midline;
}

/* Vsync edge interrupt. Deliberately tiny: capture the phase before the
 * counter moves on, note the level, flag it, leave. Everything that can
 * wait is done in the main loop. */
ISR(PORTC_PORT_vect)
{
    if (PIN_VSYNC_PORT.INTFLAGS & PIN_VSYNC_bm) {
        /* THE 16-BIT READ IS MASKED, and this is not paranoia.
         *
         * A 16-bit peripheral register is read low byte first, which
         * latches the high byte into that PERIPHERAL'S shared TEMP
         * register; the high byte then comes from TEMP. The capture
         * interrupt is level 1 and reads TCB0.CCMP, the same
         * peripheral's TEMP. Landing between these two bytes replaces
         * CNT's high byte with CCMP's -- a phase reading of a thousand
         * counts where it should have been ten, which is the difference
         * between a mid-line field and an on-grid one.
         *
         * Masking cannot lose a capture: the flag stays set and the
         * handler runs the moment it is unmasked, six cycles later. */
        TCB0.INTCTRL   = 0;
        uint16_t phase = TCB0.CNT;
        TCB0.INTCTRL   = TCB_CAPT_bm;

        uint8_t  lvl   = (PIN_VSYNC_PORT.IN & PIN_VSYNC_bm) ? 1 : 0;

        /* THE FRAME BOUNDARY IS LATCHED HERE, not deferred.
         *
         * A flag cannot carry it. This fires on BOTH edges, and a short
         * Vsync pulse gives both between two main loop passes -- so the
         * rising edge is overwritten before it is read, that frame's
         * line count is never latched, and the next frame counts two
         * frames' worth. The rate check then rejects a perfectly good
         * mode for one field, which is Csync dropping out in bursts.
         *
         * Latching in the interrupt cannot lose an edge. The line
         * counter is now advanced by the capture interrupt rather than
         * by the main loop, so it cannot lose a count in the race
         * either. */
        if (lvl) {
            vs_phase         = phase;
            /* Interval since the previous frame edge, in QUARTER-
             * milliseconds. Taken here in the interrupt so it is a real
             * frame boundary, not whenever the main loop next looked.
             *
             * qtick_count rather than tick_count, and the plain read is
             * safe for the same reason it was there: this handler and
             * TCF0 are both level 0, so neither can land inside the
             * other's 16-bit access. See tick.h, which offers no
             * accessor precisely so that rule has to be read. */
            uint16_t t = qtick_count;
            field_period_q = (uint16_t)(t - field_qtick);
            field_qtick    = t;
            /* REQUESTED, not performed. This ISR is level 0 and the
             * capture is level 1, so doing the frame work here would put
             * 16-bit writes where a capture can land between their two
             * stores. See the note at the capture ISR. */
            v_frame_req      = 1;
            v_frame_req_mid  = v_next_is_mid;
            vs_frame_flag    = 1;
        }
        vs_any_edge   = 1;
        PIN_VSYNC_PORT.INTFLAGS = PIN_VSYNC_bm;
    }
}

/* ===================================================================
 * Field rate check (section 9)
 * ===================================================================
 *
 * The band is 45-65Hz nominal, CUT AT 43.5 AND 65.6. The margin is
 * there so a legitimate mode sitting on a boundary is not rejected over
 * a little measurement noise; without it a nominal 65Hz source that
 * measures 65.2 would be refused.
 *
 * ABSENT VSYNC PASSES. Some sources send Hsync alone, and a dongle that
 * blanked them would be broken for a case that works perfectly well --
 * the check exists to reject a field rate a CRT cannot follow, and no
 * field rate at all is not one of those.
 */
/* MEASURED IN QUARTER-MILLISECONDS FROM THE TICK, not by counting lines.
 *
 * Counting lines was the obvious choice -- the capture rides an accurate
 * 20MHz clock where the tick is only good to a fraction of a
 * millisecond -- and it was rejected twice, for different reasons.
 *
 * FIRST, because the count was taken by POLLING TCB0 in the main loop
 * and any pass slower than one line silently lost a capture. Missing 60
 * lines of 262 reads as 78.7Hz, outside the band, and the output
 * blanks. Not hypothetical: the loop is busiest during a DDC read, and
 * Windows polls DDC every second or two -- exactly the rate at which the
 * mode-valid pin was seen dropping out in bursts.
 *
 * SECOND, AND STILL TRUE NOW THAT v_line IS ADVANCED BY THE LEVEL 1
 * CAPTURE AND CANNOT LOSE A COUNT: v_frame_lines is not a field. On an
 * interlaced source only one field in two resets the counter -- see the
 * capture ISR, where that is deliberate, because the on-grid field's
 * Vsync edge races an Hsync -- so v_frame_lines is a whole FRAME. Using
 * it here would read 480i as 30Hz and blank every interlaced source.
 * Correcting it with a factor of two when v_ilace is set would make a
 * blanking decision depend on the interlace detector, which is a phase
 * heuristic with its own acquisition state. A safety check should not
 * import the failure modes of the machinery it is meant to protect.
 *
 * WHY QUARTER-MILLISECONDS AND NOT MILLISECONDS. field_period_q is the
 * difference of two counts, so a source whose true period is not a whole
 * number of them measures as one of the two integers either side, in
 * proportion to the fraction. That is harmless when the quantum is small
 * against the gap between the rates the threshold separates, and fatal
 * when it is not.
 *
 * On the old 1ms grid with the band at 45-75Hz it was harmless: 75Hz is
 * 13.33ms, read 13 or 14, and both passed. Narrowing the top to 65Hz
 * broke that, because 65Hz is 15.38ms and 70Hz is 14.29ms -- 1.1ms
 * apart, both straddling 15. Every frame at 70Hz read 14, 14, 14, 15,
 * and with two bad measurements to fail and one good one to recover
 * that is FORTY blank/unblank transitions per second. 70Hz is not
 * obscure; 640x350 and 720x400, the VGA text modes, are 31.5kHz at
 * 70Hz.
 *
 * TCF0 ALREADY INTERRUPTS AT 4kHz for the software PWM, so the finer
 * count is one uint16_t in an interrupt that was running anyway, and the
 * Vsync ISR reads it under the same level-0 rule that already covered
 * tick_count. Four counts now separate 65Hz from 70Hz where none did.
 *
 * A THRESHOLD ON A QUANTISED MEASUREMENT ALWAYS HAS A WINDOW AT EACH
 * EDGE where the reading dithers across it, and finer quanta move and
 * shrink that window without removing it:
 *
 *              top window            bottom window
 *   1ms        66.67-71.43Hz  4.76Hz    41.67-43.48Hz  1.81Hz
 *   0.25ms     65.57-66.67Hz  1.09Hz    43.01-43.48Hz  0.47Hz
 *
 * FIELD_MIN_Q = 61 rather than 60 places the top window at 65.57-66.67Hz
 * on purpose. 65Hz reads 61 or 62 and always passes; 66.67Hz reads
 * exactly 60 and always blanks; the window between them is empty of any
 * mode a monitor emits. 60 would put the edge at 66.67Hz and let Mac
 * 66.67Hz through, which is above the band that was asked for.
 *
 * WHAT THE WINDOW IS FOR IS SETTLED SEPARATELY, by the hysteresis at
 * field_rate_evaluate() -- the quanta decide WHERE the edge sits, the
 * hysteresis decides that a source sitting on it settles rather than
 * strobing. The two do different jobs and are worth keeping apart when
 * reading either. */
#define FIELD_MIN_Q        61U    /* 15.25ms -- 65.6Hz */
#define FIELD_MAX_Q        92U    /* 23.00ms -- 43.5Hz */

/* Two consecutive bad measurements before blanking. A single outlier --
 * a frame boundary landing either side of a tick, a source glitch --
 * must not blank a picture that is otherwise fine. */
#define FIELD_BAD_LIMIT    2

/* THE RECOVER EDGE SITS THIS MANY QUANTA INSIDE THE FAIL EDGE.
 *
 * One quantum is the whole of it. The measurement of any period reads
 * as one of the two integers either side of it and never further, so a
 * recover edge one quantum in cannot be satisfied by the same dither
 * that trips the fail edge. Two would widen the dead zone for nothing.
 *
 * Setting it to 0 restores the single-threshold rule exactly, which is
 * the A/B this wants on the bench. */
#define FIELD_HYST_Q       1U

_Static_assert(FIELD_MIN_Q + FIELD_HYST_Q <= FIELD_MAX_Q - FIELD_HYST_Q,
               "hysteresis has closed the recover band");


static void field_rate_evaluate(void)
{
    uint16_t q = field_period_q;

    if (!vsync_present || !q) {
        field_rate_ok = 1;
        field_bad     = 0;
        return;
    }

    /* STILL ASYMMETRIC IN TIME: two consecutive bad frames to fail, ONE
     * good frame to recover. Making that symmetric was tried with the
     * frame-length stability test and reverted with it -- it did not
     * help, and slowing recovery costs real time on every legitimate
     * mode change to fix nothing. Nothing below adds a frame of latency
     * to recovery; the first qualifying measurement still takes it.
     *
     * WHAT IS NEW IS HYSTERESIS IN VALUE RATHER THAN IN TIME. The fail
     * edge and the recover edge are different numbers, one quantum
     * apart, so which test a measurement has to satisfy depends on which
     * state the check is already in.
     *
     * WHY: the time asymmetry above is exactly what makes a measurement
     * sitting on the threshold STROBE instead of settling. A source at
     * the edge reads 60, 61, 60, 61; the 60s trip the fail edge and each
     * 61 immediately undoes it. Before the quarter-millisecond change
     * that was FORTY blank/unblank transitions per second at 70Hz. Finer
     * quanta moved the window somewhere no monitor mode lives, but a
     * threshold on a quantised measurement always has one, and moving a
     * fault is not fixing it.
     *
     * With the recover edge one quantum in, that same 60/61 dither can
     * satisfy the fail test and NEVER the recover test, so it blanks and
     * stays blanked. A dither one quantum higher satisfies neither fail
     * nor anything else, so it passes and stays passing. Either way it
     * settles. Swept 40-80Hz at 0.01Hz, worst case over 8000 frames goes
     * from 5308 transitions to one, and that one is the initial settle.
     *
     * WHAT IT COSTS, AND IT IS NOT NOTHING: one quantum at each edge is
     * a DEAD ZONE where the verdict depends on the direction the source
     * arrived from -- 64.52-65.57Hz at the top, 43.48-43.96Hz at the
     * bottom. A rate drifting DOWN into it stays passing; the same rate
     * arriving from outside stays blanked. That is what hysteresis means
     * and it is not a defect, but it is why the edges were placed where
     * they are: 65Hz reads 61 or 62, so it recovers from cold on the
     * first or second frame, and only a source at 65.57Hz exactly sits
     * deep enough in the zone to take seconds about it. Nothing emits
     * 65.57Hz.
     *
     * FIELD_HYST_Q = 0 collapses this back to the single-threshold rule
     * byte for byte, which is the comparison to make on the bench. */
    if (field_rate_ok) {
        if (q >= FIELD_MIN_Q && q <= FIELD_MAX_Q) {
            field_bad = 0;
        } else if (field_bad < FIELD_BAD_LIMIT
                && ++field_bad >= FIELD_BAD_LIMIT) {
            field_rate_ok = 0;
        }
    } else if (q >= (uint16_t)(FIELD_MIN_Q + FIELD_HYST_Q)
            && q <= (uint16_t)(FIELD_MAX_Q - FIELD_HYST_Q)) {
        field_bad     = 0;
        field_rate_ok = 1;
    }
}

static void update_vsync(uint16_t now)
{
    /* ONE path latches the frame, and it is the interrupt. The polled
     * edge detection that used to sit here has gone: with both running,
     * they disagreed about which frames had been counted. */
    if (vs_any_edge) {
        vs_any_edge   = 0;
        vs_change     = now;
        vsync_present = 1;
    }

    if (vs_frame_flag) {
        vs_frame_flag = 0;
        src_track_interlace(vs_phase);   /* deferred: neither is urgent */
        field_rate_evaluate();

        /* Fields alternate, so the one just processed names the next.
         * The Vsync ISR copies this alongside its request, because
         * parity needs the phase weighed against the line period and
         * that is not interrupt work. */
        v_next_is_mid = (uint8_t)(src_prev_mid ? 0 : 1);
        v_ilace_want  = src_interlaced;

        /* Mode FO reloads TCB1 once per field, but not from here -- the
         * one-shot may still be running. Armed now, serviced a couple of
         * milliseconds later. See fo_load_service(). */
        fo_load_pending = 1;
        fo_load_at      = now;
    }

    if (elapsed_since(now, vs_change) > 100) {
        vsync_present    = 0;
        field_period_q   = 0;
        field_rate_ok    = 1;          /* absent Vsync passes */
        src_interlaced   = 0;          /* nothing to be interlaced */
        src_alt_count    = 0;
        src_prog_count   = 0;
    }

}
static uint16_t pol_ref_period;
static uint8_t  pol_streak;


static uint16_t settle_start;
static uint16_t settle_len;
static uint8_t  settling;

/* ===================================================================
 * DDC / EDID client
 * ===================================================================
 *
 * Emulates the 24C02-style DDC EEPROM every VGA/HDMI source expects at
 * I2C address 0x50: a single-byte offset pointer set by a host write,
 * then sequential reads that wrap at 256 (free, since the offset is a
 * uint8_t).
 *
 * The served image is a 256-byte RAM shadow, rebuilt per mode from a
 * base plus a patch list -- see the block below, which is where that
 * scheme and its consequences are described.
 *
 * The E-DDC segment-pointer address 0x30 is also ACKed -- SADDRMASK in
 * ADDREN mode, which makes it a second exact address rather than a mask
 * -- with the segment byte accepted and discarded. Everything served
 * fits in segment 0, but some hosts write the segment pointer
 * unconditionally and sulk if it NACKs. Host writes to 0x50 beyond the
 * offset byte are likewise ACKed and discarded, matching how write-
 * protected DDC EEPROMs in real monitors behave; NACKing mid-write
 * upsets more hosts than silently ignoring does.
 */
#define DDC_EDID_ADDR     0x50
#define DDC_SEGMENT_ADDR  0x30

/* THE SIX EDIDs ARE THE SIX .bin FILES. THAT IS THE WHOLE SCHEME.
 *
 * This replaces a generated edid_data.h holding one base image plus
 * five patch lists. That was 605 bytes of flash against 1536 for six
 * raw images, and it was given up deliberately: the point of the change
 * is that an EDID can be edited in a hex editor or CRU, dropped into
 * the build directory under its existing name, and flashed, with no
 * generator step and no C to regenerate. 931 bytes at 40% utilisation
 * is what that costs.
 *
 * ONE REPRESENTATION, AND IT IS THE ONE THE BUILD EATS. edid_gen.py
 * emits the .bin files and nothing else now. §20.2 used to warn that a
 * second pipeline emitting a different representation would be a fork;
 * there is no longer a second representation for one to fork into.
 *
 * The images are pulled in by the assembler rather than by a header,
 * because .incbin needs no extra tool, no extra file, and -- unlike the
 * avr-objcopy -I binary route -- no architecture number in the
 * Makefile. Verified byte-identical: the 256 bytes at each symbol in
 * the .hex match the source .bin.
 *
 * The FULL 256 bytes are shadowed, not just the block Mode ED will
 * edit. A 128-byte shadow would put an "is this byte below 128?" test
 * on every byte served, inside an interrupt that is deliberately held
 * below the capture. 256 bytes of a 3KB SRAM buys the ISR staying
 * exactly as it was: one indexed load, no branch. */
#define EDID_BYTES  256

/* THE FILE NAMES ARE PART OF THE INTERFACE. They are baked in here, so
 * a replacement image must arrive under the same name or the build
 * fails at assembly time with `file not found: edidN.bin`.
 *
 * EACH FILE MUST BE 128 OR 256 BYTES, and the .if guards below enforce
 * it by name. This is not tidiness. edid_checksums() rewrites bytes 127
 * and 255 on every rebuild, so a truncated or overlong file would be
 * served as a plausible, correctly-checksummed, wrong EDID -- the
 * silent-wrong-artefact failure, arriving through the one door where
 * nothing else in this firmware would notice.
 *
 * A 128-BYTE FILE IS PADDED WITH THE CEA BLOCK THE SHIPPED IMAGES
 * CARRY: rev 3, no extension DTDs, basic audio, and an HDMI VSDB with
 * physical address 0.0.0.0. Byte 255 is emitted with the value that
 * checksums that block, which the firmware then recomputes anyway.
 *
 * WHAT PADDING IMPLIES, and it is a real change to what the dongle
 * advertises: a bare 128-byte EDID that declared no audio and no HDMI
 * vendor block will, after padding, declare both. That is the whole
 * point -- it is what makes an arbitrary drop-in look like the shipped
 * images to a host -- but it is not nothing, and a file that must NOT
 * advertise them has to arrive as 256 bytes carrying its own second
 * block.
 *
 * Byte 126 is fixed up at run time rather than here, because the
 * assembler can append bytes to included data but cannot alter it --
 * see edid_rebuild().
 *
 * WRITTEN OUT SIX TIMES ON PURPOSE. THE MACRO VERSION FAILED ON
 * WINDOWS AND BUILT CLEANLY ON LINUX. This was six lines once, an
 * `.macro EDID_IMAGE sym, file` taking the symbol and the filename and
 * substituting \\file into both the .incbin and the .error string. The
 * Linux assembler expanded it correctly, including the error path,
 * which is how it got shipped. The same binutils version on Windows
 * reported six bad escapes per invocation -- \\U \\B \\A \\L \\T \\c,
 * which is C:\\Users\\...\\AppData\\Local\\Temp\\cc*.s taken apart by the
 * string lexer. So the substitution was yielding the assembler's own
 * input filename rather than the argument, and only on that host.
 *
 * THE MECHANISM IS NOT UNDERSTOOD and no explanation is offered here,
 * because a wrong one would be worse than none: it may be that `file`
 * collides with something in that build, or that macro arguments are
 * bound differently there. What is certain is the shape of the fault
 * -- backslash substitution inside an assembler string, resolved by
 * the host -- so the fix is to have none. After this change the
 * emitted assembly for this block contains no backslash at all, which
 * is a property `make lst` can be grepped for and the reason the
 * remaining macro takes no arguments and contains no strings.
 *
 * THE COST OF THE VERSION THAT LOOKED BETTER was a build that only
 * worked on the machine it was tested on, in the one construct every
 * EDID in the product passes through. Six repeated blocks is the right
 * trade. */
__asm__ (
    ".section .rodata,\"a\",@progbits\n"

    /* Parameterless and string-free, deliberately -- see the note above
     * the images. This is the only macro left and it substitutes
     * nothing. */
    ".macro EDID_CEA_PAD\n"
    "  .byte 0x02, 0x03, 0x0E, 0x40\n"          /* CEA tag, rev 3, d=14, basic audio */
    "  .byte 0x23, 0x09, 0x7F, 0x07\n"          /* audio data block: LPCM, 2ch       */
    "  .byte 0x65, 0x03, 0x0C, 0x00, 0x00, 0x00\n" /* HDMI VSDB, phys addr 0.0.0.0   */
    "  .fill 113, 1, 0x00\n"                    /* d=14 means no extension DTDs      */
    "  .byte 0x87\n"                            /* block checksum; recomputed anyway */
    ".endm\n"

    "edid_img_1: .incbin \"EDID-Mode1.bin\"\n"
    ".if (. - edid_img_1) == 128\n"
    "  EDID_CEA_PAD\n"
    ".endif\n"
    ".if (. - edid_img_1) != 256\n"
    "  .error \"EDID-Mode1.bin must be 128 or 256 bytes\"\n"
    ".endif\n"
    ".global edid_img_1\n"

    "edid_img_2: .incbin \"EDID-Mode2.bin\"\n"
    ".if (. - edid_img_2) == 128\n"
    "  EDID_CEA_PAD\n"
    ".endif\n"
    ".if (. - edid_img_2) != 256\n"
    "  .error \"EDID-Mode2.bin must be 128 or 256 bytes\"\n"
    ".endif\n"
    ".global edid_img_2\n"

    "edid_img_3: .incbin \"EDID-Mode3.bin\"\n"
    ".if (. - edid_img_3) == 128\n"
    "  EDID_CEA_PAD\n"
    ".endif\n"
    ".if (. - edid_img_3) != 256\n"
    "  .error \"EDID-Mode3.bin must be 128 or 256 bytes\"\n"
    ".endif\n"
    ".global edid_img_3\n"

    "edid_img_4: .incbin \"EDID-Mode4.bin\"\n"
    ".if (. - edid_img_4) == 128\n"
    "  EDID_CEA_PAD\n"
    ".endif\n"
    ".if (. - edid_img_4) != 256\n"
    "  .error \"EDID-Mode4.bin must be 128 or 256 bytes\"\n"
    ".endif\n"
    ".global edid_img_4\n"

    "edid_img_5: .incbin \"EDID-Mode5.bin\"\n"
    ".if (. - edid_img_5) == 128\n"
    "  EDID_CEA_PAD\n"
    ".endif\n"
    ".if (. - edid_img_5) != 256\n"
    "  .error \"EDID-Mode5.bin must be 128 or 256 bytes\"\n"
    ".endif\n"
    ".global edid_img_5\n"

    "edid_img_S: .incbin \"EDID-ModeS.bin\"\n"
    ".if (. - edid_img_S) == 128\n"
    "  EDID_CEA_PAD\n"
    ".endif\n"
    ".if (. - edid_img_S) != 256\n"
    "  .error \"EDID-ModeS.bin must be 128 or 256 bytes\"\n"
    ".endif\n"
    ".global edid_img_S\n"

    ".text\n"
);

extern const uint8_t edid_img_1[], edid_img_2[], edid_img_3[],
                     edid_img_4[], edid_img_5[], edid_img_S[];

static uint8_t edid_shadow[EDID_BYTES];
static const uint8_t * volatile edid_image = edid_shadow;
static volatile uint8_t edid_offset;
static volatile uint8_t twi_is_segment;
static volatile uint8_t twi_expect_offset;
static volatile uint8_t twi_read_active;

/* Rebuild the shadow for `mode`. The caller must have the DDC client
 * disabled -- see edid_select().
 *
 * Both checksums are recomputed rather than trusted to the .bin. Mode
 * ED edits DTDs, so they have to be; and byte 126 is rewritten above,
 * which invalidates byte 127 on its own. Having the one code path that
 * produces a served image also produce its checksum is what keeps the
 * two from drifting.
 *
 * THE CONSEQUENCE FOR DROP-IN FILES, and it cuts both ways: a .bin does
 * not need a correct checksum, which makes hand-editing far easier;
 * and a corrupt one is served looking valid, which is why the size
 * guards in the .incbin block are the only check there is. */
/* Defined below, next to the rest of the DDC client. Declared here
 * because edid_select() has to take the client down across a rebuild
 * and the two live at opposite ends of this section. */
static void apply_edid_enable(void);

/* THE IMAGE TABLE IS INDEXED BY MODE NUMBER, 1..MODE_S, WITH SLOT 0
 * DEAD. Getting this wrong is not a compile error and not a crash -- it
 * is every mode quietly serving the NEXT mode's EDID, with Mode S
 * reading off the end of the array. It shipped exactly once, in the
 * first base+patch build, because the generator emitted 0-based tables
 * while edid_images[] had been 1-based and the call site passed `mode`
 * straight through.
 *
 * The offline check at the time reconstructed indices 0..5 and compared
 * them against images 1..S, so it proved the TABLE and never the
 * mapping into it. The asserts check the mapping, which is the part
 * that was actually wrong. §19.10.
 *
 * The hazard did not leave with the patch lists -- the table below is
 * still hand-written and still 1-based, and a name dropped from it
 * would shift every mode above the gap. */
static const uint8_t *const edid_images[MODE_S + 1] = {
    0,             /* unused: modes are 1-based */
    edid_img_1,
    edid_img_2,
    edid_img_3,
    edid_img_4,
    edid_img_5,
    edid_img_S,
};

_Static_assert(sizeof(edid_images) / sizeof(edid_images[0]) == MODE_S + 1,
               "edid_images must be indexed by mode number, 0..MODE_S");
_Static_assert(MODE_1 == 1, "mode numbering is 1-based; the image table assumes it");

/* Where Mode ED's descriptor slots are, recomputed on every rebuild.
 *
 * ed_ext_base is the shadow offset of the FIRST extension-block DTD and
 * ed_nslots the TOTAL slot count including block 0's four. Held rather
 * than recomputed inside ed_transform() because edid_select()'s
 * walk-down calls that up to eighty times and the extension count costs
 * a division; the shadow's extension header cannot move underneath it,
 * since Mode ED writes only inside descriptors. */
static uint8_t ed_ext_base;
static uint8_t ed_nslots;

static void edid_rebuild(uint8_t mode)
{
    /* A bad mode here serves garbage rather than faulting, so it is
     * worth the four bytes to make it serve Mode 1 instead. */
    if (mode < MODE_1 || mode > MODE_S) mode = MODE_1;

    memcpy(edid_shadow, edid_images[mode], EDID_BYTES);

    /* BYTE 126 STATES HOW MANY EXTENSION BLOCKS FOLLOW, AND EXACTLY ONE
     * 128-BYTE BLOCK IS SERVED, so the only honest values are 1 and 0
     * and which one is decided by whether that block is real.
     *
     * Derived from the block's own tag rather than from a build-time
     * flag, because the tag is the truth and the flag would be a second
     * copy of it. It is right in all four cases that can arrive: a
     * native 256-byte file keeps its 1; a 128-byte file padded above
     * needs its 0 raised to 1, which the assembler could not do; a file
     * some tool already zero-padded to 256 keeps its 0, so no phantom
     * block is advertised; and a file claiming 2 extensions it cannot
     * carry is corrected down rather than sending a host to read a
     * block that would wrap back to byte 0. */
    edid_shadow[126] = edid_shadow[128] ? 1 : 0;

    /* WHERE THE DTDs ARE. Block 0 always has four descriptor slots at
     * 54. A CEA-861 block carries its own, and byte 2 of it is the
     * offset within the block at which they start -- 0 meaning none.
     * The list is terminated by a zero pixel clock, which ed_dtd()
     * already treats as "not a timing descriptor", so the slot count
     * only has to bound the walk rather than find the end of it.
     *
     * GATED ON THE TAG BEING 0x02. In a DisplayID, block-map or VTB
     * extension, byte 2 is not a DTD offset and reading it as one would
     * point the walk at arbitrary bytes and then WRITE to them.
     *
     * 128 + d + 18*n <= 255 by construction, so the last slot ends at
     * byte 255 at worst and the checksum is never inside one. */
    ed_ext_base = 0;
    ed_nslots   = 4;
    if (edid_shadow[128] == 0x02) {
        uint8_t d = edid_shadow[130];
        if (d >= 4 && d <= 127) {
            ed_ext_base = (uint8_t)(128 + d);
            ed_nslots   = (uint8_t)(4 + (127 - d) / 18);
        }
    }

    /* The checksums are NOT written here. edid_checksums() owns them,
     * and every caller runs it after whatever it was going to do to the
     * shadow -- so a rebuild and a Mode ED transform are checksummed
     * once, by one function, rather than twice by two. */
}

static void edid_checksums(void)
{
    /* Each 128-byte block ends with the byte that makes its own sum
     * zero modulo 256, so each sum runs over the 127 bytes BEFORE the
     * checksum and never includes it. */
    uint8_t c0 = 0, c1 = 0;
    for (uint8_t i = 0; i < 127; i++) {
        c0 = (uint8_t)(c0 + edid_shadow[i]);
        c1 = (uint8_t)(c1 + edid_shadow[128 + i]);
    }
    edid_shadow[127] = (uint8_t)(0u - c0);
    edid_shadow[255] = (uint8_t)(0u - c1);
}

/* ===================================================================
 * Mode ED -- horizontal position and size, applied to every DTD
 * ===================================================================
 *
 * Both offsets are in steps of ED_STEP_PERMILLE of the ACTIVE WIDTH,
 * so one step is the same fraction of the picture on every DTD
 * regardless of its pixel clock -- which is what makes a single stored
 * number mean the same thing across the three or four timings in one
 * EDID.
 *
 * SIGNS. Positive position moves the picture LEFT, positive size makes
 * it NARROWER. Both are the direction the UI starts in.
 *
 * WHAT EACH ONE ACTUALLY DOES.
 *
 * Position re-splits the blanking that is already there. The back
 * porch is not stored in a DTD -- it is hbl - hso - hsw -- so growing
 * the front porch shortens the back porch by the same amount for free.
 * htotal, the pixel clock and therefore the LINE RATE are all
 * untouched. Active video starts sooner after the sync edge, and the
 * picture moves left.
 *
 * Size scales the pixel clock and htotal TOGETHER, which is what holds
 * fH = clk/htotal constant while the active period ha/clk shrinks. The
 * resolution the host advertises does not change; only the time the
 * picture occupies does. The blanking this frees is split evenly
 * between the two porches so the centre of the picture stays put.
 *
 * WHY THE TWO INTERACT, AND WHY THAT IS NOT A BUG. Moving right means
 * eating the front porch, and every CRT timing has a short one -- a
 * couple of microseconds against eight for the back porch. So rightward
 * travel at constant width is small and there is no arrangement of
 * these fields that makes it otherwise. Narrowing frees blanking into
 * both porches, so it buys rightward travel at an exchange rate of two
 * for one: narrow by 10% and you gain 5% of rightward movement. That
 * coupling is the physics of a CRT line, and Mode ED lets the user feel
 * it rather than hiding it behind a range that silently does nothing.
 */
#define ED_STEP_PERMILLE   5      /* 0.5% of active width per step   */
#define ED_STEPS_MAX      40      /* nominal +/-20%; see ed_transform */
#define ED_PORCH_PERMILLE 15      /* floor on either porch, of line   */

/* Stored BIASED, so that an erased byte cannot be mistaken for a
 * setting. 0xFF decodes to +127, which is outside +/-ED_STEPS_MAX, so
 * every loader below reduces it to neutral -- which is exactly the
 * argument made when these slots were reserved in increment 3, and it
 * is what lets the step count change later without another magic bump. */
#define ED_EE_BIAS       128

/* Mode ED's two live offsets. Loaded per profile by ed_load(), written
 * by the Mode ED release ladder, and rendered into the EDID shadow by
 * ed_transform(). */
static int8_t ed_pos, ed_size;

/* Two axes share one button, so each keeps its own direction: switching
 * axis and switching back must not silently reverse the one you left.
 * Both start at +1 on entry -- position LEFT, size NARROWER.
 *
 * Declared here rather than with the rest of the UI state because the
 * rate blink needs ed_val() and sits well above it. */
static uint8_t  ed_axis;            /* 0 = position, 1 = size */
static int8_t   ed_dir[2] = { 1, 1 };
static int8_t   ed_entry_pos, ed_entry_size;

static int8_t *ed_val(void) { return ed_axis ? &ed_size : &ed_pos; }

/* One DTD's worth of the transform.
 *
 * All of it is one division per output because the scale factor
 * 1000/D appears in every term: with D = 1000 - 5*size, a quantity
 * that should be multiplied by the new-clock ratio is instead
 * multiplied by 1000 and divided by D once, at the end. The
 * intermediates run to a few hundred thousand, so they are 32-bit --
 * this runs on a button press, not in a loop.
 *
 * Returns 0 if either porch would fall below ED_PORCH_PERMILLE of the
 * line, or if any field would overflow the bit widths a DTD gives it.
 * That refusal IS the end of the range: it depends on the DTD and on
 * the size setting, not on a constant, which is why the range opens up
 * as the picture narrows. */
static uint8_t ed_dtd(uint8_t *d, int8_t pos, int8_t size, uint8_t commit)
{
    uint16_t clk10 = (uint16_t)((uint16_t)d[1] << 8 | d[0]);
    if (clk10 == 0) return 1;                     /* not a timing descriptor */

    uint16_t ha  = (uint16_t)(((uint16_t)(d[4] >> 4) << 8) | d[2]);
    uint16_t hbl = (uint16_t)(((uint16_t)(d[4] & 0x0F) << 8) | d[3]);
    uint16_t hso = (uint16_t)(((uint16_t)(d[11] >> 6) << 8) | d[8]);
    uint16_t hsw = (uint16_t)(((uint16_t)((d[11] >> 4) & 3) << 8) | d[9]);

    uint16_t htot = (uint16_t)(ha + hbl);
    int32_t  hbp  = (int32_t)hbl - hso - hsw;
    int32_t  D    = 1000L - 5L * size;

    int32_t nclk = ((int32_t)clk10 * 1000L + D / 2) / D;
    int32_t nhtot = ((int32_t)htot * 1000L + D / 2) / D;
    int32_t nhsw  = ((int32_t)hsw  * 1000L + D / 2) / D;

    /* Back porch keeps its own time, gains half of whatever the size
     * change freed, and then loses the position offset. */
    int32_t num = hbp * 1000L
                + (int32_t)ha * (1000L - D) / 2
                - (int32_t)pos * ED_STEP_PERMILLE * ha;
    int32_t nhbp = (num >= 0) ? (num + D / 2) / D : (num - D / 2) / D;

    int32_t nhbl = nhtot - ha;
    int32_t nhso = nhbl - nhsw - nhbp;

    /* CEILING, NOT NEAREST. Rounding this to nearest lets the floor
     * itself round DOWN: on 496x384 1.5% of the line is 9.3 pixels,
     * which rounds to 9, and a 9-pixel porch is 3% under the limit the
     * floor exists to enforce. A sweep of both axes across all six
     * modes found it. Rounding up costs at most one pixel of travel
     * and makes the guarantee exact. */
    int32_t floor_px = (nhtot * ED_PORCH_PERMILLE + 999L) / 1000L;
    if (nhso < floor_px || nhbp < floor_px) return 0;
    if (nclk < 1 || nclk > 65535L)          return 0;
    if (nhbl < 0 || nhbl > 4095L)           return 0;
    if (nhso > 1023L || nhsw > 1023L)       return 0;

    if (commit) {
        d[0]  = (uint8_t)(nclk & 0xFF);
        d[1]  = (uint8_t)(nclk >> 8);
        d[3]  = (uint8_t)(nhbl & 0xFF);
        d[4]  = (uint8_t)(((ha >> 8) << 4) | ((uint8_t)(nhbl >> 8) & 0x0F));
        d[8]  = (uint8_t)(nhso & 0xFF);
        d[9]  = (uint8_t)(nhsw & 0xFF);
        /* Byte 11 also carries the vertical sync high bits. Rebuild the
         * horizontal half and leave the low nibble exactly as it was --
         * this is the double-write trap from section 19.1 wearing a
         * different hat. */
        d[11] = (uint8_t)((uint8_t)(((nhso >> 8) & 3) << 6)
                        | (uint8_t)(((nhsw >> 8) & 3) << 4)
                        | (uint8_t)(d[11] & 0x0F));
    }
    return 1;
}

/* Slot i of ed_nslots. The first four are block 0's descriptors at 54;
 * the rest are the extension block's, located by edid_rebuild(). The
 * two runs are not contiguous, which is the only reason this is a
 * function rather than an index. */
static uint8_t *ed_slot(uint8_t i)
{
    return (i < 4) ? &edid_shadow[54 + 18 * i]
                   : &edid_shadow[ed_ext_base + 18 * (i - 4)];
}

/* Every timing descriptor in the shadow must accept the offsets, or
 * none of them does. A partial application would move some of an
 * EDID's timings and not others, which is exactly the inconsistency
 * one stored percentage is supposed to prevent -- so this tests every
 * slot before writing any.
 *
 * THE EXTENSION BLOCK IS WALKED TOO, AND THAT IS WHY. This used to test
 * block 0's four slots and stop. Every shipped image has d = 14 with a
 * zero at 142, so there were no extension DTDs to miss and the
 * limitation never showed -- but an EDID pulled off a real monitor or
 * out of CRU very often has them, and with the .bin files as the build
 * input that is now an ordinary thing for a user to drop in. Moving
 * block 0's timings and leaving the extension's alone is the partial
 * application this function exists to refuse, arriving through the one
 * door the four-slot loop did not cover.
 *
 * WHAT IT COSTS ON THE SHIPPED IMAGES: d = 14 gives six extension
 * slots, all of them zero-clock, so ed_dtd() returns on its first test
 * and the added work is six loads. Real extension DTDs cost what a
 * block 0 DTD costs, and edid_select()'s walk-down multiplies that by
 * up to eighty -- see the note there. */
static uint8_t ed_transform(int8_t pos, int8_t size, uint8_t commit)
{
    for (uint8_t i = 0; i < ed_nslots; i++)
        if (!ed_dtd(ed_slot(i), pos, size, 0)) return 0;
    if (commit)
        for (uint8_t i = 0; i < ed_nslots; i++)
            (void)ed_dtd(ed_slot(i), pos, size, 1);
    return 1;
}

/* SWAPPING A POINTER USED TO BE ATOMIC; REBUILDING IN PLACE IS NOT.
 *
 * With six images in flash this was one store under cli(). Now the
 * bytes a DDC read is walking are the bytes being overwritten, and a
 * host that reads across a mode change would get half of one image and
 * half of another -- with a checksum that says it is fine, because the
 * checksum is written last.
 *
 * Doing the rebuild under cli() is not an option: 256 bytes of copy
 * plus a patch is tens of microseconds with interrupts off, and
 * now_ticks() was moved off cli() for a few HUNDRED NANOSECONDS because
 * that was the largest single contributor to genlock jitter.
 *
 * So the DDC client is taken down across the rebuild instead. A host
 * mid-transaction sees the address stop being acknowledged and retries,
 * which is ordinary I2C and is what it already does when EDID is
 * switched off. apply_edid_enable() stays the only writer of
 * TWI0.SCTRLA; this borrows it by driving it from the live flag, which
 * it already reads. */
static void edid_select(uint8_t mode)
{
    uint8_t was = edid_enabled;

    if (was) { edid_enabled = 0; apply_edid_enable(); }

    /* WALK THE OFFSETS DOWN UNTIL THEY FIT, rather than abandoning them.
     *
     * Every mode has its own limits -- Mode 1 takes 28 steps of left,
     * Mode 5 only 16, because Mode 5 carries the 496x384 DTD and that
     * has the least porch to give. Dropping straight to zero when a
     * saved setting was too large for the mode being entered meant the
     * picture snapped from fully shifted back to centre on a mode
     * change, which reads as the setting having been lost.
     *
     * Position is given up FIRST and size second, because narrowing is
     * what buys position headroom in the first place -- surrendering it
     * early would make the position fit harder, not easier.
     *
     * The rebuild is done ONCE and only the test is repeated; a test is
     * a few divisions per DTD, where a rebuild is a 256-byte copy.
     *
     * THE COST OF THIS LOOP IS NOW SET BY THE .bin FILES. It runs at
     * most |p| + |z| + 1 times -- eighty-one at the extremes -- and
     * each pass is five 32-bit divisions per NON-ZERO-CLOCK descriptor.
     * On the shipped images that is three descriptors at worst. On a
     * dropped-in EDID carrying extension DTDs it could be ten, and the
     * time scales straight with it.
     *
     * AT BOOT THIS IS FREE. main() calls edid_select() before sei() and
     * before settle_begin_len() samples the start of the boot curtain,
     * so the whole walk-down happens with the output not yet running
     * and the 250ms curtain begins afterwards. It delays boot; it
     * cannot eat the curtain.
     *
     * ON A MODE CHANGE IT IS NOT. change_mode() calls edid_select()
     * BEFORE settle_begin(), and passes a `now` sampled before both --
     * so the walk-down runs unmuted and then shortens the mute it is
     * followed by. That ordering predates this change and was harmless
     * at four descriptors. It is the first place a slow drop-in EDID
     * will show, and the fix if it does is to raise the settle first.
     * §19.27 is the same shape. */
    edid_rebuild(mode);
    int8_t p = ed_pos, z = ed_size;
    while ((p || z) && !ed_transform(p, z, 0)) {
        if      (p > 0) p--;
        else if (p < 0) p++;
        else if (z > 0) z--;
        else            z++;
    }
    if (p || z) (void)ed_transform(p, z, 1);
    edid_checksums();

    if (was) { edid_enabled = 1; apply_edid_enable(); }
}

/* Try an exact pair on the CURRENT mode. Returns 1 if it was applied as
 * given; returns 0 and leaves the committed offsets rendered instead.
 *
 * THE RETURN VALUE IS THE END OF THE RANGE. Not a constant, not a step
 * count -- the answer depends on which DTDs the current EDID holds and
 * on how much blanking the size setting has freed. That is the whole of
 * option C: a press either moves the picture or says it cannot, and
 * nothing in between does nothing. */
static uint8_t ed_commit(int8_t p, int8_t z)
{
    uint8_t was = edid_enabled;

    if (was) { edid_enabled = 0; apply_edid_enable(); }
    edid_rebuild(current_mode);
    uint8_t ok = (!p && !z) ? 1 : ed_transform(p, z, 0);
    if (ok) {
        if (p || z) (void)ed_transform(p, z, 1);
    } else if (ed_pos || ed_size) {
        /* Keep what we had. ed_transform() tests before it writes, so
         * if even the committed pair no longer fits nothing is written
         * and the shadow keeps the pristine image -- a valid EDID
         * either way, never a half-applied one. */
        (void)ed_transform(ed_pos, ed_size, 1);
    }
    edid_checksums();
    if (was) { edid_enabled = 1; apply_edid_enable(); }
    return ok;
}

static void twi_client_init(void)
{
    /* Open-drain is done by the TWI hardware; the host end of the DDC
     * bus provides the pull-ups and its own supply, so there is nothing
     * to configure on the port.
     *
     * SDA on PC2, SCL on PC3 -- pins 8 and 9. PA0 is the oscillator
     * input, so the ALT3 position DDC used before the rotation is
     * gone, and PC2/PC3 is the only other client position TWI0 has.
     * That is what takes PC3 away from LUT1 and forces the whole
     * rotation.
     *
     * ALT2, NOT DEFAULT. Dual mode is for a host and a client running
     * at once on separate pins; with it off -- and this design is
     * client-only -- the client uses the HOST pins. DEFAULT's host pins
     * are documented as "SDA: -, SCL: -", so the client had no pins at
     * all and served nothing. ALT2's host pins are PC2 and PC3.
     *
     * The pre-rotation wiring worked because ALT3's host pins are
     * PA0/PA1, so the same client-uses-host-pins rule happened to land
     * on the right ones. Reading the multiplexing table's SDA(C) column
     * and assuming the client follows it is what got this wrong. */
    PORTMUX.TWIROUTEA = PORTMUX_TWI0_ALT2_gc;

    TWI0.SADDR     = (DDC_EDID_ADDR << 1);
    TWI0.SADDRMASK = (DDC_SEGMENT_ADDR << 1) | TWI_ADDREN_bm;
                     /* ADDREN=1: a second exact address to match, not a
                      * mask -- so 0x50 and 0x30 are ACKed and nothing
                      * else is */
}

/* Enabling and disabling the DDC client is a single register. With
 * ENABLE clear the address match never fires, so nothing is ACKed and
 * the TWI hardware releases the pins back to high-impedance inputs
 * under the host's pull-ups -- electrically indistinguishable from an
 * absent EEPROM, which is the whole point of the feature. Some hosts
 * (Windows with NVIDIA drivers, notably) get awkward about the 15kHz
 * detailed timings, and the useful escape hatch is to present no DDC
 * device at all, exactly as a plain passive cable would.
 *
 * Guarded because it can run from the main loop while the ISR is live. */
static void apply_edid_enable(void)
{
    uint8_t s = SREG;
    cli();
    if (edid_enabled) {
        TWI0.SCTRLA = TWI_DIEN_bm | TWI_APIEN_bm | TWI_PIEN_bm
                    | TWI_ENABLE_bm;
    } else {
        TWI0.SCTRLA  = 0;
        /* Clear anything the peripheral had pending, so re-enabling
         * later starts from a known state rather than immediately
         * servicing a flag left over from a half-finished
         * transaction. */
        TWI0.SSTATUS = TWI_DIF_bm | TWI_APIF_bm | TWI_COLL_bm
                     | TWI_BUSERR_bm;
    }
    twi_is_segment    = 0;
    twi_expect_offset = 0;
    twi_read_active   = 0;
    SREG = s;
}

/* Ordinary priority, for the same reason as the tick above: the level-1
 * capture preempts it. Without that, a DDC read -- which fires this per
 * byte -- would put a burst of phase errors into the genlock, and the
 * picture would twitch whenever anything interrogated the EDID. */
ISR(TWI0_TWIS_vect)
{
    uint8_t s = TWI0.SSTATUS;

    /* Bus error / collision: abandon the transaction cleanly. */
    if (s & (TWI_BUSERR_bm | TWI_COLL_bm)) {
        twi_read_active = 0;
        TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
        return;
    }

    if (s & TWI_APIF_bm) {
        if (s & TWI_AP_bm) {
            /* Address match: SDATA holds the address byte, telling us
             * which of the two matched. */
            twi_is_segment    = ((TWI0.SDATA >> 1) == DDC_SEGMENT_ADDR);
            twi_expect_offset = 1;
            twi_read_active   = 0;
            TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;          /* ACK it */
        } else {
            /* Stop condition. */
            TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
        }
        return;
    }

    if (s & TWI_DIF_bm) {
        if (s & TWI_DIR_bm) {
            /* Host is reading. After the first byte, RXACK reports
             * whether the host ACKed the previous one -- a NACK is its
             * "that's enough". */
            if (twi_read_active && (s & TWI_RXACK_bm)) {
                TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
            } else {
                TWI0.SDATA = edid_image[edid_offset++];
                twi_read_active = 1;
                TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
            }
        } else {
            /* Host is writing. To 0x50: the first byte sets the offset
             * pointer, anything further is an attempted EDID write.
             * To 0x30: the segment byte. Both ACKed, both discarded. */
            uint8_t d = TWI0.SDATA;
            if (!twi_is_segment && twi_expect_offset) {
                edid_offset = d;
                twi_expect_offset = 0;
            }
            TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
        }
    }
}

/* ===================================================================
 * Output gating
 * ===================================================================
 *
 * The gate pin is driven, read straight back in through EVSYS, and
 * combined in LUT1. Csync is held at its idle level whenever it is low.
 *
 * Muting is combinational, so it takes effect within a LUT propagation
 * delay of the write and cannot itself produce a partial sync pulse the
 * way disabling the LUT would.
 */
/* ===================================================================
 * Mode CS -- combine method (section 17.1)
 * ===================================================================
 *
 * Seven methods, each pairing what pin 12 does with what pin 13 does:
 *
 *   1  Csync            pin 13 blocked      the default, and every
 *                                           build before this one
 *   2  Csync            V passed
 *   3  Csync, flat      V passed            no serrations through V
 *   4  H passed         V passed            separate sync out
 *   5  H flipped        V passed
 *   6  H passed         V flipped
 *   7  H flipped        V flipped
 *
 * METHOD 3 WAS INSERTED, NOT APPENDED, and that cost a MODE_MAGIC bump.
 * It belongs beside the other two composite methods rather than after
 * the separate-sync ones, because the list is read by a user counting
 * LED swells and "the Csync ones are 1 to 3" is worth more than a
 * stable numbering. But a part holding a saved method 3 would read the
 * new meaning under the old number -- separate sync becoming flat Csync
 * -- which is exactly the in-range misread section 14 requires the bump
 * for. Saved settings reset. That is the price and it was paid
 * deliberately.
 *
 * PASS AND FLIP ARE RELATIVE TO THE SOURCE, NOT ABSOLUTE POLARITIES.
 * Flip means "invert whatever arrived"; it does not force a sense. A
 * source sending positive-going H comes out negative-going under method
 * 4 and a source sending negative-going H comes out positive-going, and
 * neither is more correct than the other. Methods 3 to 6 therefore
 * cover all four separate-sync polarity combinations for any one
 * source without the firmware needing to know which the display wants.
 *
 * BOTH HALVES ARE BUILT. An earlier increment shipped the H half alone
 * with pin 13 held low, which made 1 and 2 externally identical, as 3
 * and 5 were, and 4 and 6. That is no longer true of anything: the V
 * half is arbitrated against Mode V's regeneration and its loopback in
 * v_pin_apply(), which is pin 13's single owner. All seven methods are
 * externally distinct. */
#define CS_METHOD_MIN      1
#define CS_METHOD_MAX      7
#define CS_METHOD_DEFAULT  1

_Static_assert(EE_CSMETH_P1 + 3 <= EE_BYTES,
               "Mode CS profile slots run past the EEPROM image");

static uint8_t cs_method = CS_METHOD_DEFAULT;

/* The method table's H column, and the ONLY place it is read. ccl.c is
 * handed this rather than the method number, so the pairing of an H
 * behaviour with a V behaviour stays here where the pin arbitration
 * will also live. */
static ccl_hmode_t cs_hmode(void)
{
    switch (cs_method) {
    case 3:         return CCL_H_FLAT;
    case 4: case 6: return CCL_H_PASS;
    case 5: case 7: return CCL_H_FLIP;
    default:        return CCL_H_COMBINE;   /* 1, 2, and anything odd */
    }
}

/* The method table's V column. Only method 1 blocks. */
typedef enum { CS_V_BLOCK = 0, CS_V_PASS = 1, CS_V_FLIP = 2 } cs_vmode_t;

static cs_vmode_t cs_vmode(void)
{
    switch (cs_method) {
    case 6: case 7: return CS_V_FLIP;
    case 2: case 3: case 4: case 5: return CS_V_PASS;
    default:        return CS_V_BLOCK;      /* 1, and anything odd */
    }
}

static void v_pin_apply(void);   /* pin 13's single owner; see below */

static void set_mode_valid(uint8_t on)
{
    on = on ? 1 : 0;

    /* Called every pass of the main loop, so it writes only on a
     * change. Both pins start low in port_init() and mode_valid starts
     * 0, so the two agree before the first call and the guard cannot
     * skip a write that was needed. */
    if (on == mode_valid) return;
    mode_valid = on;

    /* Pin 10 does not carry this -- the mute is a truth table, and
     * ccl.c owns the register that holds it. */
    ccl_set_gate(on, pol_flip, h_pin_idle_high, cs_hmode());

    if (on) PIN_MODE_LED_PORT.OUTSET = PIN_MODE_LED_bm;      /* Mode Valid LED follows */
    else    PIN_MODE_LED_PORT.OUTCLR = PIN_MODE_LED_bm;

    /* PIN 13 FOLLOWS PIN 12, IN EVERY METHOD.
     *
     * The gate above mutes Csync. Without this call the V output on pin
     * 13 carried on regardless, so a source outside the windows -- or
     * one with a field rate a CRT cannot follow -- left the display
     * with a Vsync and no Csync. That is a worse thing to emit than
     * nothing: the set has something to lock the vertical to and
     * nothing to lock the line to, which is how a monitor ends up
     * hunting rather than showing a clean no-signal.
     *
     * mode_valid is assigned BEFORE this call, and v_pin_apply() reads
     * it, so the two pins cannot disagree. The ordering also matters on
     * the way down: v_line_tick() stands down on the same flag, so by
     * the time the block is written the ISR has already stopped
     * driving the pin and cannot undo it. */
    v_pin_apply();
}

/* A SETTLE NEVER SHORTENS ONE ALREADY RUNNING.
 *
 * settle_begin() used to just reset settle_start, so a short settle
 * arriving during a long one cut it down to its own length. That
 * matters now that the two differ: a modeline change asks for 220ms,
 * and the polarity re-measure it triggers would have finished it after
 * 40. The deadline is kept, not the start. */
static void settle_begin_len(uint16_t now, uint16_t len)
{
    if (settling) {
        uint16_t gone   = elapsed_since(now, settle_start);
        uint16_t remain = (gone >= settle_len) ? 0
                        : (uint16_t)(settle_len - gone);
        if (len <= remain) return;      /* already covered */
    }
    settle_start = now;
    settle_len   = len;
    settling     = 1;
    set_mode_valid(0);
}

static void settle_begin(uint16_t now)
{
    settle_begin_len(now, SETTLE_TICKS);
}

/* THE ONE PLACE cs_method CHANGES, and it does not write TRUTH2.
 *
 * There are exactly three writers of that register -- ccl_configure(),
 * ccl_set_polarity() and ccl_set_gate() -- and the file that owns them
 * carries a scar for each time a fourth appeared. So this does not add
 * one. It raises a settle, and the settle does the rest: settle_begin()
 * calls set_mode_valid(0), which mutes through ccl_set_gate(); when the
 * window expires the main loop calls set_mode_valid(1), which unmutes
 * through the same function -- and by then cs_hmode() answers
 * differently, so the new table is installed by the existing owner on
 * the way back up.
 *
 * The cost is a settle per press, roughly 40ms of muted Csync. That is
 * not a side effect to be minimised: the sync structure on pin 12 is
 * genuinely changing shape, the display is going to resync regardless,
 * and doing it inside a mute is how every other structural change in
 * this firmware is made. Section 6 judged the same trade acceptable for
 * a polarity flip, which is a strictly smaller change than this.
 *
 * If mode_valid is already down -- no source, or a rate outside the
 * window -- nothing is written now and the new table is installed
 * whenever the source comes back. That is correct rather than merely
 * tolerable: there is no output to change. */
static void v_pin_apply(void);      /* pin 13's single owner, below */

static void cs_set(uint8_t m, uint16_t now)
{
    if (m < CS_METHOD_MIN || m > CS_METHOD_MAX) m = CS_METHOD_DEFAULT;
    if (m == cs_method) return;
    cs_method = m;

    /* BOTH HALVES, IN THE SAME BREATH. The settle rebuilds TRUTH2 on
     * its way back up, which is the H half; the V half is a pin and a
     * routing decision that no settle touches. Leaving v_pin_apply()
     * to some later caller is the shape of the three bugs in section
     * 19.2 -- a stored setting that reaches half the hardware. */
    v_pin_apply();
    settle_begin(now);
}

/* ===================================================================
 * Sync polarity
 * ===================================================================
 *
 * Read from the IDLE LEVEL, which needs no calibration: a sync pulse is
 * a percent or two of its period, so whichever level dominates the
 * sample IS the idle level.
 *
 * Measured on a MODELINE CHANGE rather than continuously. A sync
 * dropout arms a re-measure on the returning captures, and a line
 * period stepping by more than ~1.6% for eight consecutive lines
 * catches the rarer change that does not drop sync. Deciding only at
 * those moments keeps a transient from flipping the combiner, and means
 * the truth-table rewrite -- which briefly interrupts Csync -- lands
 * while the display is already resyncing.
 *
 * With no Hsync arriving the measurement is abandoned rather than
 * guessed, so an unconnected input cannot flip anything.
 */

/* ===================================================================
 * TCB1 -- one timer, three jobs, one owner
 * ===================================================================
 *
 * TCB1 is the only spare timer on this part and three separate features
 * want it. Each used to configure it in place, from its own corner of
 * the file, which works exactly until two of them are live at once.
 * So the timer has ONE setter and the choice is made in ONE function,
 * tcb1_reevaluate(), from the state that decides it.
 *
 *   RESHAPE  Mode S's divide-by-2 emits a 50% square wave, which no sync
 *            separator will lock to. Each rising edge of it fires a
 *            one-shot and THAT is what reaches the combiner.
 *   WIDTH    Pulse-width capture on Hsync, so the H reconstruction can
 *            emit a pulse the same width as the source's (section 6.4).
 *   FO       Mode FO's per-field delay. A one-shot fired by the Vsync
 *            leading edge, whose output holds LUT0's copy of Vsync at
 *            idle for its duration -- so the emitted leading edge lands
 *            CCMP counts late, and the field moves by that much.
 *
 * THE THREE CANNOT COLLIDE, and it is worth saying why rather than
 * trusting the arbitration:
 *
 *   RESHAPE vs FO   Mode S divides only a FAST source; Mode FO acts only
 *                   on an INTERLACED one, which is always slow. They
 *                   cannot both be wanted.
 *   WIDTH vs FO     These genuinely compete, and WIDTH wins when it
 *                   matters. The width is a property of the modeline,
 *                   not of the line, and last_width_counts is consumed
 *                   only by hoffset_apply() -- at acquisition and on an
 *                   offset change. So it does not need measuring
 *                   continuously; it needs measuring after anything that
 *                   could have changed it. That is exactly the polarity
 *                   window, which already runs on every dropout and
 *                   every modeline step and is already muted. Mode FO
 *                   stands down for its 20ms and takes the timer back
 *                   after.
 *
 *                   Gated on pol_pending, NOT on the measurement
 *                   generally: POL_RECHECK_MS re-runs the polarity vote
 *                   four times a second, and hanging the width burst off
 *                   that would take the timer away from Mode FO four
 *                   times a second for nothing.
 *
 * No CCMPEN in any job. That bit routes the waveform to a PIN; the CCL
 * taps the output internally without it, which is what the reshaper has
 * been relying on since increment 3.
 */
typedef enum {
    TCB1_JOB_NONE = 0,
    TCB1_JOB_RESHAPE,
    TCB1_JOB_WIDTH,
    TCB1_JOB_FO
} tcb1_job_t;

static tcb1_job_t tcb1_job = TCB1_JOB_NONE;

/* Which edge of the signal feeding LUT0 is the Vsync pulse's LEADING
 * one. TCB1 must fire on that edge and no other: firing on the trailing
 * edge would delay the END of the pulse, which moves nothing, and firing
 * on both would arm the one-shot twice a field. */
static uint8_t fo_trigger_falling(void)
{
    /* v_regen cannot currently be set while Mode FO is active -- see the
     * note at the declarations -- but LUT0 is fed from a channel, not a
     * pin, so ask what that channel carries rather than assuming. The
     * regenerated pulse on pin 13 idles LOW whatever the source sends. */
    return (uint8_t)(v_regen ? 0 : v_pin_idle_high);
}

static void tcb1_set_job(tcb1_job_t job)
{
    if (job == tcb1_job) return;
    tcb1_job = job;

    TCB1.CTRLA  = 0;                      /* stopped while reconfigured */
    TCB1.EVCTRL = 0;

    switch (job) {
    case TCB1_JOB_RESHAPE:
        /* Single-Shot: every trigger drives the output high for CCMP
         * counts, then low. The output is therefore IDLE LOW with
         * positive pulses whatever the input Hsync polarity was, which
         * is what makes Mode S immune to Hsync polarity on a fast
         * source. It says nothing about Vsync, which still arrives on
         * its pin unchanged.
         *
         * ASYNC so the trigger edge drives the output immediately rather
         * than two peripheral clocks later. 100ns is nothing against a
         * 63us line, but the asynchronous path also avoids the awkward
         * interaction the data sheet describes between the enable and
         * the first event.
         *
         * EDGE stays CLEAR: positive edges only. Triggering on both
         * edges of a 50% square wave would emit two pulses per output
         * line and put the input rate straight back on the output. */
        ev_tcb1_capture_source(EV_TCB1_DIVIDED_HSYNC);
        TCB1.CCMP   = RESHAPE_PULSE_COUNTS;
        TCB1.CTRLB  = TCB_CNTMODE_SINGLE_gc | TCB_ASYNC_bm;
        TCB1.EVCTRL = TCB_CAPTEI_bm;
        /* The data sheet warns that the counter starts as soon as the
         * peripheral is enabled, without any event, and that writing TOP
         * to CNT prevents it. Without this the very first thing the
         * one-shot does is emit an unasked-for pulse. */
        TCB1.CNT    = RESHAPE_PULSE_COUNTS;
        TCB1.CTRLA  = TCB_CLKSEL_DIV1_gc | TCB_ENABLE_bm;
        break;

    case TCB1_JOB_WIDTH:
        /* PW mode measures the HIGH time. Rather than work out which
         * half is the sync pulse from the polarity, the main loop takes
         * the shorter of the high time and the remainder: sync is a few
         * percent of a line in either polarity, so the shorter one is
         * always the pulse. */
        ev_tcb1_capture_source(EV_TCB1_HSYNC);
        TCB1.CTRLB  = TCB_CNTMODE_PW_gc;
        TCB1.EVCTRL = TCB_CAPTEI_bm | TCB_FILTER_bm;
        TCB1.CTRLA  = TCB_CLKSEL_DIV1_gc | TCB_ENABLE_bm;
        break;

    case TCB1_JOB_FO:
        /* Fired by the Vsync leading edge on CHANNEL5. CCMP is rewritten
         * once per field with the delay that field is to receive, a
         * safe distance from any running pulse -- see fo_load_service().
         *
         * Armed at CCMP = 1, the shortest delay that is not zero, so the
         * first field after engagement gets 50ns rather than whatever
         * was left in the register. */
        ev_tcb1_capture_source(EV_TCB1_VSYNC_RAW);
        TCB1.CCMP   = 1;
        TCB1.CTRLB  = TCB_CNTMODE_SINGLE_gc | TCB_ASYNC_bm;
        TCB1.EVCTRL = (uint8_t)(TCB_CAPTEI_bm
                     | (fo_trigger_falling() ? TCB_EDGE_bm : 0));
        TCB1.CNT    = 1;
        TCB1.CTRLA  = TCB_CLKSEL_DIV1_gc | TCB_ENABLE_bm;
        break;

    default:
        break;
    }
}

static void tcb1_reevaluate(void)
{
    tcb1_job_t want;

    if (ccl_current_role() == CCL_ROLE_DIVIDE)  want = TCB1_JOB_RESHAPE;
    else if (fo_active)               want = TCB1_JOB_FO;
    else                              want = TCB1_JOB_WIDTH;

    tcb1_set_job(want);
}

static void polarity_measure(uint16_t now)
{
    if (!pol_measuring) {
        pol_measuring = 1;
        pol_start     = now;
        pol_h_high = pol_h_low = 0;
        pol_v_high = pol_v_low = 0;

        /* BORROW TCB1 BACK FOR THE WIDTH, but only when something that
         * can change the width has happened -- a sync dropout or a
         * modeline step. Not on pol_pending, which Mode FO's own
         * routing sets and which would therefore trigger a burst that
         * makes Mode FO stand down, which sets pol_pending again. Not on
         * pol_recheck either, which fires every POL_RECHECK_MS and would
         * take the timer away four times a second to re-measure
         * something that has not moved.
         *
         * Read here rather than at completion because the window is
         * where the borrowing happens. */
        width_burst = width_pending;
        return;
    }

    /* One read, both bits. Hsync and Vsync are on the same port by
     * necessity -- they must both be event generators and a port has
     * only two -- so this stays a single load however they are
     * assigned. If they were ever split across ports it would need to
     * become two, which is a reason the map keeps them together. */
    uint8_t in = PIN_HSYNC_PORT.IN;
    if (in & PIN_HSYNC_bm)  { if (pol_h_high != 0xFFFF) pol_h_high++; }
    else                    { if (pol_h_low  != 0xFFFF) pol_h_low++;  }
    if (in & PIN_VSYNC_bm)  { if (pol_v_high != 0xFFFF) pol_v_high++; }
    else                    { if (pol_v_low  != 0xFFFF) pol_v_low++;  }

    if (elapsed_since(now, pol_start) < POL_SAMPLE_TICKS) return;

    uint8_t h_idle_high = (pol_h_high > pol_h_low);
    uint8_t v_idle_high = (pol_v_high > pol_v_low);

    /* On ANY regenerated path the measured Hsync pin polarity is
     * irrelevant, because LUT1 is not looking at the pin. It sees
     * either TCB1's one-shot (Mode S dividing) or the TCE
     * reconstruction (H offset), and both are idle LOW with positive
     * pulses no matter what arrived.
     *
     * Using the measured value on those paths inverts Csync outright:
     * the combiner is told the two inputs agree at idle when in fact
     * one has been regenerated to the opposite sense, so it picks XNOR
     * where XOR was needed. The output then idles low with positive
     * spikes, and a display shown that loses vertical sync and rolls.
     *
     * This is the whole of the polarity immunity on the H side, and
     * note what it does NOT cover: Vsync still comes straight off its
     * pin, so a positive-going Vsync is handled by the XNOR/XOR choice
     * exactly as in every other mode, not by regeneration. */
    /* Keep the raw measurement before the forcing below hides it: the
     * H offset needs to know which edge of the incoming pulse the
     * hardware restart is landing on, and that is a property of the
     * source, not of the path Csync currently takes.
     *
     * UNCONDITIONAL, and it used to be gated on PASSTHRU -- which is
     * precisely the role H offset is never in. Engaging the offset put
     * the role in RECON and froze this value, so a source that flipped
     * its Hsync polarity while engaged left LUT3 restarting TCE0 on the
     * TRAILING edge of the pulse instead of the leading one. The
     * reconstruction is placed from that restart, so the origin moved by
     * one sync-pulse width -- 4.7us on a 63.6us line, about a tenth of
     * the picture width -- and the picture jumped sideways, in the
     * direction the flip went.
     *
     * The comment above the old gate said exactly what the code should
     * do and the code did the opposite. The samples come from PORTC.IN
     * directly, which is the source pin in every role, so there was
     * never a reason to gate it. */
    uint8_t pin_was = h_pin_idle_high;
    h_pin_idle_high = h_idle_high;

    if (ccl_current_role() != CCL_ROLE_PASSTHRU) h_idle_high = 0;

    /* The same argument on the V side, and it has to be made separately.
     * The combiner may not be looking at the Vsync pin either. Mode V
     * gives it pin 13; Mode FO gives it LUT0's delayed copy. Both idle
     * LOW with a positive-going pulse whatever the source sends, so
     * using the measured pin sense there would pick XNOR where XOR was
     * needed on a negative-going source, and invert Csync outright.
     * Measuring the pin is still correct; believing it describes what
     * the combiner sees is not.
     *
     * The raw answer is kept first, because LUT0 needs it: it is
     * building the very copy that makes the forcing true, so it must
     * know what the source actually does. */
    v_pin_idle_high = v_idle_high;

    /* PIN 13'S INVERSION IS BUILT FROM THE LINE ABOVE, so it has to be
     * rebuilt whenever that line changes its mind. A source swapped for
     * one of the opposite sense would otherwise keep the old inversion
     * and hand back a flipped V on pin 13 -- correct on the pin it was
     * measured for, wrong for the one now attached. Cheap: v_pin_apply()
     * recomputes from scratch and writes the same values when nothing
     * moved. */
    v_pin_apply();

    if (v_regen || fo_active) v_idle_high = 0;

    /* Only whether they AGREE matters. */
    uint8_t want_flip = (h_idle_high != v_idle_high);

    /* AN ABSENT VSYNC DOES NOT GET A VOTE.
     *
     * With no Vsync the pin sits whereever the pull-up and the cable
     * leave it, and the level is not a source property -- it is noise
     * with a preference. So the vote flips on one recheck and back on
     * the next, four times a second, and each flip rewrites the truth
     * table and raises a settle. Measured as the Mode-Valid LED pulsing
     * 40ms low, 210ms high: SETTLE_TICKS, and 40 + 210 = 250, which is
     * POL_RECHECK_MS.
     *
     * The flipping is not new. What is new is that the polarity apply
     * now raises a settle -- added to close a real gap, where a recheck
     * rewrote the LUTs with the output live -- so a fault that used to
     * be silent became visible. Making it visible was right; leaving it
     * flipping is not.
     *
     * Holding the previous answer is correct rather than merely quiet.
     * The combiner's V input is a constant when Vsync is absent, and
     * XNOR against a constant is XOR against it inverted: the choice
     * changes the sense of Csync, so a stable wrong answer is worse
     * than no change, and there is nothing to measure that would tell
     * us which is wrong. The last answer taken while a Vsync was
     * actually present is the best information available.
     *
     * The start-up case still gets one: pol_flip's initial value stands
     * until a source with Vsync arrives, which is the same thing the
     * rate check means by "absent Vsync passes". */
    if (!vsync_present) want_flip = pol_flip;

    pol_measuring = 0;
    pol_pending   = 0;
    pol_recheck   = 0;

    /* The burst is over and last_width_counts has had 20ms of lines to
     * settle on. Mode FO takes TCB1 back on the next fo_update() pass,
     * which is the same pass, so the timer changes hands inside the
     * settle mute this window already sits in. */
    if (width_burst) width_pending = 0;
    width_burst = 0;

    /* THE SECOND HALF OF THE SAME FAULT, and freeing h_pin_idle_high
     * alone would not have shown it.
     *
     * ccl_set_polarity() is what rewrites the edge sense's TRUTH1,
     * and it only ran when
     * pol_flip changed. In RECON h_idle_high is forced to 0 above --
     * correctly, since the combiner sees the reconstruction rather than
     * the pin -- so want_flip reduces to v_idle_high, and a source
     * flipping only its HSYNC polarity could not move it. LUT3 was
     * therefore doubly frozen: the value it reads could not change, and
     * nothing would have re-read it if it had.
     *
     * So the restart edge gets its own reason to be re-applied. */
    if (want_flip != pol_flip || h_pin_idle_high != pin_was) {
        /* MUTE AROUND IT, ON EVERY PATH THAT REACHES HERE.
         *
         * ccl_set_polarity() disables LUT1 to write the truth tables --
         * they are enable-protected -- and Csync is released for those
         * few cycles. Every other caller sits inside a settle window
         * already; this one did not. The pol_recheck path arrives here
         * periodically with the output live and nothing muting it, so
         * a source that changed polarity without changing rate got its
         * LUT rebuilt on air.
         *
         * settle_begin() here rather than at each call site, because
         * the rule is about what this function does to the output, not
         * about why it was called. */
        settle_begin(now);
        pol_flip = want_flip;
        ccl_set_polarity(pol_flip, h_pin_idle_high, mode_valid,
                         cs_hmode());
    }
}


/* ===================================================================
 * UI state -- which button table is in force
 * ===================================================================
 *
 * The provisional B1 that used to be described here -- a debounced
 * short press cycling the modes, ignoring the lockout, with no hold
 * gestures -- was replaced wholesale by section 8's scheme. What
 * follows is that scheme.
 */
/* Which button table is in force. Declared here rather than with the
 * dispatch because the LED preview needs it too, and that runs earlier. */
typedef enum {
    UI_NORMAL = 0,      /* M1-M5, or MS with its own table */
    UI_ADJ_H  = 1,
    UI_ADJ_V  = 2,
    UI_ADJ_FO = 3,
    UI_ADJ_ED = 4,      /* two axes, its own ladder, its own timeout */
    UI_ADJ_CS = 5       /* seven discrete methods, wraps, no neutral  */
} ui_state_t;

/* BOTH OF THE STATES ABOVE UI_ADJ_FO ARE OUTSIDE adj_defs, and the
 * guard in adj_cur() is what keeps that safe: anything past UI_ADJ_FO
 * gets the zero row, whose value pointer is null. Every branch that
 * would dereference it -- the release ladder and the idle discard --
 * has to test for these two FIRST. Mode ED already does; Mode CS is the
 * second, and there will not be a third without this comment being
 * read, because the failure is a null store rather than a diagnostic. */

static ui_state_t ui_state;

/* ===================================================================
 * Adjustment modes -- one engine, three settings (section 8.3)
 * ===================================================================
 *
 * Mode V was given the same step count and middle as Mode FO
 * deliberately, and Mode H the same shape again: stepping, the ends,
 * the middle pause, save, reset and the idle discard are then written
 * ONCE and shared rather than copied three times with three sets of
 * bugs.
 *
 * Each mode is a table row: where the live value lives, how many steps,
 * where the middle is, how many steps a press moves, and which EEPROM
 * slot it persists to. Adding a fourth is a row, not a function.
 *
 * The stride matters as much as the step. One H step is 56ns -- the
 * right resolution to STORE and far too fine to sweep, at 228 presses
 * end to end. Six at a time gives 38. V and FO are already coarse
 * enough at one line and one field, so they step singly. */
typedef struct {
    uint8_t *value;
    uint8_t  steps;         /* total positions, middle is steps/2 */
    uint8_t  stride;        /* steps moved per short press */
    uint8_t  ee_base;       /* first of the three profile slots */
} adj_def_t;

static uint8_t v_offset = V_OFFSET_DEFAULT;
static uint8_t f_offset = F_OFFSET_DEFAULT;

/* Mode FO's scaling, declared here because Mode V's emission now needs
 * it: on an interlaced source the regenerated pulses carry the field
 * offset themselves. The mode's own machinery is further down.
 *
 * Distance from the middle, 0 at centre and F_OFFSET_DEFAULT at either
 * end. The SIGN chooses which field moves, the MAGNITUDE says how far. */
static uint8_t fo_magnitude(void)
{
    return (uint8_t)((f_offset >= F_OFFSET_DEFAULT)
                   ? (f_offset - F_OFFSET_DEFAULT)
                   : (F_OFFSET_DEFAULT - f_offset));
}

/* Counts of delay the moved field receives, ZERO at centre. Scaled from
 * the MEASURED line period every time it is asked, not stored: half a
 * line is 32us at 15.7kHz and 16us at 31.5kHz, and a delay computed for
 * the wrong modeline is a picture the control cannot straighten.
 *
 * The arithmetic stays in 16 bits: half a line is under 700 counts at
 * any rate this device accepts, times a magnitude of at most 15 is under
 * 11000. */
static uint16_t fo_delta_counts(void)
{
    uint16_t mag = fo_magnitude();
    if (!mag) return 0;
    return (uint16_t)(((uint16_t)(last_period_counts >> 1) * mag)
                      / F_OFFSET_DEFAULT);
}

/* ===================================================================
 * Mode V -- vertical offset by regenerating Vsync (section 8.5)
 * ===================================================================
 *
 * The picture moves vertically by emitting Vsync a whole number of
 * LINES away from where the source put it, so the counting unit is the
 * Hsync capture and no timer is needed. Vertical position is quantised
 * to lines anyway, and the emitted pulse lands on a line boundary by
 * construction, so there is nothing finer to gain.
 *
 * THE TWO DIRECTIONS ARE NOT SYMMETRIC, and this is where the ATtiny
 * lost time. A POSITIVE offset is simply X lines after the pulse that
 * has just arrived -- it can be emitted this frame, no latency. A
 * NEGATIVE one cannot be emitted before the edge it is measured from,
 * so it becomes "a frame minus X lines after this pulse", which lands
 * in the NEXT frame. Getting that backwards gives an intermittent
 * Vsync, and an intermittent Vsync is a rolling picture.
 *
 * The emitted pulse is eight lines wide -- comfortably inside the
 * vertical blanking of anything this serves, and wide enough that no
 * separator can miss it.
 */
#define V_PULSE_LINES     8

/* THESE BELONG TO THE CAPTURE INTERRUPT and are written nowhere else.
 * That is the whole point of routing the frame boundary through
 * v_frame_req: with the capture at level 1 and the Vsync edge at level
 * 0, anything 16-bit shared between the two can be read half-updated.
 * v_line, v_prev_lines and v_frame_lines are declared with the
 * interrupt itself, which reads them before this block exists. */
static uint16_t v_start[2];             /* emission line, per pulse */
static uint8_t  v_pstate[2];            /* 0 waiting, 1 high, 2 done */
static uint8_t  v_npulse = 1;           /* 1 progressive, 2 interlaced */

/* The Vsync routing lives in evsys.c now, which owns every channel and
 * every event user. This wrapper stays because pol_pending is NOT an
 * event concern: the combiner's V input may have changed sense, and the
 * code that acts on that lives here.
 *
 * Passing v_regen and fo_active rather than letting evsys.c read them
 * is the point of the split -- the routing decision and the state it
 * depends on stay in one place, and that place is this file. */
/* PIN 13'S ONLY OWNER. Every decision about what drives that pin and
 * which way up lives here, and nowhere else touches DIRSET, OUTCLR,
 * PIN_VREGEN_CTRL or EVOUTD outside port_init() and v_line_tick().
 *
 * THREE CLAIMANTS, RESOLVED IN ONE PLACE:
 *
 *   Mode V's regeneration, which drives the pin from software with a
 *   rebuilt pulse and reads it straight back in through PORTD EVGEN1 as
 *   the combiner's V input.
 *
 *   Mode CS's pass and flip, which want the SOURCE's Vsync on the pin.
 *
 *   Mode CS's block, which wants it held at ground.
 *
 * WHEN REGEN IS RUNNING, PASS IS ALREADY SATISFIED. The rebuilt pulse
 * IS the Vsync the rest of the device is using, so pass does not need
 * to route anything -- it needs to not interfere. That is also why
 * BLOCK IS PROMOTED TO PASS while regen is on, per section 17.1: the
 * pin is load-bearing for the combiner and cannot be held low without
 * taking the V input away from Csync. A blocked V and an off-centre V
 * offset are not both satisfiable, and the offset wins because the
 * picture depends on it.
 *
 * POLARITY IS PORT'S JOB, NOT THE ROUTING'S, and it is the same shape
 * as the H side. "Pass" means the source's sense, and the two paths
 * arrive at it differently: EVOUTD carries the Vsync PIN's level, so it
 * is already right; the regenerated pulse is synthesised from the line
 * count and is positive-going whatever the source sent, so it needs
 * inverting when the source idles high. Hence
 *
 *     inv = v_regen && v_pin_idle_high
 *
 * against the combiner's (role != PASSTHRU) && h_pin_idle_high. Without
 * that term, nudging the V offset off neutral would silently flip the
 * output polarity on a negative-going source -- an offset control that
 * changes polarity, which is the sort of fault that gets blamed on the
 * display.
 *
 * INVEN IS SAFE HERE ONLY BECAUSE IT WAS MEASURED. Setting it while
 * regen is running inverts the pad AND the loopback that feeds the
 * combiner, so the two cancel and only the outside world sees the
 * change. That was measured on the bench: Csync unchanged on the scope
 * with INVEN set and Mode V regenerating. It is not what the data sheet
 * says, because the data sheet does not say. */
static void v_pin_apply(void)
{
    cs_vmode_t vm  = cs_vmode();

    /* MODE_VALID IS THE OUTERMOST TERM, AND IT OVERRIDES THE REGEN
     * PROMOTION. Everywhere else in this file a running regeneration
     * forces the pin on, because the combiner reads pin 13 back as its
     * V input and blocking it would take that input away (section
     * 17.5). That argument does not survive the gate being down: with
     * Csync muted there is no combiner output to feed, so there is
     * nothing left for the pin to be load-bearing FOR, and emitting a
     * lone Vsync into a display that has no Csync is worse than
     * emitting nothing at all.
     *
     * So this is deliberately ANDed outside the `|| v_regen`, not
     * folded into the method test -- the requirement is that pin 13 is
     * blocked whenever pin 12 is muted, in every method, with no
     * exception for Mode V. */
    uint8_t    out = (uint8_t)(mode_valid && (vm != CS_V_BLOCK || v_regen));
    uint8_t    inv = (uint8_t)((vm == CS_V_FLIP) ^
                               (v_regen && v_pin_idle_high));

    /* Blocked and not regenerating: ground, not floating, and not
     * inverted either -- INVEN left set from a previous method would
     * drive the pin HIGH on an OUTCLR and call it blocked. */
    if (!out) inv = 0;

    /* PINnCTRL written WHOLE. Pin 13 wants no pull-up, no input level
     * override and no interrupt, so INVEN alone is the complete value.
     * A read-modify-write here would be the fourth time this project
     * reached for |= on that register. */
    PIN_VREGEN_PORT.PIN_VREGEN_CTRL = inv ? PORT_INVEN_bm : 0;

    /* Ordering: EVOUTD is torn down inside ev_vsync_route() BEFORE
     * CHANNEL5 moves, so the pin can never drive its own input. The
     * PORT-side writes below are safe either side of it -- EVOUT
     * overrides the output register while it is enabled, so OUTCLR is
     * simply not visible until it is not. */
    ev_vsync_route(v_regen, fo_active, (uint8_t)(out && !v_regen));

    PIN_VREGEN_PORT.DIRSET = PIN_VREGEN_bm;

    if (!out) {
        /* Blocked: ground, and stay there. */
        PIN_VREGEN_PORT.OUTCLR = PIN_VREGEN_bm;
    } else if (!v_regen) {
        /* EVOUT IS DRIVING, AND PORT HOLDS THE SAME LEVEL UNDERNEATH
         * IT. The event output overrides the output register while it
         * is enabled, so this is invisible -- until the moment it is
         * not. Every teardown and restore inside ev_vsync_route() hands
         * the pin back to PORT for a few cycles, and if PORT is holding
         * 0 while the source idles high, that is a notch in the Vsync a
         * monitor is trying to lock to.
         *
         * OUT TAKES THE SOURCE'S IDLE LEVEL IN BOTH PASS AND FLIP, and
         * that is not an oversight. INVEN inverts the output register
         * as well as the input, so the pad sits at (inv XOR OUT); the
         * emitted idle wanted is (inv XOR source idle); the two agree
         * when OUT is the source's idle, whichever way inv is set. One
         * assignment, no case analysis, and it stays right if a method
         * is added. */
        if (v_pin_idle_high) PIN_VREGEN_PORT.OUTSET = PIN_VREGEN_bm;
        else                 PIN_VREGEN_PORT.OUTCLR = PIN_VREGEN_bm;
    }
    /* Regenerating: v_line_tick() owns OUT and is mid-frame. Touching
     * it here would flatten a pulse in progress -- the fault the
     * regenerated Vsync suffered under the OLD pinout, where it was
     * on pin 12 and the interlace indicator was left there with it. */
}

static void v_route_apply(void)
{
    v_pin_apply();
    pol_pending = 1;
}


/* Substitute the source. Only the routing changes; nothing downstream
 * knows the difference. */
static void v_set_source(uint8_t regen)
{
    if (regen == v_regen) return;
    v_regen = regen;
    v_route_apply();
}

/* Called from the CAPTURE interrupt, on the first line after the Vsync
 * ISR asked for it. Not from the Vsync ISR itself -- see v_frame_req.
 *
 * ONE REFERENCE PER FRAME, NOT PER FIELD, and this is the whole reason
 * Mode V can now run on an interlaced source at all.
 *
 * The obvious scheme -- reset on every field and place each field's
 * pulse independently -- cannot reconstruct interlace. Both pulses land
 * on the Hsync grid, so the half line that makes 525 out of two fields
 * of 262.5 is gone, and putting it back by delaying one field runs into
 * a rounding it cannot see: the two fields' resets land at Hsync indices
 * that differ by 262 OR 263 depending on where the true half line falls,
 * so the reconstructed separation is 262.5 or 263.5 and one of those is
 * wrong by a whole line. Worse, the ON-GRID field's Vsync edge is
 * coincident with an Hsync, so which capture follows it is a race that
 * can decide differently from one frame to the next -- jitter, not a
 * fixed error.
 *
 * So the counter is reset by the MID-LINE FIELD ONLY. That edge sits
 * squarely half a line from any Hsync, so the capture that follows it is
 * never in doubt. Both pulses are then placed from that single
 * reference -- the second at half the frame plus half a line -- and
 * their separation is exact by construction because there is only one
 * rounding in the whole scheme.
 */
static void v_frame_start(void)
{
    /* THE FRAME LENGTH MUST BE STABLE, and the raw count is not.
     *
     * A POSITIVE offset does not care what the frame length is, because
     * it is measured forward from the pulse that just arrived. A
     * NEGATIVE one is "frame minus X", so any error in the length moves
     * the emission point, and an emission point that moves every frame
     * is a rolling picture.
     *
     * The count used to be taken by polling TCB0 in the main loop, where
     * a pass slower than one line lost a capture outright -- the same
     * mechanism that made the mode-valid pin glitch during DDC reads.
     * That is gone: the count is advanced by a level 1 interrupt, which
     * nothing in this firmware can delay past a line.
     *
     * Two consecutive counts within one line before the length is
     * believed. Real frames repeat; a miscount does not repeat the same
     * way twice, so it is rejected without needing to know it happened.
     *
     * The tolerance of one used to be carrying the interlaced half line,
     * because the reference alternated between the two fields and the
     * count alternated with it. It no longer has to: on an interlaced
     * source only one field resets the counter, so successive counts are
     * whole frames and should agree exactly. The tolerance stays as
     * slack against a single lost capture, which is what it was for. */
    uint16_t n = v_line;
    v_line = 0;

    int16_t d = (int16_t)n - (int16_t)v_prev_lines;

    /* EXACT AGREEMENT, WHERE THIS USED TO ACCEPT PLUS OR MINUS ONE.
     *
     * The tolerance was believing the excursion. A deferred reference
     * makes that frame count L+1, which is one away and was therefore
     * accepted as the new frame length; the L-1 frame that follows is
     * then TWO away from L+1 and was rejected. So a single slip both
     * moved the reference AND poisoned the believed length for a frame,
     * and a negative offset -- computed as fl + off -- was moved twice.
     * The bench sees exactly that: a negative offset tics visibly
     * further than a positive one.
     *
     * The tolerance's own note already said it no longer carries the
     * interlaced half line and survives only as slack against a lost
     * capture. A lost capture is precisely a count that must not be
     * believed, so the slack was protecting the wrong thing.
     *
     * THE MIDDLE BRANCH IS NOT DECORATION. Without it the plus-or-minus
     * one frames fall through to the bad-frame counter, and the slip
     * burst runs d = +1, -2, +1 -- three increments, against a maximum
     * of four, before the following frame clears it. One ordinary slip
     * would sit one frame away from dropping the frame length outright,
     * which is a rolling picture in place of a one-line tic. So the
     * excursion is HELD: not believed, and not held against the source
     * either.
     *
     * The cost is that a source whose count genuinely alternates by one
     * every frame can no longer acquire a length from cold, where the
     * old test would have latched one of the two. That is the honest
     * answer for a source whose frame length will not settle -- and on
     * an interlaced source adj_locked_out() already says so out loud
     * rather than silently regenerating from a number it made up. */
    if (n && d == 0) {
        /* A DIFFERENT length means the slip carried against the old one
         * is meaningless. Re-phase from here rather than subtracting a
         * residue measured against a frame that no longer exists. */
        if (v_frame_lines != n) v_slip = 0;
        v_frame_lines = n;
        v_bad_frames  = 0;
    } else if (d == 1 || d == -1) {
        /* The one-line excursion. Hold the length, say nothing. */
    } else if (v_bad_frames < V_BAD_FRAMES_MAX
            && ++v_bad_frames >= V_BAD_FRAMES_MAX) {
        /* The alternation is being followed and the count still will not
         * settle, so the alternation itself is wrong -- a lost Vsync
         * edge inverts it permanently, and nothing downstream would ever
         * notice. Dropping the length forces re-acquisition from the
         * measurement, which is the only thing that can re-phase it. */
        v_frame_lines = 0;
        v_slip        = 0;
    }
    v_prev_lines = n;

    /* WHERE THE REFERENCE ENDED UP. Measured against the BELIEVED
     * length, not against the previous count: the question is how far
     * this reset is from where the source's frame boundary actually is,
     * and only the believed length knows that.
     *
     * A difference of more than one is not the excursion this corrects
     * -- it is a lost capture or a real change of mode -- so the
     * accumulator is abandoned rather than fed a number it cannot
     * interpret. The bad-frame counter above is what deals with that
     * case; this only has to avoid making it worse. */
    if (v_frame_lines && n) {
        int16_t r = (int16_t)n - (int16_t)v_frame_lines;
        if (r > 1 || r < -1) {
            v_slip = 0;
        } else {
            int8_t s = (int8_t)(v_slip + (int8_t)r);
            if (s >  V_SLIP_MAX) s =  (int8_t)V_SLIP_MAX;
            if (s < -V_SLIP_MAX) s = (int8_t)-V_SLIP_MAX;
            v_slip = s;
        }
    } else {
        v_slip = 0;
    }

    v_pstate[0]  = 0;
    v_pstate[1]  = 0;
    PIN_VREGEN_PORT.OUTCLR = PIN_VREGEN_bm;

    uint16_t fl  = v_frame_lines;
    int16_t  off = (int16_t)v_offset - V_OFFSET_DEFAULT;
    int16_t  sv;

    if (off >= 0)                        sv = off;
    else if (fl > (uint16_t)(-off))      sv = (int16_t)((int16_t)fl + off);
    else                                 sv = 0;   /* nonsense; emit now */

    /* CANCEL THE SLIP. The reference is v_slip lines LATER than the
     * source's frame boundary, so the emission has to be that many
     * lines EARLIER from it to land in the same place. Subtract.
     *
     * Signed all the way to here, because a small positive offset with
     * the reference one line late gives a negative emission point, and
     * on the uint16_t this used to be that is 65535 -- which
     * v_line_tick()'s "at or past" test would never reach, so no pulse
     * at all would be emitted and the frame would be dropped. A dropped
     * Vsync is a rolling picture, from a correction meant to stop a
     * one-line tic.
     *
     * Clamped at zero rather than wrapped into the previous frame: the
     * pulse is emitted from the line counter and there is no earlier
     * line than zero to emit it on. The residual error is at most
     * V_SLIP_MAX lines and only at the very bottom of the positive
     * travel, where the offset control has nowhere left to go anyway. */
    sv -= v_slip;
    if (sv < 0) sv = 0;

    uint16_t s0 = (uint16_t)sv;

    /* A PULSE MUST NOT STRADDLE THE REFERENCE THAT RESETS IT. The frame
     * boundary clears pin 13 and re-arms both pulses, so an emission
     * point within V_PULSE_LINES of the end of the frame is cut short --
     * a one-line Vsync where eight were intended. Only small negative
     * offsets reach this, which is precisely the range a user sweeping
     * gently through centre will cross. */
    if (fl > V_PULSE_LINES && s0 > (uint16_t)(fl - V_PULSE_LINES))
        s0 = (uint16_t)(fl - V_PULSE_LINES);

    v_start[0] = s0;
    v_npulse   = 1;

    if (v_ilace && fl) {
        /* The second pulse sits half a frame later, WRAPPED. A negative
         * offset puts the first pulse near the end of the frame, so
         * without the wrap the second would be placed past the frame
         * length and never emitted -- a Vsync missing every other field,
         * which is a rolling picture rather than a shifted one.
         *
         * Wrapping means the second pulse can be emitted BEFORE the
         * first within a frame. That is harmless: they are two pulses
         * half a frame apart either way, and the emission below treats
         * them independently rather than assuming an order. */
        uint16_t s1 = (uint16_t)(s0 + (fl >> 1));
        if (s1 >= fl) s1 = (uint16_t)(s1 - fl);
        v_start[1] = s1;
        v_npulse   = 2;
    }

    /* Adopted here rather than the moment the main loop decides it, so
     * the mode cannot change halfway through a count. The frame that
     * straddles the change is rejected by the stability test above. */
    v_ilace = v_ilace_want;
}

/* The delay this pulse's Vsync receives from TCB1, in counts.
 *
 * Half a line for the second pulse is what RECONSTRUCTS the interlace
 * the line-quantised regeneration destroyed. Mode FO's delta is then
 * added on top of it, to whichever pulse the sign selects:
 *
 *   separation = half-frame lines + delay2 - delay1
 *
 *   centre        262 + half - 0            = 262.5   interlaced
 *   full one way  262 + half - half         = 262.0   doublestrike
 *   full other    262 + (half + half) - 0   = 263.0   doublestrike,
 *                                                     other pairing
 *
 * So the delay never exceeds one whole line, which matters twice: it
 * fits TCB1's 16-bit CCMP with room to spare, and it is subtracted from
 * an eight-line pulse, leaving seven at worst. */
static uint16_t v_pulse_delay(uint8_t idx)
{
    uint16_t d = idx ? (uint16_t)(last_period_counts >> 1) : 0;

    if ((uint8_t)(idx != 0) == (uint8_t)(f_offset > F_OFFSET_DEFAULT))
        d = (uint16_t)(d + fo_delta_counts());

    return d ? d : 1;    /* never 0: a one-shot of zero is degenerate */
}

/* Called once per captured line. */
static void v_line_tick(void)
{
    if (!v_regen) return;

    /* THE GATE STOPS THE PIN, NOT THE MODE. v_pin_apply() grounds pin
     * 13 whenever mode_valid is down, and this function drives the same
     * pin from the level-1 ISR -- so without this return the next line
     * would put a regenerated pulse straight back on a pin that is
     * supposed to be blocked.
     *
     * IT IS THE PIN THAT STANDS DOWN, NOT MODE V. Gating v_set_source()
     * in the main loop was tried instead and is a LOCKUP: standing the
     * source down calls v_route_apply(), which sets pol_pending, which
     * is gated into mode_valid, which keeps the gate down, which stands
     * the source down again. That is the same loop recorded against
     * width_pending near the top of this file -- Mode FO engaging, then
     * standing down, then re-engaging forever with Csync muted
     * throughout. The shape is: never let a consumer of mode_valid
     * change routing, because routing sets pol_pending and pol_pending
     * is mode_valid.
     *
     * Returning here leaves v_regen set and the routing untouched, so
     * nothing feeds back. v_pstate and v_start keep advancing under
     * v_frame_start()'s per-frame reset, so the machine resumes on the
     * next frame reference rather than resuming mid-pulse -- and
     * v_pin_apply() has already put the pin where it belongs. */
    if (!mode_valid) return;

    uint16_t n = v_line;

    for (uint8_t i = 0; i < v_npulse; i++) {
        if (v_pstate[i] == 0) {
            /* AT OR PAST, never exactly equal.
             *
             * Exact equality assumes the line count never skips a value.
             * With the capture at level 1 it should not -- but "should
             * not" is not "cannot", and the failure mode of being wrong
             * is severe: miss the one value being watched for and NO
             * PULSE IS EMITTED. A dropped Vsync is a rolling picture,
             * from a single lost count.
             *
             * It also explained why positive offsets survived the polled
             * version and negative ones did not. A positive offset
             * watches for a line in the first few after the reference; a
             * negative one waits until nearly the end of the frame,
             * giving two hundred more chances to skip the value. */
            if (n >= v_start[i]) {
                /* LOADED IMMEDIATELY BEFORE THE EDGE THAT USES IT.
                 *
                 * TCB1's one-shot is triggered by pin 13 through
                 * CHANNEL5, and pin 13 is raised on the next line of
                 * this function. So writing CCMP here is exact: the
                 * write completes, then the pin rises, then the event
                 * reaches the timer. No deferral, no window in which the
                 * one-shot could be running -- the previous one finished
                 * half a frame ago.
                 *
                 * This is why the regenerated path does not use
                 * fo_load_service(). That exists for the case where the
                 * SOURCE supplies the edge and its arrival cannot be
                 * anticipated. Here we are the ones emitting it.
                 *
                 * Guarded on the job, not on fo_active: during the
                 * polarity window's width burst TCB1 is measuring Hsync,
                 * and writing a field delay into CCMP would corrupt the
                 * width the H reconstruction is built from. Both pulses
                 * then land on the grid for those 20ms, which is a
                 * doublestruck picture inside a settle mute. */
                if (v_ilace && tcb1_job == TCB1_JOB_FO) {
                    /* CNT FIRST, AND IT IS NOT BELT-AND-BRACES.
                     *
                     * The data sheet says an event "will reset and start
                     * counting from BOTTOM to TOP". The bench says
                     * otherwise when the counter is stopped ABOVE the
                     * new compare, which is exactly what this alternation
                     * produces: pulse two leaves CNT at half a line, and
                     * pulse one then wants a compare of 1.
                     *
                     * Left alone, the counter starts from 635, never
                     * meets 1 on the way up, wraps at 65535 and only
                     * stops the second time round. 64902 counts is
                     * 3.25ms -- about 51 lines -- and the output is high
                     * for all of it. Pulse one is eight lines, so LUT0
                     * swallowed it whole, every frame, leaving the
                     * combiner a Vsync at half the field rate.
                     *
                     * It even self-healed in a way that hid the cause:
                     * the overrun finished 3.25ms later, long before
                     * pulse two arrived 8.3ms on, and that shot then
                     * started from CNT=1 -- below its compare -- so
                     * pulse two always looked perfect. The alternation
                     * is always large-then-small, so the damage always
                     * landed on the same pulse.
                     *
                     * Clearing CNT makes the starting point unambiguous
                     * whichever way the silicon behaves. */
                    TCB1.CCMP = v_pulse_delay(i);
                    TCB1.CNT  = 0;
                }

                PIN_VREGEN_PORT.OUTSET = PIN_VREGEN_bm;
                v_pstate[i]  = 1;
            }
        } else if (v_pstate[i] == 1
                && n >= (uint16_t)(v_start[i] + V_PULSE_LINES)) {
            PIN_VREGEN_PORT.OUTCLR = PIN_VREGEN_bm;
            v_pstate[i]  = 2;
        }
    }
}

/* ===================================================================
 * Mode FO -- field offset (section 7, section 8.4)
 * ===================================================================
 *
 * WHAT IT IS FOR. An interlaced source puts its two fields half a line
 * apart, and that half line is what makes 525 lines out of two fields of
 * 262.5. Some displays -- and some scan converters upstream of them --
 * would rather have the fields on top of each other (doublestrike, a
 * stable 262-line progressive-looking picture with visible line gaps)
 * than interlaced. The control sweeps continuously between the two, so
 * the user can stop wherever their display is happiest.
 *
 * HOW THE PICTURE MOVES. Call the field whose Vsync lands on the line
 * grid A, and the one half a line later B. What matters is the SPACING
 * from A to B:
 *
 *      spacing 0.5 lines   ordinary interlace
 *      spacing 0           doublestrike, B on A's line n
 *      spacing 1.0         doublestrike, B on A's line n+1
 *
 * The last two look the same on one line and are not the same picture,
 * because they pair different lines together over a whole frame. That is
 * what section 7 means by "either end is doublestrike, with opposite
 * line pairings", and it is why one direction is not enough.
 *
 *      f_offset  15  neither field delayed          spacing 0.5
 *      f_offset < 15  A delayed by up to half a line spacing 0.5 -> 0
 *      f_offset > 15  B delayed by up to half a line spacing 0.5 -> 1.0
 *
 * WHICH END IS WHICH PAIRING IS NOT CALIBRATED. A and B are told apart
 * by an absolute measurement -- B's Vsync sits in the middle quarter of
 * a line -- so the two ends are genuinely different pictures and stay
 * consistent for a given source. But nothing here knows which pairing a
 * given display prefers, and it may differ between sources. The control
 * is symmetric on purpose: try both ends and keep the better.
 *
 * HOW THE DELAY IS MADE. TCB1 fires a one-shot on the Vsync leading
 * edge. LUT0 holds its copy of Vsync at idle for as long as that
 * one-shot is high, so the emitted leading edge lands CCMP counts late.
 * The trailing edge is untouched, so the pulse is SHORTENED by the
 * delay -- at most half a line, 32us, out of a Vsync pulse of two or
 * three lines. Nothing downstream measures Vsync width.
 *
 * WHY NOT TCE0's CMP2. It was the obvious idea and it does not work.
 * TCE0 is already restarted by every Hsync, so its count IS the sub-line
 * phase and a compare there would give the delay window with no timer at
 * all. But the window it produces is anchored at the start of the line,
 * so it can only ever catch field A -- half the range, no opposite
 * pairing. And section 9 notes that A's phase reads near zero OR near a
 * full period depending on which edge wins the race; in the second case
 * the window misses it completely and emits a narrow false edge just
 * before the restart instead of a delay.
 */


/* Rewritten once per field, a safe distance from any running one-shot.
 *
 * The Vsync interrupt latches on the pin's RISING edge, which is the
 * LEADING edge of a positive-going Vsync and the TRAILING edge of a
 * negative-going one. In the first case the one-shot is running at that
 * instant, and the data sheet is clear that writing CCMP mid-run gives
 * an unpredictable output. So the write is deferred: the one-shot lasts
 * at most half a line, and two milliseconds is sixty times that and an
 * eighth of the field it has to land inside. */
#define FO_LOAD_DELAY_MS   2

static void fo_load_service(uint16_t now)
{
    if (!fo_load_pending) return;
    if (elapsed_since(now, fo_load_at) < FO_LOAD_DELAY_MS) return;
    fo_load_pending = 0;

    if (tcb1_job != TCB1_JOB_FO) return;

    /* NOT WHEN MODE V IS REGENERATING. There the trigger edge is one we
     * emit ourselves, so v_line_tick() loads CCMP in the same breath as
     * raising the pin -- exact, and with no window in which the one-shot
     * could be running. Two writers for one register is the bug this
     * whole file keeps rediscovering; this is the one place they could
     * overlap, so it is refused here rather than sequenced. */
    if (v_regen) return;

    /* src_prev_mid is the parity of the field whose edge started this
     * timer. The one-shot fires next on the field AFTER it, so the
     * question is always about the other one. */
    uint8_t next_mid  = (uint8_t)(src_prev_mid ? 0 : 1);
    uint8_t move_mid  = (uint8_t)(f_offset > F_OFFSET_DEFAULT);

    uint16_t d = (next_mid == move_mid) ? fo_delta_counts() : 0;

    /* Same hazard as the regenerated path, same fix, and it applies for
     * the same reason: this alternates between a real delay and 1, so
     * every other load leaves CNT stopped above the new compare. See
     * v_line_tick() for what that costs. */
    TCB1.CCMP = d ? d : 1;      /* never 0: a one-shot of zero is degenerate */
    TCB1.CNT  = 0;
}

/* What LUT0's truth table and TCB1's trigger edge were BUILT for. Both
 * are decided from the source's Vsync sense, which is measured and can
 * change under us -- POL_RECHECK_MS re-votes four times a second
 * precisely because a source flipping polarity changes nothing that any
 * trigger watches. If that happened while Mode FO was engaged, LUT0
 * would keep gating on the old sense and emit an inverted Vsync, and
 * TCB1 would fire on the trailing edge, delaying the END of the pulse --
 * which moves nothing at all.
 *
 * Neither would announce itself. The picture would simply stop working
 * and the control would look broken, which is exactly the class of fault
 * "the combiner sees something other than the pin" keeps producing. */
static uint8_t fo_built_falling;

/* Engage or release the whole path. Called every main loop pass; does
 * nothing unless the answer has changed. */
static void fo_update(void)
{
    /* MANDATORY WHEN THE REGENERATION IS INTERLACED, even at centre.
     *
     * Everywhere else Mode FO is an optional adjustment that engages
     * only off centre. Here it is load-bearing: Mode V's pulses land on
     * the line grid, so without the one-shot supplying half a line to
     * the second of them there is no interlace at all -- centre would
     * give doublestrike and the control would have no neutral. */
    uint8_t want = (uint8_t)((f_offset != F_OFFSET_DEFAULT
                              || (v_regen && v_ilace))
                          && src_interlaced
                          && vsync_present
                          && last_period_counts
                          && ccl_current_role() != CCL_ROLE_DIVIDE
                          && !width_burst);

    /* Already engaged, but built for the other polarity: rebuild in
     * place rather than dropping out and back, which would flick the
     * combiner's input twice for no reason. */
    if (want && fo_active && fo_built_falling != fo_trigger_falling()) {
        fo_built_falling = fo_trigger_falling();
        tcb1_set_job(TCB1_JOB_NONE);      /* force the rewrite */
        tcb1_reevaluate();
        ccl_fo_lut0_build(1, fo_trigger_falling());
        pol_pending = 1;
        return;
    }

    if (want == fo_active) return;

    if (want) {
        /* TCB1 FIRST. LUT0 reads the one-shot's output, so the timer has
         * to be doing this job before the LUT starts listening to it --
         * otherwise the combiner briefly sees Vsync gated by whatever
         * the reshaper or the width capture left on that output. */
        fo_built_falling = fo_trigger_falling();
        fo_active = 1;
        tcb1_reevaluate();
        ccl_fo_lut0_build(1, fo_trigger_falling());
        v_route_apply();
    } else {
        /* And the reverse on the way out: take the combiner off LUT0
         * before LUT0 stops being driven by anything. */
        fo_active = 0;
        v_route_apply();
        ccl_fo_lut0_build(0, fo_trigger_falling());
        tcb1_reevaluate();
    }
}

static const adj_def_t adj_defs[4] = {
    { 0,          0,               0,               0          },
    { &h_offset,  H_OFFSET_STEPS,  HOFF_UI_STRIDE,  EE_HOFF_P1 },
    { &v_offset,  V_OFFSET_STEPS,  1,               EE_VOFF_P1 },
    { &f_offset,  F_OFFSET_STEPS,  1,               EE_FOFF_P1 },
};

static const adj_def_t *adj_cur(void)
{
    return &adj_defs[(ui_state <= UI_ADJ_FO) ? ui_state : 0];
}

static uint8_t  b1_state = 1;         /* idle high */
static uint16_t b1_change;

/* ===================================================================
 * Blink engine -- the Blink LED, pin 5
 * ===================================================================
 *
 * A pattern is a list of signed millisecond durations: positive is LED
 * high for that long, negative is low, zero ends it. So the Mode S code
 * -- two 200ms pulses, a 700ms gap, then the same again -- is just
 * { 200,-200,200,-700,200,-200,200,0 }.
 *
 * TWO INDEPENDENT PLAYERS, and section 8.8 requires it: "the indicator
 * must own its state... anything critical should drive the pin directly
 * rather than share a sequence buffer with the mode blinks and stop
 * pulses."
 *
 * The reason is what happens when two patterns collide. The cannot-help
 * pattern is a RESPONSE TO A PRESS -- it means "I heard you, I cannot
 * act". Share one buffer and, if a 1200ms strobe is already running, it
 * either waits until the strobe ends (arriving a second late, by which
 * time the user has pressed again), gets dropped, or overwrites the
 * strobe halfway and produces a third pattern that means nothing. All
 * three read as a broken button, so the user presses harder, in a mode
 * that by definition cannot respond.
 *
 * So urgent has its own buffer and its own timer, pre-empts the normal
 * player, and abandons rather than resumes it -- a mode report cut in
 * half has lost its meaning anyway.
 */
#define BLINK_MAX   28        /* longest is the 22s hold preview */

static int16_t  blink_seq[BLINK_MAX];
static uint8_t  blink_len, blink_idx;
static uint16_t blink_started;
static uint8_t  blink_active;

static int16_t  urgent_seq[BLINK_MAX];
static uint8_t  urgent_len, urgent_idx;
static uint16_t urgent_started;
static uint8_t  urgent_active;

/* ===================================================================
 * Blink LED software PWM
 * ===================================================================
 *
 * No hardware PWM is available for it. TCF0's outputs reach only
 * PA0/PA1 and are left disabled; both TCBs are committed, TCB0 to the
 * Hsync capture and TCB1 to its three one-shot jobs; and TCE0's period
 * IS the line, so its output would stop whenever the source did --
 * which is precisely when the indicator matters most. So it is done in
 * software off the housekeeping tick, which tick.c runs at 4kHz for
 * this purpose.
 *
 * 16 levels at 4kHz is a 250Hz frame. blink_duty is one byte, so the
 * main loop writing it cannot be caught half-updated by the interrupt
 * reading it, and no guard is needed. pwm_phase belongs to the
 * interrupt and is written nowhere else. */
#define PWM_LEVELS   16

static volatile uint8_t blink_duty;     /* 0..PWM_LEVELS, set by main loop */
static uint8_t          pwm_phase;      /* interrupt-private */

/* Called from the TCF0 ISR, 4000 times a second. Keep it this short.
 * Not static: tick.c's ISR is the caller, and tick.h declares it. */
void tick_pwm_service(void)
{
    uint8_t d = blink_duty;
    if (++pwm_phase >= PWM_LEVELS) pwm_phase = 0;
    if (pwm_phase < d) PIN_BLINK_PORT.OUTSET = PIN_BLINK_bm;
    else               PIN_BLINK_PORT.OUTCLR = PIN_BLINK_bm;
}

/* THE ONE PLACE THE BLINK LED PIN IS DRIVEN AT RUN TIME.
 *
 * Section 8.8 counted five functions and seven write sites on this pin
 * -- blink_stop, both writes in blink_run_one, the Mode FO rate blink,
 * the hold preview and blink_service's idle clear -- with the standing
 * warning that anything needing to wrap the pin's transition has to
 * unify them FIRST or it will miss one. Software PWM is exactly that, so this is that unification, done
 * on its own with no behaviour change: every site below now asks for a
 * LEVEL and this decides what the pin does about it.
 *
 * port_init() is deliberately NOT routed through here. It runs before
 * tcf0_init(), so once this function owns a PWM state machine driven
 * from the tick, calling it at that point would be touching a timebase
 * that does not exist yet. Boot writes the pin low directly; the level
 * this function tracks starts at zero in BSS, so the two agree without
 * needing to be sequenced.
 *
 * THE SETTER NO LONGER TOUCHES THE PIN. It sets a DUTY, and the PWM
 * sub-tick below is the only thing that writes PORTA at run time. That
 * is what the unification was for: with two writers -- a main-loop one
 * for the on/off cases and an interrupt one for the ramp -- the pin's
 * level would depend on which ran last, and the answer would change
 * with the duty. One writer has no such question.
 *
 * Cost: a transition is quantised to one sub-tick, 250us. The fastest
 * thing this LED is ever asked to do is Mode FO's 20ms half-period at
 * the ends of its range, where 250us is 1.25% -- and the eye is not
 * the instrument that would find it. */
__attribute__((always_inline))
static inline void blink_set(uint8_t on)
{
    blink_duty = on ? PWM_LEVELS : 0;
}

/* Perceived brightness goes roughly as the 2.2 root of duty, so a
 * linear duty ramp spends most of its travel looking bright. This maps
 * an even PERCEPTUAL index onto the duty that produces it.
 *
 * THE BOTTOM OF THIS TABLE IS WHERE 16 LEVELS RUNS OUT, and the shape
 * is deliberate. A true gamma-2.2 curve over the full range rounds to
 * duty 0 for the first FOUR indices, so a pulse would sit black for an
 * eighth of its cycle and then jump -- a stall, not a fade. The curve
 * here is floored at duty 1 from index 1 upward, which costs three
 * repeats near the bottom and reads as a soft start instead.
 *
 * If the dark end still looks steppy on the bench, the fix is more
 * levels, not a different curve: PWM_LEVELS to 32 and PWM_HZ in tick.c
 * to 8000, which holds the frame at 250Hz and doubles the interrupt
 * load. Two constants. Do not reach for it before looking. */
static const uint8_t pwm_gamma[PWM_LEVELS] = {
    0, 1, 1, 1, 2, 2, 3, 3, 4, 5, 7, 8, 10, 12, 14, 16
};

static void blink_level(uint8_t i)
{
    blink_duty = pwm_gamma[i & (uint8_t)(PWM_LEVELS - 1)];
}

/* Off to full to off, from a millisecond timestamp. The period is 1024
 * milliseconds rather than 1000 so the whole thing is masks and shifts
 * with no division on a path that runs every main-loop pass: bit 9
 * picks the rising or falling half. 1.024s where the spec says one
 * second.
 *
 * THE TIME AXIS IS WARPED, AND THAT IS A SEPARATE JOB FROM THE GAMMA
 * TABLE. pwm_gamma makes PERCEIVED brightness proportional to its
 * index; feeding it a ramp that is linear in time therefore gives a
 * pulse whose perceived brightness rises at a constant rate, which
 * spends most of its cycle looking lit. Squaring the time axis first
 * makes the ramp loiter at the bottom and hurry through the top.
 *
 * THE PULSE IS ALSO ASYMMETRIC: 640ms rising, 384ms falling. The climb
 * is deliberately the slower half and the drop is 40% quicker, which is
 * what stops the peak reading as a plateau. There is no flat dark tail
 * -- the fall runs straight into the next rise -- so it still reads as
 * a breath rather than a blink, with 256ms of the cycle fully off and
 * 160ms in the top third.
 *
 * The two multiplies are what buy the odd split. 32 slots across 640ms
 * is p/20 and across 384ms is q/12, neither a shift; (p*51)>>10 and
 * (q*85)>>10 are those ratios to within a slot, and both intermediates
 * stay inside 16 bits (639*51 and 383*85 are both under 32768). AVR has
 * a hardware multiplier, so this is cheaper than the division would be.
 *
 * Both halves reach j = 31 exactly, so the peak is continuous across
 * the changeover and (31*31)>>6 = 15 cannot exceed the table.
 * blink_level() masks as well, but it should never have to. */
static void blink_pulse(uint16_t t)
{
    uint16_t p = (uint16_t)(t & 0x03FF);
    uint8_t  j = (p < 640) ? (uint8_t)(((uint16_t)p * 51) >> 10)
                           : (uint8_t)((((uint16_t)(1023 - p)) * 85) >> 10);
    blink_level((uint8_t)(((uint16_t)j * j) >> 6));
}

static void blink_stop(void)
{
    blink_active = 0;
    if (!urgent_active) blink_set(0);
}

static void blink_play(const int16_t *seq, uint8_t n, uint16_t now)
{
    if (n > BLINK_MAX) n = BLINK_MAX;
    for (uint8_t i = 0; i < n; i++) blink_seq[i] = seq[i];
    blink_len     = n;
    blink_idx     = 0;
    blink_started = now;
    blink_active  = n ? 1 : 0;
}

/* Pre-empts whatever is playing. Used by the cannot-help lockout. */
static void blink_urgent(const int16_t *seq, uint8_t n, uint16_t now)
{
    if (n > BLINK_MAX) n = BLINK_MAX;
    for (uint8_t i = 0; i < n; i++) urgent_seq[i] = seq[i];
    urgent_len     = n;
    urgent_idx     = 0;
    urgent_started = now;
    urgent_active  = n ? 1 : 0;
    blink_active   = 0;            /* abandoned, not suspended */
}

/* Whether either player owns the LED. The rate blinks and the Mode CS
 * count test this before taking the pin, because neither is a pattern
 * the blink engine could hold. */
static uint8_t blink_busy(void)
{
    return (uint8_t)(blink_active || urgent_active);
}

/* Advances one player. Returns 1 if it drove the pin this pass. */
static uint8_t blink_run_one(int16_t *seq, uint8_t len, uint8_t *idx,
                             uint16_t *started, uint8_t *active, uint16_t now)
{
    if (!*active) return 0;

    for (;;) {
        if (*idx >= len || seq[*idx] == 0) {
            *active = 0;
            blink_set(0);
            return 1;
        }
        int16_t  d  = seq[*idx];
        uint16_t ms = (uint16_t)((d < 0) ? -d : d);
        if (elapsed_since(now, *started) < ms) {
            blink_set(d > 0);
            return 1;
        }
        *started = (uint16_t)(*started + ms);
        (*idx)++;
    }
}

/* ===================================================================
 * The patterns of section 8
 * =================================================================== */

/* Mode S code: 2 x 200ms high 200ms apart, 700ms gap, then again.
 * 1.9s total, which is what makes two of them plus a 1.2s gap span
 * 9.0s to 14.0s on the held-button timeline. */
static const int16_t pat_modeS[] = {
    200, -200, 200, -700, 200, -200, 200, 0
};

/* Cannot-help: 3 x 100ms, 3 x 300ms, 3 x 100ms, then 1200ms dark.
 *
 * Played while a control CANNOT ACT -- Mode V on an interlaced source,
 * Mode FO on a progressive one. Note "while", not "when pressed": it is
 * driven by the condition, so it starts on its own and repeats for as
 * long as the condition holds. A pattern that only appeared on a press
 * would leave a user who has not pressed yet with a mode that silently
 * does nothing.
 *
 * The 1200ms of dark at the end IS the gap between repeats, which is why
 * the repeat logic only has to wait for the player to go idle.
 *
 * Deliberately unlike anything else so it cannot be mistaken for a
 * count: nine pulses, three lengths, no pause a code could hide in. */
static const int16_t pat_cannot_help[] = {
    100, -100, 100, -100, 100, -300,
    300, -300, 300, -300, 300, -300,
    100, -100, 100, -100, 100, -1200, 0
};

/* 1200ms strobe: the ends of an adjustment range, and passing centre. */
static const int16_t pat_strobe[] = { 1200, 0 };

/* ===================================================================
 * Mode FO's rate blink (section 8.4)
 * ===================================================================
 *
 * Mode H and Mode V are dark: their effect is visible on the picture, so
 * the LED has nothing to add. Mode FO is not, because both ends look
 * alike -- doublestrike either way -- and the middle is just the picture
 * the source already sent. Without an indication there is no way to tell
 * where in the range you are, or which direction the next press moves.
 *
 * So the LED blinks at a rate that reads position directly: slow at the
 * centre, fast at either end. Half-periods in milliseconds, geometric
 * from 200 (a 400ms period) down to 20 (40ms), indexed by fo_magnitude()
 * -- 0 at centre, 15 at either end.
 *
 * Geometric rather than linear because the eye reads blink rate as a
 * ratio: a linear ramp spends most of its travel looking the same and
 * then changes character abruptly near the end.
 *
 * These are half-periods, not durations, so the table is uint8_t and 200
 * is the largest value in it. Sixteen bytes.
 */
static const uint8_t fo_blink_half[F_OFFSET_DEFAULT + 1] = {
    200, 172, 147, 126, 108, 93, 80, 68,
     59,  50,  43,  37,  32, 27, 23, 20
};

static uint16_t fo_blink_at;
static uint8_t  fo_blink_on;

static void fo_blink_reset(uint16_t now)
{
    fo_blink_at = now;
    fo_blink_on = 0;
}

/* Drives the Blink LED directly. Only called when nothing with a stronger
 * claim -- a hold preview, a strobe, the cannot-help pattern -- wants
 * it, so it never has to negotiate. */
static void rate_blink(uint8_t half, uint16_t now)
{
    if (elapsed_since(now, fo_blink_at) >= half) {
        fo_blink_at = now;
        fo_blink_on ^= 1;
    }

    blink_set(fo_blink_on);
}

static void fo_rate_blink(uint16_t now)
{
    rate_blink(fo_blink_half[fo_magnitude()], now);
}

/* Mode ED borrows the same slow-at-centre, fast-at-the-ends mapping,
 * and for the same reason Mode FO needs it: the picture is the only
 * other feedback and it says nothing about how far from neutral you
 * are.
 *
 * fo_blink_half has 16 entries and ED's travel is +/-ED_STEPS_MAX, so
 * the magnitude is scaled rather than indexed directly: *3>>3 maps 40
 * onto 15 exactly and cannot overshoot the table.
 *
 * THE RATE IS AGAINST THE NOMINAL RANGE, NOT THE ACHIEVABLE ONE. The
 * achievable end moves with the mode and with the size setting, so a
 * rate scaled to it would change meaning underneath the user every time
 * they touched the other axis. The strobe is what announces the real
 * end; this only says how far from centre. */
static void ed_rate_blink(uint16_t now)
{
    int8_t  v   = *ed_val();
    uint8_t mag = (uint8_t)((v < 0) ? -v : v);
    rate_blink(fo_blink_half[(uint8_t)((mag * 3u) >> 3)], now);
}

/* ===================================================================
 * Mode CS's method report (section 17.1)
 * ===================================================================
 *
 * COUNT, NOT RATE. Mode FO and Mode ED blink at a rate because their
 * settings are positions on a continuum, where the useful question is
 * "how far from centre". A combine method is not a position: there is
 * no centre, method 5 is not further from anything than method 2, and a
 * rate would be reading a scale that does not exist. So the LED says
 * the number.
 *
 * BREATHS, NOT BLINKS. The count is played with the brightness ramp
 * rather than hard on/off, for the same reason Mode ED's fourth hold
 * band uses it: brightness is the one axis nothing else on this pin
 * claims. Six 200ms blinks would be indistinguishable from a mode code
 * or a profile report caught mid-play; six swells cannot be mistaken
 * for either.
 *
 * 512ms per count -- a 256ms swell then 256ms dark -- so the phase is
 * (t & 511) and the index is (t >> 9), with no division on a path that
 * runs every main loop pass. The swell is blink_pulse()'s 1024ms shape
 * played at four times speed, which puts the peak 160ms in and the
 * whole thing back to black with a quarter second to spare. Method 6
 * takes 3.1s.
 *
 * A NEW PRESS RESTARTS THE COUNT rather than being swallowed. The
 * adjustment engine swallows presses during its 1200ms strobes because
 * a strobe means stop and look at a landmark, and walking past it is
 * the mistake being prevented. Nothing here is a landmark -- the whole
 * range is six equally valid choices -- so swallowing would just make
 * the button feel dead for three seconds. The user who keeps pressing
 * sees the count restart each time and the full number when they stop,
 * which is what a counter should do. */
#define CS_SLOT_MS      512u      /* per count: swell then dark  */
#define CS_SWELL_MS     256u      /* the lit part of a slot      */

/* DARK BEFORE THE COUNT, ON ENTRY ONLY.
 *
 * Reaching Mode CS means holding the button through eleven seconds of
 * LED preview and then two seconds of dark. A count that begins the
 * instant the button comes up runs straight out of that dark with no
 * seam, and the first swell reads as part of the gesture rather than as
 * the answer to it -- so the number is short by one, or missed.
 *
 * A PRESS INSIDE THE MODE GETS NO LEAD-IN. There the count is a reply
 * to something the user just did deliberately, the previous count is
 * still fresh, and 800ms of nothing would make the button feel like it
 * had not registered. Same report, different question being answered. */
#define CS_ENTRY_LEAD_MS 500u

static uint16_t cs_report_at;
static uint16_t cs_report_lead;   /* ms of dark before the first swell */
static uint8_t  cs_report_n;      /* counts left to play; 0 = idle */

static void cs_report_start(uint16_t now, uint16_t lead)
{
    cs_report_at   = now;
    cs_report_lead = lead;
    cs_report_n    = cs_method;
    /* Abandon whatever the pattern players were doing. Entering the
     * mode stops the mode-and-profile report mid-word, and that is
     * correct: it was answering a question the user has already moved
     * on from. */
    blink_stop();
}

/* Drives the pin directly, like rate_blink(), and for the same reason:
 * it is not a pattern the blink engine could hold, because its shape is
 * a brightness curve rather than a list of durations.
 *
 * Only ever called with cs_report_n non-zero, so the idle case below is
 * the END of a report and not a guard. */
static void cs_report_run(uint16_t now)
{
    uint16_t t = elapsed_since(now, cs_report_at);

    /* SUBTRACTED FROM THE ELAPSED TIME, not added to the start stamp.
     * A start stamp in the future breaks elapsed_since(): the unsigned
     * difference wraps to something near 65535, which reads as "long
     * finished" rather than "not yet begun", and the count would be
     * skipped entirely. */
    if (t < cs_report_lead) { blink_set(0); return; }
    t = (uint16_t)(t - cs_report_lead);

    if (t >= (uint16_t)(cs_report_n * CS_SLOT_MS)) {
        cs_report_n = 0;
        blink_set(0);
        return;
    }

    uint16_t r = (uint16_t)(t & (CS_SLOT_MS - 1u));
    if (r < CS_SWELL_MS) blink_pulse((uint16_t)(r << 2));
    else                 blink_set(0);
}

/* Builds N pulses of on/off ms into a buffer, returns entries used. */
static uint8_t pat_pulses(int16_t *buf, uint8_t n, int16_t on, int16_t off)
{
    uint8_t k = 0;
    while (n--) { buf[k++] = on; buf[k++] = (int16_t)-off; }
    return k;
}

/* Mode-and-profile report (section 8.1). Mode code, 1000ms dark, then
 * the profile number. Played on entering any numbered mode, on entering
 * MS, on a short press while locked, and at power-on. */
static void report_mode_profile(uint16_t now)
{
    int16_t buf[BLINK_MAX];
    uint8_t k = 0;

    if (current_mode == MODE_S) {
        /* Mode S code ONLY -- no profile.
         *
         * Section 8.1 appends the profile number to every report, but in
         * Mode S that is 200ms pulses following a code built entirely
         * from 200ms pulses, and the two cannot be told apart on the
         * bench. The mode code has to stay legible; the profile is
         * available from any numbered mode, where the 500ms mode pulses
         * make the 200ms profile pulses unmistakable. */
        for (uint8_t i = 0; pat_modeS[i] && k < BLINK_MAX - 1; i++)
            buf[k++] = pat_modeS[i];
    } else {
        buf[k++] = -750;                       /* 750ms dark first */
        k += pat_pulses(buf + k, current_mode, 500, 500);
        buf[k++] = -1000;
        k += pat_pulses(buf + k, current_profile, 200, 200);
    }

    buf[k] = 0;
    blink_play(buf, (uint8_t)(k + 1), now);
}

/* Held-button preview -- WHAT THE LED DOES WHILE B1 IS DOWN.
 *
 * Drives the pin directly from elapsed time rather than playing a
 * sequence, because it has to track the hold as it happens and a
 * sequence would need restarting on every press.
 *
 * KEEP THIS SEPARATE FROM WHAT RELEASING DOES. The preview says what
 * releasing NOW will do; the report plays AFTER release. Both timelines
 * start dark, and that dark is not silence:
 *
 *   unlocked, held < 2s   dark          release: advance mode, then the
 *                                       mode-and-profile report
 *   locked,   held < 3s   dark          release: the mode-and-profile
 *                                       report ONLY -- no mode change,
 *                                       no profile change, no MS
 *
 * A locked unit therefore still answers a short press by blinking what
 * it is; it simply refuses to be changed. Section 8.1a: "a locked
 * dongle is a working dongle that cannot be misconfigured." A unit that
 * ignored the button outright would read as broken. */
static void hold_preview(uint16_t held, uint16_t now)
{
    (void)now;
    uint8_t on = 0;

    if (b1_locked) {
        /* Nothing under 3s, fast blinks to 10s, then dark. Applies in
         * every mode, including Mode S, so a locked unit can always be
         * unlocked wherever it happens to be. */
        if (held >= HOLD_L_FACTORY_MS)      on = 0;
        else if (held >= HOLD_L_UNLOCK_MS)  on = (uint8_t)((held / 150) & 1);
    } else if (ui_state == UI_ADJ_ED) {
        /* FOUR bands, and the fourth is why the PWM exists.
         *
         * 2-4s solid, 4-6s dark, 6-8s solid again -- alternating, so
         * adjacent windows are never confused. That leaves nothing for
         * the axis change at 8s+ that is not already in use, and a
         * fourth band made of blinks would collide with the lockout's
         * blinks further up every other ladder in this firmware.
         *
         * Brightness is the one axis nothing else on this pin uses. */
        if (held >= ED_AXIS_MS) {
            blink_pulse(held);
            return;                       /* blink_pulse owns the pin */
        }
        on = (uint8_t)((held >= ED_REVERSE_MS && held < ED_SAVE_MS)
                    || (held >= ED_RESET_MS));
    } else if (ui_state != UI_NORMAL) {
        /* One indication per hold action, alternating so adjacent
         * windows can never be confused: 2s reverse (on), 4s save
         * (off), 6s reset (on).
         *
         * MODE CS USES THIS UNCHANGED even though its 2-4s rung is
         * inert. A case was tried that darkened 2-4s on the grounds
         * that the LED should not light over a window a release does
         * nothing in, and it was REJECTED: the lit band is what makes
         * the 4s save boundary findable. Going dark at 4s is the
         * signal. Darkening 2-4s as well leaves 0-6s uniformly dark and
         * the boundary can then only be found by counting, which is the
         * fault the Mode S preview was re-cut to avoid.
         *
         * So in Mode CS the lit 2-4s band means "keep holding, save is
         * next" rather than "release here". That is a weaker promise
         * than the same band makes in H/V/FO, and it is the right
         * trade: an inert release costs nothing, an unfindable save
         * boundary costs the setting. */
        on = (uint8_t)((held >= ADJ_REVERSE_MS && held < ADJ_SAVE_MS)
                    || (held >= ADJ_RESET_MS));
    } else if (current_mode == MODE_S) {
        /* Section 8.2, three windows and they must be told apart:
         * solid 2-4s is Mode H, dark 4-6s is Mode V, blinking from 6s is
         * Mode FO. Leaving the last two both dark meant the only way to
         * know which one a release would land in was to count seconds. */
        if (held >= HOLD_MODE_FO_MS)
            on = (uint8_t)((held / 200) & 1);
        else
            on = (uint8_t)(held >= HOLD_MODE_H_MS && held < HOLD_MODE_V_MS);
    } else if (held >= HOLD_FACTORY_MS) {
        on = 0;                              /* last warning: dark */
    } else if (held >= HOLD_LOCK_MS) {
        on = (uint8_t)((held / 150) & 1);    /* lockout window */
    } else if (held >= HOLD_ED_ENTER_MS) {
        on = 0;                              /* Mode ED window: dark   */
    } else if (held >= HOLD_CS_ENTER_MS) {
        on = 1;                              /* Mode CS window: solid  */
    } else if (held >= HOLD_MS_ENTER_MS) {
        /* ONE Mode S code, 8.0s to 9.9s, then dark to 11.0s.
         *
         * RE-CUT, NOT MOVED. This used to play the code twice with a
         * 1200ms gap, spanning 9.0s to 14.0s -- which now runs straight
         * through both the CS and ED windows and would have had the LED
         * promising Mode S while a release meant something else
         * entirely. The window is 3s wide and the code is 1.9s, so it
         * plays once and the rest is the gap before the CS window
         * lights. */
        uint16_t t = (uint16_t)(held - HOLD_MS_ENTER_MS);
        if (t < 1900) {
            on = (t < 200) || (t >= 400 && t < 600)
              || (t >= 1300 && t < 1500) || (t >= 1700 && t < 1900);
        }
    } else if (held >= HOLD_PROFILE1_MS) {
        /* 1, 2 or 3 pulses of 200/200 at 2s, 4s and 6s. */
        uint8_t n = (held >= HOLD_PROFILE3_MS) ? 3
                  : (held >= HOLD_PROFILE2_MS) ? 2 : 1;
        uint16_t base = (n == 3) ? HOLD_PROFILE3_MS
                      : (n == 2) ? HOLD_PROFILE2_MS : HOLD_PROFILE1_MS;
        uint16_t t = (uint16_t)(held - base);
        if (t < (uint16_t)(n * 400)) on = (uint8_t)(((t / 200) & 1) == 0);
    }

    blink_set(on);
}

static void blink_service(uint16_t now)
{
    if (blink_run_one(urgent_seq, urgent_len, &urgent_idx,
                      &urgent_started, &urgent_active, now)) return;
    if (blink_run_one(blink_seq, blink_len, &blink_idx,
                      &blink_started, &blink_active, now)) return;

    /* Idle: the pin is ours, and ours means dark.
     *
     * Without this the LED keeps whatever level the hold preview left
     * it at. Releasing during the high half of a 150ms blink -- which
     * the unlock gesture makes likely, since it ends in that pattern --
     * left the LED on with nothing left running to turn it off. Neither
     * player was active, so neither touched the pin, and it simply
     * stayed lit. */
    blink_set(0);
}

/* ===================================================================
 * H offset engine -- TCE0 restarted by Hsync IN HARDWARE
 * ===================================================================
 *
 * This replaced the free-running genlock of increment 4a, and the
 * reasoning is worth keeping because the spec says the opposite.
 *
 * Section 6.3 chose a free-running counter to avoid the 50ns
 * quantisation that comes from restarting a timer on every incoming
 * edge. That is a real effect and the ATtiny824 had it. But a
 * free-running counter has to be steered, steering needs the incoming
 * phase measured, and measuring it needs an interrupt -- so the loop
 * inherits the interrupt's latency jitter, the source's own jitter, and
 * its own settling behaviour. Measured on this board: 400ns of edge
 * spread rejecting source jitter, 200ns tracking it. The 50ns it was
 * avoiding was four to eight times smaller than the cost of avoiding
 * it.
 *
 * TCE0.EVCTRL.EVACTB = RESTART_POSEDGE restarts the counter from the
 * Hsync event ENTIRELY IN HARDWARE. No interrupt, no measurement, no
 * loop, and nothing in the signal path that software can be late for.
 * The only quantisation left is the restart aligning to CLK_PER: 50ns,
 * deterministic, and it does not accumulate.
 *
 * The pulse is then placed by CMP0 and CMP1, which the high-resolution
 * extension resolves to 6.25ns -- so unlike the ATtiny824, where both
 * the restart and the placement were quantised at 50ns, only the
 * restart is. And because the counter is referenced to the incoming
 * edge, source jitter is tracked perfectly rather than being something
 * to reject or follow.
 *
 * NEGATIVE OFFSETS come from section 3.2's periodicity argument rather
 * than from any trickery: within a modeline every line is identical, so
 * a pulse wanted 5us EARLIER is emitted 5us before the NEXT restart --
 * a delay of one period minus five microseconds. The counter is already
 * counting from the previous edge, so this costs nothing.
 *
 * PER is fixed at its maximum rather than tracking the line. In
 * single-slope PWM the outputs are set at BOTTOM and cleared at their
 * compare, so BOTTOM must not arrive on its own: with PER at 0xFFF8 the
 * counter would need 410us to wrap, and a restart always arrives first.
 * The only BOTTOM is the one Hsync causes. Low three bits are zero, as
 * 8x mode requires.
 */
/* ===================================================================
 * Averaged line period -- for the NEGATIVE side of the offset only
 * ===================================================================
 *
 * A positive offset is a delay from an edge that has already arrived,
 * so its placement needs no knowledge of the period at all beyond
 * scaling the step size. A negative offset is a PREDICTION: the pulse
 * is placed a whole line minus the offset after the last edge, which
 * only lands where intended if the next edge arrives where the period
 * says it will.
 *
 * Until now that prediction used a period captured once, when the rate
 * last changed, and never revisited. A source that drifts -- warming
 * up, or simply not on its nominal frequency -- walked away from it
 * with nothing to pull it back.
 *
 * BE CLEAR ABOUT WHAT THIS FIXES. It tracks DRIFT, which is slow and
 * predictable. It does nothing whatever for the line-to-line jitter
 * that makes the negative side shimmer, because that is by definition
 * unpredictable and no average can anticipate it. This is a
 * correctness improvement, not the fix.
 *
 * Held in sixteenths of a count so the average has resolution the
 * measurement lacks: individual captures are whole counts, but their
 * mean is not, and a sixteenth of a count is 3.1ns.
 */
#define PERIOD_AVG_SHIFT  6      /* ~64-line time constant */


static uint16_t period_avg16;    /* counts << 4 */
static int16_t  period_acc;      /* remainder the divide would discard */
static uint8_t  period_avg_ok;

/* Divide by a power of two, truncating TOWARD ZERO. C's >> rounds a
 * negative value toward minus infinity, and that asymmetry is a
 * one-sided force on an average, not a rounding detail: every negative
 * correction would be rounded away from zero and every positive one
 * toward it, so the average would creep downward on noise alone. */
static int16_t sdiv2(int16_t v, uint8_t shift)
{
    return (v >= 0) ? (int16_t)(v >> shift) : (int16_t)-((-v) >> shift);
}

static void period_average(uint16_t period)
{
    if (period < LINE_PER_MIN || period > LINE_PER_MAX) return;

    int16_t meas = (int16_t)(period << 4);      /* 1400<<4 fits int16 */

    if (!period_avg_ok) {
        period_avg16  = (uint16_t)meas;
        period_acc    = 0;
        period_avg_ok = 1;
        return;
    }

    /* A capture that arrives a line late reports roughly twice the
     * period, and fed to an average that is a step of hundreds of
     * counts. Anything more than an eighth off is not a line of this
     * modeline. */
    int16_t diff = (int16_t)(meas - (int16_t)period_avg16);
    int16_t tol  = (int16_t)(period_avg16 >> 3);
    if (diff > tol || diff < (int16_t)-tol) return;

    /* The remainder is CARRIED, not discarded, and that is the whole
     * reason this average can resolve anything finer than a count.
     *
     * A plain shifted average has a dead band: individual captures are
     * whole counts, so the difference from the average is always a
     * multiple of sixteen in these units, and >>6 of anything under 64
     * is zero. Every correction rounds away and the average sits
     * forever on whichever whole count it started at -- which is
     * exactly what it did when first written, reading 1271.000 against
     * a true 1271.11 and never moving.
     *
     * Accumulating the differences and spending them only when they add
     * up to a whole unit loses nothing in either direction, so a long
     * run of measurements that are mostly N and occasionally N+1
     * converges on the fraction between them. */
    period_acc = (int16_t)(period_acc + diff);
    int16_t adj = sdiv2(period_acc, PERIOD_AVG_SHIFT);
    period_acc = (int16_t)(period_acc - (int16_t)(adj << PERIOD_AVG_SHIFT));
    period_avg16 = (uint16_t)((int16_t)period_avg16 + adj);
}

/* The period in HI-RES units, carrying the average's fractional part
 * rather than rounding it away.
 *
 * This is the point of averaging at all. Rounding to whole counts threw
 * away exactly the resolution the average was computed to have, and the
 * negative side is placed a whole period out, so every rounding step
 * moved the picture 50ns -- 100ns once hysteresis was added to stop it
 * flickering across a boundary. Kept as a fraction, successive updates
 * move the placement by a hi-res unit at a time: 6.25ns, below the
 * quantisation floor and invisible.
 *
 * period_avg16 counts sixteenths; hi-res units are eighths; so a single
 * shift converts. */
static uint16_t period_best_hires(void)
{
    if (!period_avg_ok) return (uint16_t)(last_period_counts << 3);
    return (uint16_t)(period_avg16 >> 1);
}


static void hoffset_apply(uint16_t per_hires)
{
    if (per_hires < (LINE_PER_MIN << 3)
     || per_hires > (LINE_PER_MAX << 3)) return;

    int16_t  step      = (int16_t)(per_hires >> HOFF_STEP_SHIFT);
    int16_t  off       = (int16_t)(((int16_t)h_offset - H_OFFSET_DEFAULT
                                    ) * step);

    /* Section 6.4: the emitted width follows the source's. A sixteenth
     * of the line until one has been measured -- close to the analogue
     * standard at every rate this serves. */
    uint16_t w = last_width_counts ? (uint16_t)(last_width_counts << 3)
                                   : (uint16_t)(per_hires >> 4);
    if (w < 8u * 20u)  w = 8u * 20u;
    if (w > 8u * 200u) w = 8u * 200u;

    /* REVERTED: the restart edge no longer follows the offset's sign.
     *
     * Making it do so removed a real defect -- the negative side emitted
     * 2.0us near centre where the source sends 4.7us -- and it did not
     * measurably improve the shimmer it was aimed at. What it did do was
     * couple the emitted POSITION to the measured pulse WIDTH, because a
     * trailing-edge restart places at off minus the width. The width is
     * measured per line and quantised to whole counts, so it moves by
     * one count now and then, and every time it did the picture ticked
     * sideways by 50ns.
     *
     * That is a fix which solved one failure mode and exposed another,
     * so it goes rather than being layered over. The restart stays on
     * the leading edge, the offset is a plain delay, and the position
     * depends on nothing that is measured per line. */
    int16_t d_raw = off;

    /* A NEGATIVE d_raw becomes a delay of nearly a whole line. That is
     * what forces the pulse onto the PREVIOUS line, where section 3.2's
     * periodicity argument is the only thing that makes it expressible
     * at all. Moving the neutral into the delay range would have
     * removed this branch entirely; it was built, measured and rejected
     * because a neutral that is a delay moves the picture by default.
     * See the note above H_OFFSET_DEFAULT. */
    uint16_t d = (d_raw >= 0) ? (uint16_t)d_raw
                              : (uint16_t)((int16_t)per_hires + d_raw);

    /* CMP0 at BOTTOM is degenerate: the output would be set and cleared
     * in the same tick. */
    if (d < 8u) d = 8u;

    /* If the pulse would still be high at the next restart it is simply
     * cut short there, and that is ACCEPTABLE -- the leading edge is
     * what positions the picture, and a separator only needs enough
     * width to trigger on. What is not acceptable is moving the pulse
     * to make it fit, which is what the previous clamp did: it pinned
     * the first four negative increments to one position and they did
     * nothing at all. Clamp on the width that survives, not on the
     * position. */
    if (d > (uint16_t)(per_hires - HOFF_MIN_WIDTH))
        d = (uint16_t)(per_hires - HOFF_MIN_WIDTH);

    TCE0.CMP0BUF = d;
    TCE0.CMP1BUF = (uint16_t)(d + w);
}

static void hoffset_start(uint16_t period)
{
    /* KEEP TCE0's WAVEFORM OUTPUTS OFF PORTA. This must be set before
     * CMP0EN/CMP1EN are enabled below, and it is not optional.
     *
     * PORTMUX.TCEROUTEA resets to 0x00 = PORTA, which puts WO0 on PA0
     * and WO1 on PA1 -- the DDC pins. Enabling the compare outputs then
     * makes TCE0 drive SDA and SCL directly, and since TCE0 only starts
     * when Hsync arrives, the symptom was that EDID worked perfectly
     * until a source was connected and never again until a power cycle:
     * the timer keeps driving the bus once started, and only a reset
     * re-runs twi_client_init.
     *
     * PORTD routing maps WO0 and WO1 to NO PIN AT ALL, which is exactly
     * what is wanted -- the CCL taps them internally, so the
     * reconstruction is unaffected while the bus is left alone. WO4-WO7
     * would land on PD4-PD7, and those stay quiet because only CMP0 and
     * CMP1 are enabled. */
    PORTMUX.TCEROUTEA = PORTMUX_TCE0_PORTD_gc;

    TCE0.CTRLA  = 0;
    /* 8x hi-res on the compare outputs. Fed by CLK_PER4 at 80MHz, which
     * is the only reason clock_init sets the PLL up as it does. */
    TCE0.CTRLD  = TCE_HREN_8X_gc;
    TCE0.CTRLB  = TCE_WGMODE_SINGLESLOPE_gc | TCE_CMP0EN_bm | TCE_CMP1EN_bm;
    TCE0.PER    = 0xFFF8;
    TCE0.CNT    = 0;
    TCE0.EVCTRL = TCE_EVACTB_RESTART_POSEDGE_gc | TCE_CNTBEI_bm;
    period_avg_ok = 0;
    period_average(period);
    hoffset_apply(period_best_hires());
    /* Buffered writes only transfer at BOTTOM, and the first BOTTOM is
     * the first restart -- so seed the live registers too, or the first
     * line emits whatever was there before. */
    TCE0.CMP0   = TCE0.CMP0BUF;
    TCE0.CMP1   = TCE0.CMP1BUF;
    TCE0.CTRLA  = TCE_CLKSEL_DIV1_gc | TCE_ENABLE_bm;
    hoffset_running = 1;
}

static void hoffset_stop(void)
{
    TCE0.CTRLA  = 0;
    TCE0.EVCTRL = 0;
    hoffset_running = 0;
}

/* Defined below with the other hardware setup, but needed here. */

/* The role the current mode and the current source together call for.
 *
 * Only Mode S on a fast source divides. No hysteresis and no debounce:
 * the two windows are 13kHz apart and a source cannot be in both, so
 * there is no boundary to chatter across. A source that genuinely moves
 * between them has changed modeline, which drops sync, which mutes and
 * re-measures anyway. An earlier ATtiny824 revision added hysteresis
 * and then a longer-timescale block on top of it to damp a different
 * boundary; each layer fixed its target and exposed a new failure at
 * another timescale, and the whole thing was reverted. */
static ccl_role_t role_wanted(uint8_t windows)
{
    /* Mode S divides a fast source -- but only Mode S ITSELF does.
     *
     * Widening the acceptance window for the adjustment modes was not
     * enough on its own: the role is chosen separately, so a 31kHz
     * source still went to the divider inside Mode H and the offset had
     * nothing to act on. And a 25kHz source, now inside the window,
     * would have been passed through by a role picked for Mode S rather
     * than for the mode being adjusted.
     *
     * Inside an adjustment mode the behaviour must be Mode 5's --
     * everything passed straight through -- so the picture being
     * adjusted is the picture the source sent. */
    if (ui_state == UI_NORMAL && current_mode == MODE_S && (windows & W_FAST))
        return CCL_ROLE_DIVIDE;

    /* The reconstruction is used only when it has something to do and
     * something to do it with. At the neutral offset there is nothing
     * to correct, so the pin is better. */
    /* There is no lock to wait for: the restart is hardware, so the
     * counter is either being restarted by Hsync or it is not.
     *
     * The neutral is zero delay, so the reconstruction is engaged only
     * off centre and a centred unit passes the pin straight through,
     * adding nothing at all. That is only true while the neutral stays
     * at zero -- a neutral inside the delay range would have to stay
     * engaged at centre too, or dropping to the pin there would put a
     * step of that size in the middle of the adjustment. */
    if (h_offset != H_OFFSET_DEFAULT && hoffset_running)
        return CCL_ROLE_RECON;

    return CCL_ROLE_PASSTHRU;
}

/* Swaps LUT2's job. Muted across, and the polarity is re-measured
 * afterwards because the H input's idle level changes with the role --
 * see polarity_measure(). */
static void apply_role(ccl_role_t want, uint16_t now)
{
    if (want == ccl_current_role()) return;

    settle_begin(now);

    ccl_configure(want, pol_flip, h_pin_idle_high, mode_valid,
                  cs_hmode());                       /* installs the role */

    /* TCB1's job follows the role rather than being set here. Entering
     * the divide role takes the timer from Mode FO if it had it -- which
     * cannot actually happen, since dividing needs a fast source and
     * Mode FO an interlaced one, but the arbitration does not depend on
     * that being true. Leaving the role hands it back to the width
     * measurement, and fo_update() takes it from there on the next pass
     * if Mode FO still wants it. */
    if (fo_active && want == CCL_ROLE_DIVIDE) {
        fo_active = 0;
        v_route_apply();
        ccl_fo_lut0_build(0, fo_trigger_falling());
    }
    tcb1_reevaluate();

    pol_pending = 1;
}

static void change_mode(uint8_t mode, uint16_t now)
{
    current_mode = mode;
    persist_byte(EE_MODE, mode);
    edid_select(mode);
    settle_begin(now);                /* mute across the change */

    /* Leaving Mode S while it was dividing must undo the divider now
     * rather than waiting for the next capture, or the new mode spends
     * a frame combining Vsync with a stale reshaped pulse. Entering
     * Mode S goes the other way and is left to the next capture, since
     * that is the first point at which the source's rate is known. */
    if (mode != MODE_S) apply_role(CCL_ROLE_PASSTHRU, now);
}


/* ===================================================================
 * B1 -- press and release dispatch (section 8.1, 8.1a, 8.1b)
 * ===================================================================
 *
 * Everything is decided on RELEASE, from how long the button was down.
 * The preview during the hold says what releasing now will do; the
 * report plays after. One button is sufficient only because the
 * timeline is the same shape in every state -- indications, then fast
 * blinks, then dark as the last warning before something destructive.
 */
static uint16_t b1_down;
static uint8_t  b1_pressed;

/* THE CANDIDATE. A disagreeing level that has not yet held long enough
 * to be believed. b1_cand says one is outstanding; b1_cand_at is when
 * it started. Cleared by any agreeing sample, which is what makes the
 * qualify test CONTINUOUS rather than cumulative -- and continuous is
 * the whole of the noise rejection. See update_b1(). */
static uint8_t  b1_cand;
static uint16_t b1_cand_at;

/* When B1 last actually CHANGED, for the sync report's "any button edge
 * restarts the cycle".
 *
 * IT USED TO BE DISTINCT FROM b1_change AND NO LONGER IS. The old
 * debouncer refreshed b1_change on every pass the level was stable, so
 * it read as "now" almost always and was useless as "time since the
 * last edge" -- hence a second variable. The rewrite writes b1_change
 * only on accepted edges, so the two now carry the same value and this
 * one is redundant.
 *
 * KEPT ANYWAY, deliberately, and this is a decision rather than an
 * oversight. Removing it would fold the sync report's timing onto the
 * debounce's lockout timestamp, which is two meanings on one variable
 * for the sake of two bytes -- the shape of fault this project keeps
 * finding. The two carrying equal values today is a property of how the
 * debouncer currently works, not a guarantee about what the sync report
 * needs. */
static uint16_t b1_edge_at;

/* ===================================================================
 * UI state (sections 8.2, 8.3, 8.6)
 * =================================================================== */
/* MS deferred single press: until the 500ms window closes a single press
 * is indistinguishable from the first half of a double press, so it
 * cannot be acted on yet. */
static uint8_t  ms_press_pending;
static uint16_t ms_press_at;

/* Section 8.3: the step direction persists until reversed, and reversing
 * is a hold rather than a separate control. */
static int8_t   adj_dir = 1;
static uint8_t  adj_entry;          /* value on entry, for discard */

/* Mode CS's session state. Separate from adj_dir and adj_entry rather
 * than sharing them, because the two sessions can never be open at once
 * but the DISCARD can: the idle timeout reads the entry value from
 * whichever mode is live, and a shared byte would work perfectly until
 * someone allowed a mode to be entered from another. Two bytes. */
/* cs_dir IS GONE. Mode CS stepped by +/-1 with a direction flag the
 * 2-4s hold flipped; that window is now inert (see update of the CS
 * branch in b1_release), so the direction could only ever be +1 and a
 * variable that can hold one value is a false signal -- the next reader
 * of `cs_method + cs_dir` would believe direction was something that
 * varied. Restoring reverse means restoring this, the flip in the 2-4s
 * branch, the reset on entry, and the underflow clamp on the step. */
static uint8_t  cs_entry = CS_METHOD_DEFAULT;


static uint16_t adj_last_input;     /* for the 20s idle timeout */
static uint16_t adj_strobe_at;      /* when the strobe started */
static uint8_t  cannot_help_on;     /* the lockout is currently active */
static uint16_t cannot_help_at;     /* last refusal, for the repeat gap */
static uint8_t  cannot_help_playing; /* the urgent player has the refusal */
static uint8_t  adj_strobing;

/* ===================================================================
 * The lockout: a CONDITION, not an event
 * ===================================================================
 *
 * Mode V regenerates Vsync on the line grid, which destroys the half
 * line that makes interlace. Mode FO moves one field of two, and a
 * progressive source has only one. Each is useless on the source the
 * other one needs, and neither can be made to work there by trying
 * harder.
 *
 * So this is asked FRESH EVERY PASS rather than latched when a button is
 * pressed. Three things follow from that, and all three are the point:
 *
 *   The refusal announces itself. A user who enters Mode FO on a
 *   progressive source is told immediately, rather than pressing a
 *   button to find out. "A working action with no indication reads as
 *   broken" has bitten this project twice; an action that CANNOT work
 *   and says nothing is worse.
 *
 *   It clears itself. Change the source and the mode simply starts
 *   working -- nothing has to be acknowledged, and there is no latched
 *   flag to be left set by a path that forgot to clear it.
 *
 *   It cannot go stale. A latched refusal outlives the reason for it.
 *
 * No Vsync at all reads as locked out for Mode FO, correctly: there are
 * no fields to offset. */
static uint8_t adj_locked_out(void)
{
    /* Mode V NO LONGER REFUSES INTERLACED SOURCES OUTRIGHT. The
     * regeneration is frame-referenced now and hands the half line to
     * Mode FO's one-shot, so an interlaced source is served properly
     * rather than flattened to doublestrike.
     *
     * What it still refuses is an interlaced source whose FRAME LENGTH
     * has not been established -- the second pulse is placed at half a
     * frame, so without a believed length there is nowhere to put it and
     * the mode would sit there doing nothing. Saying so is the whole
     * point of the pattern. */
    if (ui_state == UI_ADJ_V)
        return (uint8_t)(src_interlaced && !v_frame_lines);
    if (ui_state == UI_ADJ_FO)
        return (uint8_t)(!src_interlaced);

    /* Mode ED edits the EDID being SERVED. With the DDC client switched
     * off for this profile there is nothing being served, so there is
     * nothing to edit -- and the same argument as Mode FO's applies:
     * refusing the gesture would say "this mode does not exist", where
     * refusing from inside says "this mode cannot help with the way
     * this profile is set up", which is the true statement. It also
     * clears itself the instant EDID is switched back on. */
    if (ui_state == UI_ADJ_ED)
        return (uint8_t)(!edid_enabled);
    return 0;
}

/* Section 8.2 EDID toggle: ten pulses ramping between 50ms and 400ms
 * period. Accelerating means enabled, decelerating means disabled -- the
 * direction carries the meaning, so it reads the same whichever state
 * you started from. */
static void play_edid_ramp(uint8_t enabled, uint16_t now)
{
    int16_t buf[BLINK_MAX];
    uint8_t k = 0;
    for (uint8_t i = 0; i < 10 && k < BLINK_MAX - 2; i++) {
        uint8_t step = enabled ? (uint8_t)(9 - i) : i;   /* fast<-slow */
        /* 20% faster than the 50-400ms of section 8.2: 40-320ms. The
         * ramp reads better shorter -- the direction is the message and
         * ten slow pulses outlast the reader's attention. */
        int16_t half = (int16_t)((40 + (uint16_t)step * 31) / 2);
        buf[k++] = half;
        buf[k++] = (int16_t)-half;
    }
    buf[k] = 0;
    blink_play(buf, (uint8_t)(k + 1), now);
}

/* Section 8.2 sync report: 250ms pulses 300ms apart -- one = no Vsync,
 * two = no Hsync, three = neither, dark when both are present. */
static void play_sync_report(uint16_t now)
{
    uint8_t n = 0;
    if (!hsync_valid) n = 2;
    if (!vsync_present) n = n ? 3 : 1;
    if (!n) return;

    int16_t buf[BLINK_MAX];
    uint8_t k = 0;
    while (n--) { buf[k++] = 250; buf[k++] = -300; }
    buf[k] = 0;
    blink_play(buf, (uint8_t)(k + 1), now);
}

static void factory_reset(uint16_t now)
{
    /* Section 8.1b: the same defaults a freshly programmed part boots
     * with -- and that includes LOCKED, so a reset locks a unit that
     * was unlocked. */
    current_mode    = MODE_1;
    current_profile = PROFILE_DEFAULT;
    b1_locked       = 1;
    h_offset        = H_OFFSET_DEFAULT;
    v_offset        = V_OFFSET_DEFAULT;
    f_offset        = F_OFFSET_DEFAULT;
    ed_pos          = 0;
    ed_size         = 0;

    persist_byte(EE_MODE,      current_mode);
    persist_byte(EE_PROFILE,   current_profile);
    persist_byte(EE_B1_LOCKED, 1);
    for (uint8_t p = 0; p < 3; p++) {
        persist_byte((uint8_t)(EE_EDID_P1 + p), 0);      /* EDID off */
        persist_byte((uint8_t)(EE_HOFF_P1 + p), H_OFFSET_DEFAULT);
        persist_byte((uint8_t)(EE_VOFF_P1 + p), V_OFFSET_DEFAULT);
        persist_byte((uint8_t)(EE_FOFF_P1 + p), F_OFFSET_DEFAULT);
        persist_byte((uint8_t)(EE_EDPOS_P1  + p), (uint8_t)ED_EE_BIAS);
        persist_byte((uint8_t)(EE_EDSIZE_P1 + p), (uint8_t)ED_EE_BIAS);
        persist_byte((uint8_t)(EE_CSMETH_P1 + p), CS_METHOD_DEFAULT);
    }

    /* APPLY the defaults, do not merely store them. Writing EEPROM and
     * leaving the live state alone is the same fault that made the EDID
     * toggle appear to work while changing nothing, and that profile
     * switching had with its EDID byte. This is the third instance: a
     * reset came back with EDID still on, because only the stored value
     * had changed and it would not be read until the next power-up.
     *
     * Anything persisted here has to be pushed into the running state
     * in the same breath. */
    edid_enabled = 0;
    edid_select(current_mode);          /* mode went back to MODE_1 */
    apply_edid_enable();
    hoffset_apply(period_best_hires()); /* offset went back to centre */
    cs_set(CS_METHOD_DEFAULT, now);     /* and the combiner back to Csync */

    /* The LED is dark at the moment of release, so without this a reset
     * would give no confirmation at all. */
    report_mode_profile(now);
}

/* An erased or out-of-range byte reads as method 1. That still matters
 * after MODE_MAGIC changes, when load_settings() discards the stored
 * layout and every profile falls back to this. */
static uint8_t cs_load(uint8_t profile)
{
    uint8_t r = eeprom_read_byte(&eeprom_data[EE_CSMETH_P1 + profile - 1]);
    return (r >= CS_METHOD_MIN && r <= CS_METHOD_MAX) ? r
                                                      : CS_METHOD_DEFAULT;
}

/* An erased or out-of-range byte reads as neutral. */
static int8_t ed_load(uint8_t base, uint8_t profile)
{
    uint8_t r = eeprom_read_byte(&eeprom_data[base + profile - 1]);
    int8_t  v = (int8_t)((int16_t)r - ED_EE_BIAS);
    return (v <= ED_STEPS_MAX && v >= -ED_STEPS_MAX) ? v : 0;
}

static void select_profile(uint8_t p, uint16_t now)
{
    if (p < 1 || p > 3 || p == current_profile) return;
    current_profile = p;
    persist_byte(EE_PROFILE, p);

    /* Release itself blinks nothing -- the preview seen before letting
     * go is the confirmation. Section 8.1.
     *
     * EVERY per-profile setting has to be reloaded here, not just the
     * offset. The EDID enable is one of them, and leaving it behind
     * meant a profile stored with DDC off resumed with whatever the
     * previous profile had -- the stored value only reappearing after a
     * power cycle, which is the same fault the toggle had. */
    uint8_t ho = eeprom_read_byte(&eeprom_data[EE_HOFF_P1 + p - 1]);
    h_offset = (ho < H_OFFSET_STEPS) ? ho : H_OFFSET_DEFAULT;

    uint8_t vo = eeprom_read_byte(&eeprom_data[EE_VOFF_P1 + p - 1]);
    v_offset = (vo < V_OFFSET_STEPS) ? vo : V_OFFSET_DEFAULT;

    uint8_t fo = eeprom_read_byte(&eeprom_data[EE_FOFF_P1 + p - 1]);
    f_offset = (fo < F_OFFSET_STEPS) ? fo : F_OFFSET_DEFAULT;

    ed_pos  = ed_load(EE_EDPOS_P1,  p);
    ed_size = ed_load(EE_EDSIZE_P1, p);

    /* THROUGH cs_set(), not assigned. Section 19.2, for the fourth
     * time: the stored value has to reach the running output in the
     * same breath, and for this setting the running output is a truth
     * table. Assigning cs_method here would leave pin 12 combining
     * while the profile said pass, until the next thing that happened
     * to raise a settle -- which on a stable source is nothing at all,
     * so it would present as a profile switch that changed the sync
     * structure only after a mode change or a cable pull. */
    cs_set(cs_load(p), now);

    edid_enabled = (eeprom_read_byte(&eeprom_data[EE_EDID_P1 + p - 1]) == 1);

    /* The EDID has to be REBUILT, not merely re-enabled: the new
     * profile's ED offsets are different numbers and the shadow is
     * still holding the old profile's geometry. Section 19.2 again --
     * this is the same shape as the three faults listed there, and it
     * would have presented as a profile switch that changed the picture
     * only after a mode change. */
    edid_select(current_mode);
    apply_edid_enable();
}

static void b1_release(uint16_t held, uint16_t now)
{
    if (b1_locked) {
        if (held >= HOLD_L_FACTORY_MS) {
            factory_reset(now);
        } else if (held >= HOLD_L_UNLOCK_MS) {
            b1_locked = 0;
            persist_byte(EE_B1_LOCKED, 0);

            /* Never emerge from lockout INTO Mode S. Mode S is where a
             * unit is configured, and the lock exists to stop a
             * configured unit being disturbed -- so unlocking straight
             * into the configuration mode is backwards, and it puts the
             * user one stray press from changing something they were
             * protecting. M1 is the safe place to arrive. */
            if (current_mode == MODE_S) change_mode(MODE_1, now);

            report_mode_profile(now);    /* confirm the transition */
        } else {
            /* Report ONLY. No mode change, no profile change, no MS.
             * A locked dongle is a working dongle that cannot be
             * misconfigured -- but it still answers. */
            report_mode_profile(now);
        }
        return;
    }

    /* --- Mode ED (section 17.1) ----------------------------------
     *
     * FIRST, because adj_cur() has no row for it -- it would hand back
     * the zero row and the generic branch would dereference a null
     * value pointer. */
    if (ui_state == UI_ADJ_ED) {
        adj_last_input = now;

        if (adj_strobing) {
            if (elapsed_since(now, adj_strobe_at) < ADJ_STROBE_MS) return;
            adj_strobing = 0;
        }

        if (held >= ED_AXIS_MS) {
            /* Switch axis, and SAY WHICH ONE. The rate blink cannot:
             * it reports distance from centre, and both axes are at
             * centre when you arrive. Without this, a user who looked
             * away has no way to tell whether the next press moves the
             * picture or resizes it. One pulse position, two size. */
            ed_axis ^= 1;
            int16_t buf[6];
            uint8_t k = pat_pulses(buf, (uint8_t)(ed_axis ? 2 : 1), 250, 250);
            buf[k] = 0;
            blink_play(buf, (uint8_t)(k + 1), now);
        } else if (held >= ED_RESET_MS) {
            /* Resets the ACTIVE AXIS and stays in the mode. It does not
             * exit, unlike H/V/FO's 6s hold -- with two axes to set,
             * being thrown out for zeroing one of them would mean
             * re-entering through a 13s hold to reach the other. */
            *ed_val() = 0;

            /* ZEROING ONE AXIS CAN INVALIDATE THE OTHER. Size is what
             * frees the blanking position spends, so resetting size
             * while position is out near its limit leaves a pair that
             * no longer renders -- and ed_commit() would then fall back
             * to the previous geometry while these variables said
             * something else. Live state and served image disagreeing
             * is the section 19.2 fault in a new place.
             *
             * Walk the OTHER axis in until the pair fits. It always
             * terminates: (0,0) renders on every mode by construction. */
            while (!ed_commit(ed_pos, ed_size)) {
                int8_t *o = ed_axis ? &ed_pos : &ed_size;
                if      (*o > 0) (*o)--;
                else if (*o < 0) (*o)++;
                else             break;
            }
            blink_play(pat_strobe, 2, now);
            adj_strobe_at = now; adj_strobing = 1;
        } else if (held >= ED_SAVE_MS) {
            /* BOTH axes, not just the active one. They are set in one
             * sitting and describe one geometry; committing half of it
             * would leave a picture nobody chose. */
            persist_byte((uint8_t)(EE_EDPOS_P1  + current_profile - 1),
                         (uint8_t)(ed_pos  + ED_EE_BIAS));
            persist_byte((uint8_t)(EE_EDSIZE_P1 + current_profile - 1),
                         (uint8_t)(ed_size + ED_EE_BIAS));
            ui_state = UI_NORMAL;
            report_mode_profile(now);
        } else if (held >= ED_REVERSE_MS) {
            ed_dir[ed_axis] = (int8_t)-ed_dir[ed_axis];
        } else if (adj_locked_out()) {
            /* Swallowed -- EDID is off for this profile, so there is
             * nothing being served to edit. See adj_locked_out(). */
        } else {
            /* One step, and whether it lands is decided by the DTDs
             * rather than by a step count. ed_commit() renders the
             * candidate and says whether every timing in the current
             * EDID could take it; a refusal IS the end of travel, so it
             * strobes and ping-pongs exactly as a fixed range would. */
            int8_t *v = ed_val();
            int8_t  n = (int8_t)(*v + ed_dir[ed_axis]);
            int8_t  cp = (ed_axis ? ed_pos  : n);
            int8_t  cz = (ed_axis ? n       : ed_size);

            uint8_t fits = (n <= ED_STEPS_MAX && n >= -ED_STEPS_MAX)
                        && ed_commit(cp, cz);

            if (!fits) {
                ed_dir[ed_axis] = (int8_t)-ed_dir[ed_axis];
                n  = (int8_t)(*v + ed_dir[ed_axis]);
                cp = (ed_axis ? ed_pos : n);
                cz = (ed_axis ? n      : ed_size);
                if (n <= ED_STEPS_MAX && n >= -ED_STEPS_MAX
                 && ed_commit(cp, cz)) {
                    *v = n;
                }
                blink_play(pat_strobe, 2, now);
                adj_strobe_at = now; adj_strobing = 1;
            } else {
                *v = n;
                if (n == 0) {             /* neutral is a stop point too */
                    blink_play(pat_strobe, 2, now);
                    adj_strobe_at = now; adj_strobing = 1;
                }
            }
        }
        return;
    }

    /* --- Mode CS (section 17.1) ----------------------------------
     *
     * SECOND, and for the same reason Mode ED is first: adj_cur() has
     * no row for it and would hand back the zero row, whose value
     * pointer is null. See the note on the ui_state enum.
     *
     * The hold ladder is the standard one MINUS ITS FIRST RUNG: the
     * 2-4s window is inert here, 4s saves and exits, 6s resets and
     * exits. The generic preview in hold_preview() is used UNCHANGED
     * and needs no case -- its lit 2-4s band is what makes the 4s save
     * boundary findable, and going dark at 4s is the signal. See the
     * note there for the darkened variant that was tried and rejected.
     * Only the short press differs, and it differs in two ways.
     *
     * IT WRAPS. The adjustment engine ping-pongs at the ends and
     * strobes there, because for a continuous range a wrap would leap
     * the whole travel in one press at exactly the point a real
     * discontinuity would show. Seven discrete methods have no travel
     * to leap and no discontinuity to hide: 7 to 1 is one step like any
     * other, and refusing it would mean the only way back from the end
     * of the list is a hold. Section 17.1 asks for the wrap.
     *
     * IT DOES NOT STROBE. A strobe means stop and look at a landmark,
     * and there are no landmarks here -- no neutral, no ends, seven
     * equally valid choices. The count report is the whole indication.
     * That also means no adj_strobing test at the top of this block:
     * nothing in it ever sets the flag, so a leftover strobe from a
     * different mode is the only way it could be set, and swallowing a
     * press for it would be swallowing it for something that already
     * finished. */
    if (ui_state == UI_ADJ_CS) {
        adj_last_input = now;

        if (held >= ADJ_RESET_MS) {
            cs_set(CS_METHOD_DEFAULT, now);       /* reset and exit */
            persist_byte((uint8_t)(EE_CSMETH_P1 + current_profile - 1),
                         cs_method);
            ui_state = UI_NORMAL;
            report_mode_profile(now);
        } else if (held >= ADJ_SAVE_MS) {
            persist_byte((uint8_t)(EE_CSMETH_P1 + current_profile - 1),
                         cs_method);
            ui_state = UI_NORMAL;
            report_mode_profile(now);
        } else if (held >= ADJ_REVERSE_MS) {
            /* DELIBERATELY EMPTY. The 2-4s window does nothing in Mode
             * CS.
             *
             * THE BRANCH MUST STAY. Deleting it does not make the window
             * do nothing -- it makes a 2-4s hold fall through to the
             * else below and STEP THE METHOD, which is worse than the
             * reverse it replaced: a user pausing on the way to save
             * would silently change the setting they were about to
             * store. The empty branch is what makes the window inert.
             *
             * Reverse used to live here, on the argument that reaching
             * method 7 from method 1 is six presses forward or one
             * backward. With seven wrapping methods and a count report
             * on every press, going round is quick and unambiguous,
             * whereas a hidden direction flag is not: nothing on the LED
             * says which way the next press will go, so a user who
             * reversed and then came back later would find the button
             * running backwards for no visible reason. cs_dir went with
             * it -- see the note where it used to be declared. */
        } else {
            uint8_t n = (uint8_t)(cs_method + 1);
            if (n > CS_METHOD_MAX) n = CS_METHOD_MIN;
            cs_set(n, now);
            cs_report_start(now, 0);   /* restarts, never swallowed */
        }
        return;
    }

    /* --- Adjustment mode (section 8.3), any of the three ---------- */
    if (ui_state != UI_NORMAL) {
        const adj_def_t *a = adj_cur();
        uint8_t middle = (uint8_t)(a->steps / 2);
        adj_last_input = now;

        /* A strobe means STOP AND LOOK -- an end of travel, or the
         * middle. Presses during it are discarded, so it always runs its
         * full 1200ms and the setting cannot be walked past the landmark
         * by someone still pressing. */
        if (adj_strobing) {
            if (elapsed_since(now, adj_strobe_at) < ADJ_STROBE_MS) return;
            adj_strobing = 0;
        }

        if (held >= ADJ_RESET_MS) {
            *a->value = middle;                   /* reset and exit */
            persist_byte((uint8_t)(a->ee_base + current_profile - 1),
                         *a->value);
            ui_state = UI_NORMAL;
            report_mode_profile(now);
        } else if (held >= ADJ_SAVE_MS) {
            persist_byte((uint8_t)(a->ee_base + current_profile - 1),
                         *a->value);
            ui_state = UI_NORMAL;
            report_mode_profile(now);
        } else if (held >= ADJ_REVERSE_MS) {
            adj_dir = (int8_t)-adj_dir;           /* reverse direction */
        } else if (adj_locked_out()) {
            /* SWALLOWED. The step is refused and the strobe is not
             * played, because there is nothing to strobe about -- the
             * setting has not moved and will not.
             *
             * adj_last_input was set at the top of this block, before
             * any of these branches, so a swallowed press still counts
             * as activity and still holds off the 20s idle discard. A
             * user pressing at a locked-out control is plainly still
             * using it, and timing them out mid-attempt would throw away
             * a setting they had already made on a source that worked.
             *
             * Only the SHORT press is refused. Reverse, save, reset and
             * the exits above all still work, so the way out of a
             * locked-out mode is the same gesture as anywhere else --
             * and save still stores whatever value was set before the
             * source changed. */
        } else {
            /* One step. The range PING-PONGS rather than wrapping: a
             * wrap would leap the whole travel in one press, exactly
             * where a real discontinuity would show. And the middle is a
             * STOP POINT -- it strobes and the next press continues in
             * the same direction, so neutral can be found without
             * counting. */
            int16_t n = (int16_t)(*a->value)
                      + (int16_t)(adj_dir * a->stride);
            if (n >= a->steps || n < 0) {
                adj_dir = (int8_t)-adj_dir;
                n = (int16_t)(*a->value) + (int16_t)(adj_dir * a->stride);
                blink_play(pat_strobe, 2, now);
                adj_strobe_at = now; adj_strobing = 1;
            } else if (n == middle) {
                blink_play(pat_strobe, 2, now);
                adj_strobe_at = now; adj_strobing = 1;
            }
            *a->value = (uint8_t)n;
        }
        return;
    }

    if (current_mode == MODE_S) {
        /* Mode S has its own button table (section 8.2) and none of the
         * section 8.1 timeline applies inside it. In particular B1
         * CANNOT BE LOCKED FROM HERE: the lock exists to stop a
         * configured dongle being disturbed, and Mode S is where
         * configuring happens. Locking from inside it would seal the
         * unit in the one mode whose whole purpose is adjustment.
         *
         * Booting into Mode S is fine -- a unit powered off from here
         * comes back here -- and the locked timeline still works in
         * Mode S, so a unit locked while in Mode S can always be
         * unlocked again.
         *
         * All of section 8.2 is here: the deferred single press, the
         * double-press EDID toggle, and the three holds into Mode H,
         * Mode V and Mode FO. */
        if (held >= HOLD_MODE_FO_MS) {
            /* Mode FO. It exists now, so the 6s hold opens it rather
             * than playing cannot-help at the gesture.
             *
             * The refusal did not disappear -- it moved to where it
             * belongs. Mode FO on a progressive source cannot act, and
             * adj_locked_out() says so continuously from inside the
             * mode. Refusing the GESTURE said "this mode does not
             * exist"; refusing from inside says "this mode cannot help
             * with the source you have", which is a different statement
             * and the true one. */
            ui_state       = UI_ADJ_FO;
            adj_entry      = *adj_cur()->value;
            adj_dir        = 1;
            adj_last_input = now;
            cannot_help_at = now;   /* one gap's grace before refusing */
            fo_blink_reset(now);
            blink_stop();
        } else if (held >= HOLD_MODE_H_MS) {
            /* 2-4s Mode H, 4-6s Mode V. Both are dark throughout --
             * they are judged by watching the picture, not the LED.
             * Mode FO is not, because both its ends look alike; see
             * fo_rate_blink(). */
            ui_state       = (held >= HOLD_MODE_V_MS) ? UI_ADJ_V : UI_ADJ_H;
            adj_entry      = *adj_cur()->value;
            adj_dir        = 1;
            adj_last_input = now;
            cannot_help_at = now;   /* Mode V can be locked out too */
            blink_stop();
        } else {
            /* Deferred: until the 500ms window closes this is
             * indistinguishable from the first half of a double press. */
            if (ms_press_pending) {
                ms_press_pending = 0;   /* second press -> EDID toggle */

                /* Toggle the LIVE state, then store it -- not the other
                 * way round. Two faults lived here:
                 *
                 * apply_edid_enable() decides from edid_enabled, and
                 * nothing was updating it. The EEPROM byte changed and
                 * the ramp played, but the TWI client was left exactly
                 * as it was, so the setting only took effect after a
                 * power cycle. The indication said it had worked; the
                 * hardware disagreed.
                 *
                 * And the new value came from re-reading EEPROM, where
                 * an untouched byte is 0xFF. That is truthy, so the
                 * first press computed "off" on a unit that was already
                 * off, and the very first toggle of a fresh part did
                 * nothing at all. The live flag has no such ambiguity --
                 * load_settings already reduced 0xFF to 0. */
                edid_enabled = edid_enabled ? 0 : 1;
                persist_byte((uint8_t)(EE_EDID_P1 + current_profile - 1),
                             edid_enabled);
                apply_edid_enable();
                play_edid_ramp(edid_enabled, now);
            } else {
                ms_press_pending = 1;
                ms_press_at      = now;
            }
        }
        return;
    }

    if (held >= HOLD_FACTORY_MS) {
        factory_reset(now);
    } else if (held >= HOLD_LOCK_MS) {
        b1_locked = 1;
        persist_byte(EE_B1_LOCKED, 1);
        report_mode_profile(now);        /* confirm the transition */
    } else if (held >= HOLD_ED_ENTER_MS) {
        /* Mode ED. Entered from a NUMBERED mode, and current_mode is
         * never touched while inside -- which is what makes "return to
         * whichever mode it was selected from" free rather than a piece
         * of remembered state. It also means ED's exits report mode AND
         * profile, unlike H/V/FO, which always exit into Mode S.
         *
         * Entered even when EDID is switched off for this profile.
         * adj_locked_out() refuses from inside instead, for the reason
         * spelled out there -- and it clears itself if EDID is turned
         * back on rather than needing the mode re-entered. */
        ui_state       = UI_ADJ_ED;
        ed_axis        = 0;                    /* position first       */
        ed_dir[0]      = 1;                    /* ...moving LEFT       */
        ed_dir[1]      = 1;                    /* ...size NARROWER     */
        ed_entry_pos   = ed_pos;
        ed_entry_size  = ed_size;
        adj_last_input = now;
        cannot_help_at = now;                  /* one gap's grace      */
        fo_blink_reset(now);
        blink_stop();
    } else if (held >= HOLD_CS_ENTER_MS) {
        /* Mode CS. Entered from a NUMBERED mode and current_mode is
         * never touched inside, exactly as Mode ED is -- which is what
         * makes "return to whichever mode it was selected from" free
         * rather than remembered state, and what makes its exits report
         * mode AND profile.
         *
         * No lockout. Method 1 works on any source and the other six
         * do something definite on any source, so there is nothing for
         * adj_locked_out() to refuse and it is not given a case. That is
         * a decision, not an omission: every other adjustment mode has
         * a source it cannot help, and this one does not.
         *
         * The report plays IMMEDIATELY on entry rather than waiting for
         * a press, because the method is invisible until the picture is
         * looked at and the user needs to know where they are starting
         * from before they start stepping. */
        ui_state       = UI_ADJ_CS;
        cs_entry       = cs_method;
        adj_last_input = now;
        cs_report_start(now, CS_ENTRY_LEAD_MS);
    } else if (held >= HOLD_MS_ENTER_MS) {
        change_mode(MODE_S, now);
        report_mode_profile(now);
    } else if (held >= HOLD_PROFILE3_MS) {
        select_profile(3, now);
    } else if (held >= HOLD_PROFILE2_MS) {
        select_profile(2, now);
    } else if (held >= HOLD_PROFILE1_MS) {
        select_profile(1, now);
    } else {
        /* M1..M5 and back to M1. Mode S is NOT in the short-press
         * cycle -- it is reached only by holding into its window, so a
         * run of presses can never land in it by accident. */
        uint8_t next = (current_mode >= MODE_5) ? MODE_1
                                               : (uint8_t)(current_mode + 1);
        change_mode(next, now);
        report_mode_profile(now);
    }
}

/* QUALIFY THE LEVEL, THEN ACCEPT, THEN LOCK OUT.
 * ------------------------------------------------------------------
 * Carry this note into any migration document. Three different faults
 * came out of this one function and the third was caused by the fix for
 * the first two, so the history matters more than the code does.
 *
 * THE ORIGINAL STRUCTURE. An edge was accepted only after the pin had
 * disagreed with b1_state CONTINUOUSLY for one window, and an `else`
 * branch restarted that clock on any agreeing sample. One constant,
 * pulled in two directions:
 *
 *   MERGE, at 30ms. A re-press inside the window swallowed the RELEASE.
 *   Release at t=0, press again at t=20: at t=20 the pin reads closed,
 *   which AGREES with a b1_state still saying "pressed", so the pending
 *   release was discarded. b1_down still pointed at the FIRST press, so
 *   `held` ran across both taps. Rapid tapping read as one long hold and
 *   past 2000ms produced a profile change instead of the presses made.
 *
 *   SPLIT, at 8ms. Break bounce holding the release level for more than
 *   8ms was accepted as a real release, and the settling contact after
 *   it as a fresh press. One push, counted twice.
 *
 * THE FIRST ATTEMPT AT A FIX made it worse. Accepting on the FIRST
 * disagreeing sample cured both -- and let electrical noise straight in,
 * because THIS LOOP RUNS MANY TIMES PER MILLISECOND. The original
 * window was never "8ms of debounce"; it was 8ms of continuous level
 * across thousands of consecutive samples, which is very strong
 * rejection of anything short. Trusting one sample threw all of that
 * away and the bench filled up with presses nobody made. Sampling rate
 * was the variable that mattered and it is not visible from this
 * function -- see the comment at the head of the main loop.
 *
 * WHAT IT DOES NOW. Two constants, one for each job, and neither can
 * reintroduce the other's failure:
 *
 *   1. LOCKOUT. If an edge was accepted less than B1_LOCKOUT_MS ago,
 *      return immediately and drop any candidate. Contact settling is
 *      never even looked at, which is what kills the split.
 *
 *   2. QUALIFY. A disagreeing level starts a candidate and must then
 *      hold CONTINUOUSLY for B1_QUALIFY_MS -- any agreeing sample
 *      clears it. At loop rate that is thousands of samples, giving
 *      exactly the noise rejection the original structure had, which is
 *      what kills the phantom presses.
 *
 *   3. ACCEPT, and start the lockout.
 *
 * WHY THIS IS NOT THE ORIGINAL STRUCTURE AGAIN. The original had one
 * window doing both jobs, so it was long -- comparable to the gap
 * between two real presses, which is what let it swallow one. Here the
 * qualify time is short (8ms, shorter than any human gap) and the part
 * that has to outlast bounce is the lockout, which runs AFTER the edge
 * is already believed and therefore cannot swallow anything. A second
 * press arriving inside the lockout is DELAYED, not lost.
 *
 * DO NOT COLLAPSE THESE BACK INTO ONE CONSTANT. That is the fault, and
 * it has now been made three times.
 *
 * THE HOLD LADDER DOES NOT MOVE. Both edges are accepted B1_QUALIFY_MS
 * after the real transition, so the delays cancel and `held` is
 * unbiased, exactly as under the original structure. All eight
 * thresholds keep their meaning.
 *
 * THE ONE REMAINING MERGE, and it is deliberate. If the pin returns to
 * its old level before the candidate qualifies, the candidate is
 * dropped -- so a gap shorter than B1_QUALIFY_MS between two presses is
 * still merged. That is 8ms. No finger produces an 8ms gap; a bouncing
 * contact produces nothing else. This is the residue of the original
 * fault reduced from 30ms to 8ms, not eliminated, and reducing it
 * further trades directly against noise rejection. */
static void update_b1(uint16_t now)
{
    uint8_t level = (PIN_B1_PORT.IN & PIN_B1_bm) ? 1 : 0;

    /* 1. Lockout. Tested first, so settling after an accepted edge
     *    never reaches the comparison at all. */
    if (elapsed_since(now, b1_change) < B1_LOCKOUT_MS) {
        b1_cand = 0;
        return;
    }

    /* 2. Qualify. Any agreeing sample clears the candidate, so the
     *    level must hold continuously -- this is the noise filter, and
     *    at loop rate it is thousands of samples deep. */
    if (level == b1_state) {
        b1_cand = 0;
        return;
    }
    if (!b1_cand) {
        b1_cand    = 1;
        b1_cand_at = now;
        return;
    }
    if (elapsed_since(now, b1_cand_at) < B1_QUALIFY_MS) return;

    /* 3. Accept, and start the lockout. */
    b1_cand    = 0;
    b1_change  = now;
    b1_edge_at = now;
    b1_state   = level;

    if (level == 0) {                     /* pressed */
        b1_down    = now;
        b1_pressed = 1;
        /* A strobe keeps the pin for its full 1200ms. Pressing during
         * one is acknowledged -- the press is counted and debounced --
         * but it neither moves the offset nor steals the LED, so the
         * landmark is always shown in full. */
        if (!adj_strobing) blink_stop();
    } else {                              /* released */
        b1_pressed = 0;
        /* b1_change is set ABOVE this call, not after it. b1_release()
         * can spend tens of milliseconds in EEPROM writes on a save
         * gesture, and the lockout has to date from the edge rather
         * than from whenever the write happens to finish. */
        b1_release(elapsed_since(now, b1_down), now);
    }
}

/* ===================================================================
 * Start-up
 * =================================================================== */


static void port_init(void)
{
    /* Inputs. Both syncs get pull-ups so a disconnected input reads as
     * a steady idle level rather than floating -- which matters for the
     * polarity vote as much as for the watchdog.
     *
     * PULLUPEN AND ISC LIVE IN THE SAME REGISTER, and that cost this
     * project the Mode-Valid ticking fault.
     *
     * PINnCTRL is one byte: INVEN(7), INLVL(6), PULLUPEN(3), ISC[2:0]
     * -- data sheet 18.5.12. The Vsync edge interrupt used to be armed
     * further down this function with a second PLAIN ASSIGNMENT,
     *
     *     PIN_VSYNC_PORT.PIN_VSYNC_CTRL = PORT_ISC_BOTHEDGES_gc;
     *
     * and BOTHEDGES is 0x01, so that write cleared bit 3 and took the
     * pull-up straight back off again. The line above it said the pin
     * was pulled up. It was not. With the lead off, pin 6 sat at the
     * 50nA leakage figure -- not the 32k of the pull-up -- while the
     * Blink LED switched on the pin next door, and the coupled edges
     * fed garbage intervals into field_rate_evaluate().
     *
     * Rev 1 had the identical defect and never showed it, because in
     * rev 1 pin 5 was SCL and idle. It took the rotation putting a
     * hard-switching output beside the floating input to make it
     * visible, which is why it read as a rev 2 regression.
     *
     * ONE WRITE PER PIN, COMPLETE, HERE. Not a set here and a
     * modification elsewhere: a read-modify-write further down would
     * have been correct and would have left the same trap for the next
     * person to add an attribute. Two writers on one register, again --
     * the only novelty being that the register is a pin's and not a
     * peripheral's. */
    PIN_VSYNC_PORT.PIN_VSYNC_CTRL =
        PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;   /* Vsync + edge int */
    PIN_HSYNC_PORT.PIN_HSYNC_CTRL = PORT_PULLUPEN_bm;          /* Hsync */
    PIN_B1_PORT.PIN_B1_CTRL = PORT_PULLUPEN_bm;          /* B1, active low */

    /* Outputs. Mode-valid and its LED start LOW: Csync stays muted
     * until the first capture proves the source is inside a window. */
    PIN_MODE_LED_PORT.DIRSET = PIN_MODE_LED_bm;                     /* Mode Valid LED */
    PIN_MODE_LED_PORT.OUTCLR = PIN_MODE_LED_bm;
    PIN_BLINK_PORT.DIRSET = PIN_BLINK_bm;                     /* Blink LED */
    /* Direct, and NOT via blink_set(). This runs before tcf0_init(), so
     * once blink_set() carries a PWM state machine off the tick there
     * is no timebase here for it to use. The level blink_set() tracks
     * starts at zero in BSS and this write leaves the pin at zero, so
     * the two agree without an ordering dependency. If that ever stops
     * being true, this is the line that has to move -- not the setter. */
    PIN_BLINK_PORT.OUTCLR = PIN_BLINK_bm;
    /* PIN 11 IS SPARE, AND IS DRIVEN LOW RATHER THAN LEFT FLOATING.
     * It has carried a succession of diagnostics and carries none now.
     * A floating input on a package this small is an antenna next to
     * two switching outputs, which is the same argument the Vsync
     * pull-up already cost this project one bench round to learn. An
     * output at ground is the cheapest definite state. */
    PIN_DIAG_PORT.DIRSET = PIN_DIAG_bm;
    PIN_DIAG_PORT.OUTCLR = PIN_DIAG_bm;

    /* Pin 13: the regenerated Vsync for Mode V, and Mode CS's V
     * output. Idles LOW, which is
     * the inactive level for the positive-going pulse the CCL expects
     * -- the combiner's XNOR/XOR choice handles the source's own
     * polarity, exactly as it does for the reconstructed Hsync. */
    PIN_VREGEN_PORT.DIRSET = PIN_VREGEN_bm;
    PIN_VREGEN_PORT.OUTCLR = PIN_VREGEN_bm;

    /* NO CCLROUTEA WRITE HERE. evsys_init() already sets LUT2 to ALT1
     * unconditionally, so the routing to PD6 exists in every build and
     * only OUTEN decides whether the pin is driven. A second write of
     * the same register from a second function is the two-owner shape
     * this project keeps paying for, even when both writers agree. */

    /* The Vsync edge interrupt is armed at the top of this function,
     * in the same write as its pull-up, because they share a register.
     * Nothing to do here. See the note there before adding a second
     * write to any PINnCTRL. */

    /* PC3 is left as an input here. The CCL takes the pin when LUT1 is
     * enabled with OUTEN set; driving it from PORT as well would fight
     * that. */
}



/* THE INTCTRL WRITE BELOW AND THE ISR ARRIVE AND DEPART TOGETHER.
 *
 * This capture ran under an interrupt while the genlock needed a phase
 * sample at a known instant. When the genlock went, the ISR went with
 * it -- but TCB0.INTCTRL was left enabling the interrupt, and an enabled
 * interrupt with no handler vectors to __bad_interrupt, which jumps to
 * address zero. The first Hsync capture therefore reset the part, and
 * every one after it, forever. Nothing downstream ever ran: no valid
 * mode, no Csync, no clue.
 *
 * The interrupt is back, permanently, because Mode V counts lines and a
 * lost capture moves the emitted Vsync. There are now THREE things that
 * belong together and must be removed together if it ever goes again:
 * this INTCTRL write, the ISR, and the CPUINT.LVL1VEC assignment in
 * main(). Leaving LVL1VEC pointing at a vector whose interrupt is
 * disabled is harmless; the other two are not. */
static void tcb0_init(void)
{
    /* Input Capture FREQUENCY measurement: the counter captures into
     * CCMP and RESTARTS on every rising edge, so there is one period
     * measurement per line and no re-arming for the CPU to do.
     *
     * This replaced FRQPW, and the reason is worth recording because
     * FRQPW looks strictly better on paper -- it gives period and pulse
     * width together from one peripheral. What it does not give is a
     * measurement every line. Its sequence is start on a rising edge,
     * capture on the falling edge, STOP on the next rising edge; and
     * from there every further edge is ignored until the CPU reads CCMP,
     * after which the next rising edge starts a fresh sequence. So the
     * edge that ends one measurement cannot begin the next, and the best
     * possible rate is one measurement every TWO lines -- worse if the
     * read is late.
     *
     * That was visible on the bench as capture interrupts missing on
     * alternate lines. It mattered to the genlock loop of increment 4a,
     * which is gone -- but it matters just as much now, because Mode V
     * counts LINES from this capture and a measurement every other line
     * is a line count that is wrong by half.
     *
     * The pulse width goes with it, and is measured by TCB1 instead,
     * from the same event, whenever the CCL is in a role that leaves
     * TCB1 free -- which is exactly when the H offset is permitted.
     * Section 6.4 wants the emitted pulse to follow the source's, and
     * hoffset_apply() is the consumer. What is NOT acceptable is the
     * ATtiny824's approach of switching one timer between the two
     * modes: that stopped the capture feeding the sync watchdog, which
     * triggered the polarity routine, which restarted the switch, and
     * blanked the output.
     *
     * The noise canceller takes four consecutive equal samples before
     * the edge detector sees a change, which is free here and worth
     * having on an input arriving down a VGA cable.
     *
     * EVCTRL.EDGE stays clear, so the capture is on positive edges and
     * CCMP is the full line period. */
    TCB0.CTRLB  = TCB_CNTMODE_FRQ_gc;
    TCB0.EVCTRL = TCB_CAPTEI_bm | TCB_FILTER_bm;
    TCB0.INTCTRL = TCB_CAPT_bm;       /* see the ISR above */
    TCB0.CTRLA  = TCB_CLKSEL_DIV1_gc | TCB_ENABLE_bm;
}


static void load_settings(void)
{
    uint8_t magic = eeprom_read_byte(&eeprom_data[EE_MAGIC]);

    if (magic != MODE_MAGIC) {
        /* Fresh part, or a layout this build does not understand.
         * These are the same defaults a factory reset restores. */
        current_mode    = MODE_1;
        current_profile = PROFILE_DEFAULT;
        edid_enabled    = 0;
        b1_locked       = 1;
        return;
    }

    uint8_t m = eeprom_read_byte(&eeprom_data[EE_MODE]);
    current_mode = (m >= MODE_1 && m <= MODE_S) ? m : MODE_1;

    uint8_t p = eeprom_read_byte(&eeprom_data[EE_PROFILE]);
    current_profile = (p >= 1 && p <= PROFILE_COUNT) ? p : PROFILE_DEFAULT;

    /* Anything that is not exactly 1 -- an erased 0xFF included --
     * reads as off, so a half-written byte can only fail safe. */
    uint8_t e = eeprom_read_byte(&eeprom_data[EE_EDID_P1
                                              + current_profile - 1]);
    edid_enabled = (e == 1) ? 1 : 0;

    uint8_t l = eeprom_read_byte(&eeprom_data[EE_B1_LOCKED]);
    b1_locked = (l == 0) ? 0 : 1;      /* default, and 0xFF, is LOCKED */

    uint8_t ho = eeprom_read_byte(&eeprom_data[EE_HOFF_P1
                                               + current_profile - 1]);
    h_offset = (ho < H_OFFSET_STEPS) ? ho : H_OFFSET_DEFAULT;

    uint8_t vo = eeprom_read_byte(&eeprom_data[EE_VOFF_P1
                                               + current_profile - 1]);
    v_offset = (vo < V_OFFSET_STEPS) ? vo : V_OFFSET_DEFAULT;

    uint8_t fo = eeprom_read_byte(&eeprom_data[EE_FOFF_P1
                                               + current_profile - 1]);
    f_offset = (fo < F_OFFSET_STEPS) ? fo : F_OFFSET_DEFAULT;

    ed_pos  = ed_load(EE_EDPOS_P1,  current_profile);
    ed_size = ed_load(EE_EDSIZE_P1, current_profile);

    /* Assigned rather than pushed through cs_set(): this runs before
     * the CCL exists, so there is no table to rewrite and no settle to
     * raise. ccl_configure() in main() reads cs_hmode() for its first
     * build, which is where the resumed method actually takes effect. */
    cs_method = cs_load(current_profile);
}

int main(void)
{
    clock_init();
    port_init();
    load_settings();

    evsys_init();
    ccl_configure(CCL_ROLE_PASSTHRU, pol_flip, h_pin_idle_high,
                  mode_valid, cs_hmode());

    /* THE RESUMED METHOD'S V HALF. load_settings() assigned cs_method
     * directly rather than through cs_set(), because at that point
     * there was no hardware to apply it to. This is the point where
     * there is: EVSYS is up, the port is configured, and pin 13 has to
     * start in the state the stored method asks for rather than in
     * whatever port_init() left it in. Without this a saved method 3
     * would come back with a dead pin 13 until the first method change
     * or offset nudge happened to re-route it. */
    v_pin_apply();
    tcb0_init();

    /* TCB1 HAS NEVER BEEN STARTED AT BOOT, and it should have been.
     *
     * It was only ever set up inside apply_role(), which returns early
     * when the role has not changed -- and ccl_role initialises to
     * PASSTHRU, which is the role boot configures. So on a source that
     * never changed role, TCB1 sat disabled forever, last_width_counts
     * stayed 0, and hoffset_apply() fell back to a sixteenth of the line
     * instead of the measured width section 6.4 asks for. It never
     * showed, because a sixteenth of a line is close to 4.7us at every
     * rate this serves -- and because the defence against reading CCMP
     * before the first capture made the symptom benign.
     *
     * One line, and the job owner has to be told what it is doing here
     * regardless, so this is where it goes. */
    tcb1_reevaluate();

    tcf0_init();

    /* Point the DDC client at the resumed mode's image before the
     * hardware could ACK anything, set the addresses up, then let
     * apply_edid_enable() decide from the resumed state whether the
     * client listens at all. */
    edid_select(current_mode);
    twi_client_init();
    apply_edid_enable();

    /* THE HSYNC CAPTURE IS THE ONE INTERRUPT THAT CANNOT BE LATE.
     *
     * TCB0 in frequency mode overwrites CCMP on every capture and leaves
     * CAPT set, so a handler delayed past one line -- 63.6us -- does not
     * merely arrive late, it loses that line entirely. Mode V counts
     * lines, and a lost count moves the emitted Vsync.
     *
     * At level 0 the handler queues behind every other interrupt, and
     * the TWI client is the one that matters: it is busiest during a DDC
     * read, which Windows performs every second or two, and that is
     * exactly when the mode-valid pin was seen dropping out in bursts.
     * Moving the capture out of the main loop and into an interrupt
     * removed the polling latency but not this one.
     *
     * Level 1 is the only priority that can preempt a level 0 handler,
     * and only one vector may hold it. This is the vector that earns it:
     * everything else here -- the tick, the Vsync edge, the button, the
     * TWI client -- tolerates tens of microseconds without consequence.
     *
     * Consequences, both handled above: this handler can now land inside
     * the Vsync ISR, so the frame boundary is passed to it as a flag
     * rather than being done there (see v_frame_req); and cli() still
     * blocks it, since only an NMI ignores the I bit -- which is why
     * ccl_configure() and ccl_set_polarity() may still lose a capture, and
     * why both run only inside a settle mute. */
    CPUINT.LVL1VEC = TCB0_INT_vect_num;

    sei();

    uint16_t start = now_ticks();
    last_hsync_tick = start;

    /* ONE CURTAIN OVER THE WHOLE OF ACQUISITION -- see
     * BOOT_SETTLE_TICKS. This is the only call that needs to change:
     * settle_begin_len() keeps whichever deadline is LONGER, so every
     * 40ms settle raised underneath it during acquisition is absorbed
     * rather than shortening it, and the settle cannot expire early
     * because the expiry test already refuses while pol_pending or
     * pol_measuring is set. */
    settle_begin_len(start, BOOT_SETTLE_TICKS);

    /* Section 8.1: the mode-and-profile report plays at power-on, so a
     * unit says what it is without being asked. On a locked unit this is
     * the only unprompted indication there will ever be. */
    report_mode_profile(start);

    while (1) {
        /* One tick sample per iteration, shared by everything below.
         * The loop has no blocking calls, so it runs many times per
         * millisecond and a single sample is as current as several. */
        uint16_t now = now_ticks();

        /* --- Hsync capture -------------------------------------- */
        if (cap_ready) {
            cap_ready = 0;
            /* Lock-free, and this one mattered more than the others.
             * It used to hold cli() across five memory accesses -- two
             * 16-bit loads, a store, and the flag clear -- which the
             * disassembly puts at about fifteen cycles, 750ns. Any Hsync
             * arriving inside that window had its phase sample delayed
             * by the whole of it, and 750ns is 15 counts, which is
             * precisely what the loop was reporting as its worst error.
             *
             * Read both values twice and retry if either moved. */
            uint16_t period = cap_period;

            last_period_counts = period;
            last_hsync_tick    = now;

            period_average(period);

            /* Shorter of the two halves is the sync pulse, whichever
             * polarity arrived. Read only when TCB1 IS ACTUALLY
             * MEASURING IT -- the timer now has three jobs, and CCMP in
             * the other two is a reshaping period or a field delay, both
             * of which would pass the sanity test below and quietly
             * corrupt the H reconstruction's pulse width. Asking the job
             * rather than the role is the difference between checking
             * what the timer is doing and inferring it. */
            if (tcb1_job == TCB1_JOB_WIDTH) {
                uint16_t hi = TCB1.CCMP;
                /* hi == 0 must be rejected, not just hi >= period.
                 * TCB1 reads zero before its first completed capture,
                 * and zero passed the old test: min(0, period) is 0, so
                 * last_width_counts became 0 and the emitted pulse
                 * clamped to its 1us floor. A 1us pulse where the source
                 * sends 4.7us is not a sync pulse a separator handles
                 * well, and it would show as a picture displaced by a
                 * fixed amount with the edges unstable -- regardless of
                 * anything the offset did. */
                if (hi > 0 && hi < period) {
                    uint16_t lo = (uint16_t)(period - hi);
                    last_width_counts = (hi < lo) ? hi : lo;
                }
            }

            /* Modeline change by period step: more than ~1.6% for eight
             * consecutive lines.
             *
             * pol_ref_period == 0 means there is no reference yet --
             * at start-up, and after every dropout, where the watchdog
             * clears it. Adopt the period silently rather than
             * comparing against zero, which would otherwise read as a
             * 100% step and manufacture a modeline change out of the
             * first eight captures back. The watchdog has already
             * asked for a polarity re-measure in that case, so the
             * streak would have been redundant as well as wrong. */
            if (pol_ref_period == 0) {
                pol_ref_period = period;
                pol_streak     = 0;
            } else {
                uint16_t dp = (period > pol_ref_period)
                            ? (uint16_t)(period - pol_ref_period)
                            : (uint16_t)(pol_ref_period - period);
                if (dp > (period >> POL_PERIOD_SHIFT)) {
                    if (pol_streak < POL_STREAK_REQUIRED) pol_streak++;
                    if (pol_streak >= POL_STREAK_REQUIRED) {
                        pol_ref_period = period;
                        pol_streak     = 0;
                        pol_pending       = 1;
                        width_pending     = 1;  /* new modeline, new width */
                        genlock_reacquire = 1;
                        /* THE LONG ONE. A card changing modeline does
                         * not go straight there: it emits genuinely
                         * stable intermediate modes for tens of
                         * milliseconds each, and where those land in an
                         * accepted window -- Modes 3 and 5 take 15, 25
                         * and 31kHz -- the unit correctly reports a
                         * valid source and unmutes into it. Measured at
                         * 100ms coming down from 31kHz and 200ms going
                         * up.
                         *
                         * Nothing in the signal separates a stable
                         * intermediate 25kHz from a real 25kHz source,
                         * because it IS one. A stability test was tried
                         * and reverted; see hsync_valid below. So this
                         * is a timer, knowingly, and it is the only
                         * mechanism left.
                         *
                         * It retriggers on each step, so the mute runs
                         * from the LAST one rather than the first and
                         * a slower card is covered without a longer
                         * constant. */
                        settle_begin_len(now, SETTLE_TICKS_SOURCE);
                    }
                } else {
                    pol_streak = 0;
                }
            }

            uint8_t windows = 0;
            if (period >= HSYNC_FAST_MIN_COUNTS
             && period <= HSYNC_FAST_MAX_COUNTS) windows |= W_FAST;
            if (period >= HSYNC_SLOW_MIN_COUNTS
             && period <= HSYNC_SLOW_MAX_COUNTS) windows |= W_SLOW;
            if (period >= HSYNC_M4_MIN_COUNTS
             && period <= HSYNC_M4_MAX_COUNTS)   windows |= W_M4;

            /* Computed straight from this period's window check, mode
             * by mode, with no smoothing or debounce on top. An earlier
             * ATtiny824 revision added hysteresis and then a longer-
             * timescale block mechanism to smooth boundary flicker on
             * Mode 4's narrow window; each layer fixed the failure mode
             * it targeted and was immediately followed by a new one at
             * a different timescale. It was fully reverted. */
            last_windows = windows;
            /* Inside an adjustment mode the acceptance window comes
             * from MODE_5, not from the mode being adjusted.
             *
             * Mode S normally accepts 15kHz and 31kHz and divides the
             * fast one, which is right for Mode S itself -- but an
             * adjustment mode is a place you sit and watch a picture
             * while turning a control, and a source that drifts out of
             * the window mid-adjustment would blank the very picture
             * being adjusted. MODE_5 accepts all three windows, so the
             * picture stays up. Applies to Mode V and Mode FO too when
             * they arrive, and to anything else added inside Mode S. */
            /* MODE CS KEEPS THE NUMBERED MODE'S WINDOW; the other
             * adjustment modes widen to Mode 5's.
             *
             * Widening is right for H, V and FO: you are tuning an
             * offset and the picture has to be visible while you do it,
             * which is the same reasoning behind the ui_state guard on
             * the DIVIDE role above.
             *
             * It is wrong for Mode CS, because a combine method is a
             * property OF the mode it was chosen in. Widening would
             * bring a source outside Mode 4's window on screen while
             * the user picks a method for it, then drop it again on
             * exit -- the method validated against a window it will
             * never run in. Mode ED widens and is deliberately left
             * doing so: it edits EDID rather than sync behaviour, so
             * the argument does not carry across. */
            uint8_t win = (ui_state == UI_NORMAL || ui_state == UI_ADJ_CS)
                            ? mode_windows[current_mode]
                            : mode_windows[MODE_5];
            /* MEMBERSHIP ONLY. A stability test was tried here and
             * reverted: the same tolerance and streak the step detector
             * uses, inverted to mean "the rate has stopped moving",
             * gating the valid direction.
             *
             * It made no difference, which says the premise was wrong.
             * The card is not sweeping continuously through 25kHz on
             * its way to 31kHz -- if it were, no streak could form and
             * the test would have worked. It is far more likely to be
             * emitting genuinely stable intermediate modes for tens of
             * milliseconds each, and a stable 25kHz signal is
             * indistinguishable from a real 25kHz source because that
             * is exactly what it is. No amount of looking at the
             * signal separates them.
             *
             * Which leaves only a timer -- ignore everything for N ms
             * after a change -- and that is the block mechanism the
             * ATtiny824 grew and then reverted, described three lines
             * above. Not adding it a second time. */
            hsync_valid = (windows & win) ? 1 : 0;

            /* Only act on a window the current mode actually accepts.
             * Deciding the role from an unaccepted window would have
             * Mode S engage the divider for a source it is about to
             * mute anyway. */
            /* HIDDEN BY THE BOOT CURTAIN, NOT FIXED -- do not let this
         * stand as evidence the ordering is right.
         *
         * hoffset_start() runs BELOW this call, so on the first good
         * capture after reset or a dropout hoffset_running is still 0
         * and role_wanted() returns PASSTHRU. The role only reaches
         * RECON on a later pass, from the H-offset block further down.
         *
         * The cost is not the capture of latency it looks like. The
         * first polarity measurement takes 20ms, and the role is still
         * moving while it runs -- so want_flip gets computed against a
         * transient role, then recomputed against the settled one, and
         * each answer raises its own settle. That was the boot strobe.
         * BOOT_SETTLE_TICKS covers it; the ordering is still wrong.
         *
         * Reordering means starting the H offset before the role that
         * consumes it is chosen, which is a change to working
         * acquisition code and wants its own increment and its own
         * bench round. */
        if (hsync_valid) apply_role(role_wanted(windows), now);

            /* Acquire on the first good line, then never restart. A
             * modeline change arrives as a period step, which the block
             * above has already flagged -- and that is the one event
             * that justifies re-acquiring, because the line rate itself
             * has moved. A role change is not: the rate is unchanged
             * and the loop has nothing to relearn. */
            if (hsync_valid && !hoffset_running) hoffset_start(period);
            else if (hoffset_running && genlock_reacquire) {
                /* Only a genuine rate change matters now, and only
                 * because the offset step and the pulse width are
                 * scaled from the measured period. */
                genlock_reacquire = 0;
                period_avg_ok = 0;          /* the rate moved: start over */
                period_average(period);
                hoffset_apply(period_best_hires());
            }
        }

        /* --- H offset: re-place the pulse when the setting moves - */
        /* The offset is a setting, not a per-line quantity: the
         * hardware places every pulse from CMP0 and CMP1 without help,
         * so these only need rewriting when the setting changes. */
        if (hoffset_running) {
            static uint8_t last_off = 0xFF;

            if (last_off != h_offset) {
                last_off = h_offset;
                hoffset_apply(period_best_hires());
                apply_role(role_wanted(last_windows), now);
            }

            /* NO PERIOD REFRESH, and the measurement is why.
             *
             * Re-applying the negative side's placement as the averaged
             * period moved was measured at 180ns of edge spread against
             * 110ns with it frozen. Every rewrite moves the pulse, and
             * moving it more often does not make the moves smaller --
             * it just adds a second source of displacement on top of
             * whatever the first one was.
             *
             * The average is still used, once, for the placement made
             * at acquisition and on each offset change: a single
             * capture is quantised to a whole count and can be 25ns
             * wrong, where the average is good to about 3ns. That is a
             * static accuracy gain and costs nothing. What does not
             * work is treating it as something to track.
             *
             * The rejected refresh was kept for a while as an
             * `else if (0)` branch. The compiler discarded it, so it
             * cost nothing to run and everything to read: a reader has
             * to work out that the condition is a constant before they
             * can stop reasoning about when it fires. The reasoning is
             * worth keeping; the code was not. */
        }

        /* --- Sync watchdog -------------------------------------- */
        /* HSYNC_WATCHDOG_TICKS being too small is not a mild problem,
         * because a false watchdog does two things that compound. It
         * re-arms the polarity measurement, and it clears the reference
         * period. The cleared reference then reads as a 100% step on
         * every following capture, so the eight-line streak completes
         * in half a millisecond and calls settle_begin(), which starts
         * a 40ms mute. Fire the watchdog once per millisecond and that
         * mute is restarted forty times before it can expire: Csync is
         * muted permanently, mode-valid never goes high, and the whole
         * thing looks like a source that is never accepted. */
        if (elapsed_since(now, last_hsync_tick) >= HSYNC_WATCHDOG_TICKS) {
            if (hsync_valid) {
                hsync_valid = 0;
                settle_begin(now);
            }
            /* Fall back to pass-through while there is no source. The
             * divider would otherwise sit clocked by a dead pin, and
             * whatever level the flip-flop stopped at would be frozen
             * into the combiner. Pass-through leaves the array in the
             * shape the next source is most likely to want, and
             * role_wanted() puts the divider back if it is a fast one. */
            apply_role(CCL_ROLE_PASSTHRU, now);
            hoffset_stop();
            /* A dropout is the reliable announcement of a modeline
             * change: a GPU reprogramming its PLL always drops sync.
             * Arm the re-measure for whenever captures come back. */
            pol_pending       = 1;
            width_pending     = 1;  /* sync went away; re-measure it */
            genlock_reacquire = 1;
            pol_ref_period    = 0;
        }

        /* --- Polarity ------------------------------------------- */
        if (elapsed_since(now, pol_recheck_at) >= POL_RECHECK_MS) {
            pol_recheck_at = now;
            pol_recheck    = 1;
        }

        if ((pol_pending || pol_recheck) && hsync_valid) {
            polarity_measure(now);
        } else if (pol_measuring && !hsync_valid) {
            /* Abandoned rather than guessed: with no Hsync arriving
             * there is nothing to measure against. */
            pol_measuring = 0;
        }

        /* --- Output gating -------------------------------------- */
        /* THE SETTLE DOES NOT EXPIRE WHILE A POLARITY DECISION IS
         * STILL OUTSTANDING.
         *
         * A modeline change fires the period-step streak, which arms
         * this settle AND asks for a polarity re-measure. The settle is
         * a fixed 40ms from the disturbance; the measurement cannot
         * start until Hsync comes back in-window, and takes 20ms once
         * it does. On a source that takes its time returning, the timer
         * runs out first and the output unmutes into the middle of an
         * unfinished decision -- then mutes again when the answer
         * arrives and the truth tables are rewritten.
         *
         * Two mutes with a live gap between them, from one event. The
         * gap is the defect, not either mute.
         *
         * Holding the settle open bounds it by the decision rather than
         * by a guessed duration, so there is no constant to tune and no
         * source too slow for it. It cannot hang: pol_measuring is
         * abandoned above when Hsync goes away, and pol_pending only
         * keeps the mute up while hsync_valid is false, which is muting
         * the output anyway. */
        if (settling && !pol_pending && !pol_measuring
         && elapsed_since(now, settle_start) >= settle_len) {
            settling = 0;
        }
        /* Csync is gated on the field rate too. A rate a CRT cannot
         * follow is worse than no output: the set hunts, and on some
         * older monitors that is not merely ugly. Blanking says
         * plainly that this source will not work here. */
        set_mode_valid(hsync_valid && field_rate_ok
                       && !settling && !pol_pending);

        /* --- Vsync presence, for the sync report ----------------- */
        update_vsync(now);

        /* --- Button --------------------------------------------- */
        update_b1(now);

        /* --- Deferred single press in MS (section 8.2) ----------- */
        if (ms_press_pending
         && elapsed_since(now, ms_press_at) >= MS_DOUBLE_MS) {
            ms_press_pending = 0;
            change_mode(MODE_1, now);
            report_mode_profile(now);
        }

        /* --- Mode V: substitute the regenerated Vsync ------------
         *
         * Only when the offset is off centre AND a frame length is
         * known, so a unit that has never seen a complete frame cannot
         * emit a Vsync from a guess. At centre the pin is handed back
         * and the source passes through untouched.
         *
         * INTERLACED SOURCES ARE NOW SERVED, not refused. The count is
         * referenced to one field per frame and two pulses are emitted
         * from it, the second carrying half a line from Mode FO's
         * one-shot. The frame length matters more than it used to --
         * it now places the second pulse as well as the negative
         * offsets -- which is why the test on it stays. */
        v_set_source(v_offset != V_OFFSET_DEFAULT && v_frame_lines
                     && vsync_present);

        /* --- Mode FO: the per-field delay ------------------------
         *
         * Same argument in the mirror. Engaged only on an interlaced
         * source, off centre, with a measured line period to scale the
         * delay from -- and it stands down for the polarity window's
         * width burst, which wants TCB1 back. */
        fo_update();
        fo_load_service(now);

        /* --- Lockout, and its repeating refusal ------------------- */
        cannot_help_on = adj_locked_out();

        /* TRUNCATED THE INSTANT THE SOURCE BECOMES USABLE.
         *
         * The refusal runs 4.3 seconds. Letting it finish after the
         * condition has cleared means up to four seconds of a control
         * saying "I cannot help" while it can, in fact, help -- and the
         * obvious thing to do during those four seconds is press the
         * button, which now works, so the picture moves while the LED
         * insists nothing is happening. Contradicting itself is worse
         * than either message alone.
         *
         * Only the refusal is cut. cannot_help_playing marks the urgent
         * player as carrying THIS pattern, so a future urgent user is
         * not silently truncated by a condition that has nothing to do
         * with it. Nothing is played in its place: blink_service clears
         * the pin on the same pass, and in Mode FO the rate blink picks
         * it up on the next one. */
        if (!cannot_help_on && cannot_help_playing) {
            cannot_help_playing = 0;
            urgent_active       = 0;
        }

        if (cannot_help_on && !b1_pressed && !blink_busy()
         && elapsed_since(now, cannot_help_at) >= ADJ_STROBE_MS) {
            /* The gap between repeats is the 1200ms of dark at the end
             * of the pattern itself, so waiting for the player to go
             * idle is enough. The elapsed test only matters at the
             * moment the mode opens, where cannot_help_at is seeded with
             * the entry time: that buys one gap's grace before the first
             * refusal, which the interlace detector -- four alternations,
             * four frames, under 70ms -- comfortably fits inside. An
             * interlaced source is never accused of being progressive
             * merely for not having been measured yet.
             *
             * Suppressed while B1 is down because the hold preview owns
             * the LED then, and a refusal fired underneath it would be
             * invisible and would have consumed its own repeat slot. */
            cannot_help_at      = now;
            cannot_help_playing = 1;
            blink_urgent(pat_cannot_help,
                         (uint8_t)(sizeof pat_cannot_help / 2), now);
        }

        /* --- Strobe expiry ---------------------------------------
         *
         * The strobe has to time out on its own, not merely when the
         * next press arrives. It was cleared only inside b1_release, so
         * after a landmark the flag stayed set indefinitely -- and
         * because the LED block suppresses the hold preview while it is
         * set, every subsequent hold went dark. The actions still ran;
         * only the indication vanished, which made a working control
         * look dead. */
        if (adj_strobing
         && elapsed_since(now, adj_strobe_at) >= ADJ_STROBE_MS) {
            adj_strobing = 0;
        }

        /* --- Adjustment mode idle timeout (section 8.3) ---------- */

        /* A HELD BUTTON IS ACTIVITY. adj_last_input is written only on
         * RELEASE -- at the top of the adjustment branch of
         * b1_release(), and on entry to each mode. Both are release
         * events, so without the !b1_pressed term a hold that spans the
         * deadline fires the discard MID-PRESS, and the damage is not
         * confined to the discard:
         *
         * In Mode H at 19s, press and hold intending a 4s save. At 20s
         * this block reverts h_offset to adj_entry and sets ui_state to
         * UI_NORMAL. At 4s the button comes up, and b1_release() no
         * longer takes the adjustment branch -- ui_state says NORMAL --
         * so it falls into the Mode S table instead, where held >=
         * HOLD_MODE_V_MS lands the user in MODE V with the H offset
         * silently thrown away. The save gesture became a mode change.
         *
         * The release path refreshes adj_last_input itself, so holding
         * the timeout off while the button is down is the whole fix;
         * the countdown restarts from the release. */
        if (ui_state != UI_NORMAL
         && !b1_pressed
         && elapsed_since(now, adj_last_input)
            >= ((ui_state == UI_ADJ_ED) ? ED_IDLE_MS : ADJ_IDLE_MS)) {
            /* Discard: walking away must not commit a half-made change,
             * and the unit is left where it was rather than where it was
             * being dragged to.
             *
             * Mode ED gets 30s rather than 20s, and discards BOTH axes
             * -- the same argument as saving both. The re-render is not
             * optional: ed_pos and ed_size have been driving the served
             * EDID all the way through the session, so putting the
             * variables back without rebuilding would leave the shadow
             * holding the abandoned geometry. Live state and stored
             * state, together, in one breath -- section 19.2. */
            if (ui_state == UI_ADJ_ED) {
                ed_pos  = ed_entry_pos;
                ed_size = ed_entry_size;
                edid_select(current_mode);
            } else if (ui_state == UI_ADJ_CS) {
                /* Through cs_set(), so the truth table goes back with
                 * the variable. Assigning cs_method here would leave
                 * pin 12 in the abandoned method with the setting
                 * saying otherwise -- the same shape as leaving the
                 * EDID shadow holding abandoned geometry, three lines
                 * up. And it must be tested BEFORE the else: adj_cur()
                 * has no row for UI_ADJ_CS and the store below would be
                 * through a null pointer. */
                cs_set(cs_entry, now);
                cs_report_n = 0;
            } else {
                *adj_cur()->value = adj_entry;
            }
            ui_state = UI_NORMAL;
            report_mode_profile(now);
        }

        /* --- Sync report (section 8.2) --------------------------- */
        if (current_mode == MODE_S && ui_state == UI_NORMAL && !b1_pressed) {
            /* A free-running cycle rather than a timer armed by the
             * fault, so one mechanism reports both failure and recovery.
             * Any button edge restarts it, so a report cannot land on
             * top of another pattern. */
            static uint16_t sync_at;
            if (elapsed_since(now, b1_edge_at) < 5000) {
                sync_at = now;
            } else if (elapsed_since(now, sync_at) >= 5000) {
                sync_at = now;
                if (!blink_busy()) play_sync_report(now);
            }
        }

        /* THE BLINK LED, PIN 5, HAS FIVE CLAIMANTS. In order,
         * strongest first:
         *
         *   1  the hold preview, while B1 is down. It has to say what
         *      releasing at this instant will do, and nothing else may
         *      interrupt that.
         *   2  the cannot-help refusal, on the urgent player.
         *   3  the 1200ms landmark strobes, on the normal player.
         *   4  Mode FO's rate blink, which is the background state of
         *      that mode and yields to everything.
         *   5  Mode CS's method count, which is a brightness curve
         *      rather than a pattern and so cannot be held by the blink
         *      engine either.
         *
         * The CS report tests cs_report_n INSTEAD of blink_busy(), and
         * the difference is deliberate. The rate blinks are a
         * background state with no end, so they must stand aside for
         * any pattern that wants to play. The CS report is a finite
         * answer to a press that has just happened, it stopped the
         * players itself when it started, and it is the ONLY indication
         * that mode has -- deferring it to a pattern would mean the
         * user pressed the button and nothing happened. It also needs
         * no cannot_help_on term: Mode CS has no lockout, and
         * adj_locked_out() is not given a case for it.
         *
         * cs_report_n is left set on exit rather than cleared. It is
         * only ever read while ui_state is UI_ADJ_CS, and entry always
         * calls cs_report_start(), so a stale count cannot be reached.
         *
         * The rate blink is tested against both players rather than
         * being folded into the blink engine, because it is not a
         * pattern: it has no end, and a player that never finishes would
         * block every pattern behind it forever. */
        if (b1_pressed && !adj_strobing)
            hold_preview(elapsed_since(now, b1_down), now);
        else if (ui_state == UI_ADJ_FO && !cannot_help_on && !blink_busy())
            fo_rate_blink(now);
        else if (ui_state == UI_ADJ_ED && !cannot_help_on && !blink_busy())
            ed_rate_blink(now);
        else if (ui_state == UI_ADJ_CS && cs_report_n)
            cs_report_run(now);
        else
            blink_service(now);
    }
}
