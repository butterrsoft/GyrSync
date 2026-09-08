/* CCL -- SINGLE OWNER OF ALL FOUR LUTs, EVERY TRUTH TABLE, AND
 * PORTMUX.CCLROUTEA.
 *
 * The CCL was written from four places: ccl_configure() built the
 * array, ccl_set_polarity() rewrote TRUTH1 and TRUTH3 underneath it,
 * fo_lut0_build() owned LUT0 separately, and evsys_init() set
 * CCLROUTEA. Three of those are legitimate -- the truth tables really
 * do have to move at run time -- but nothing made them one owner
 * except the order they happened to be called in.
 *
 * WHAT IS DELIBERATELY NOT HERE: which role the design wants. That
 * depends on the offset, the mode and the accepted window, and it is
 * policy. This file is told which role to build and what sense the
 * source has; it does not decide either. The polarity state is passed
 * in for the same reason -- a module that reads pol_flip for itself
 * can disagree with the code that set it.
 *
 * ROLE TABLE AS BUILT TODAY. This was stale until rev 1 was deleted --
 * it described the PRE-ROTATION roles as current and the rotation as
 * something that would happen later. It had already happened.
 *
 * | LUT  | PASSTHRU / RECON                    | DIVIDE (Mode S)      |
 * |------|-------------------------------------|----------------------|
 * | LUT0 | Mode FO Vsync conditioner           | J, clocked by Hsync  |
 * |      | (no OUTEN -- its pin is SDA/SCL)    | on EVENTB            |
 * | LUT1 | Hsync edge sense -> TCE0 restart    | K, held high         |
 * |      | (writes CCL.TRUTH1 via                |                      |
 * |      |  edge_sense_truth())                  |                      |
 * | LUT2 | the combiner, OUTEN on pin 12       | combiner, H from     |
 * |      |                                     | TCB1 on IN2          |
 * | LUT3 | reconstruction XOR of TCE0 WO0/WO1  | disabled             |
 *
 * WHY IT IS THIS WAY AND NOT THE OBVIOUS WAY: only PC3 and PD6 can be
 * CCL outputs, and SCL took PC3 when the oscillator took PA0. So the
 * output stage has to be LUT2, the reconstruction XOR has to feed it by
 * LINK -- and LINK on LUT(n) reads LUT(n+1), so it can only come from
 * LUT3 -- and LUT1 is what is left for the edge sense. LUTnCTRLB shares
 * one INSEL table across all four LUTs, so any of them can select TCE0
 * WO0/WO1; that is what makes the rotation possible at all.
 *
 * The J-K is on SEQ0 rather than SEQ1 because SEQ1's even half is the
 * output stage now.
 *
 * COMMENTS BELOW THAT SAY "rev 1" MEAN THE PRE-ROTATION WIRING, which
 * no longer exists in this tree. They are kept where they explain why
 * something is the shape it is -- several of the traps in this file
 * were only understood by comparing the two. Grepping for BOARD_REV
 * will find nothing; that is intended. */

#include "pins.h"
#include "ccl.h"

#include <avr/io.h>
#include <avr/interrupt.h>

/* THE CURRENT ROLE LIVES HERE, with the code that installs it.
 *
 * It used to be a file-scope variable in main.c that ccl_configure()
 * assigned as a side effect and four other places read -- so the
 * authority for "what shape is the CCL in" sat in a different file from
 * the register writes that made it that shape. Reads go through
 * ccl_current_role() now; there is no setter, because the only way to
 * change the role is to build it. */
static ccl_role_t ccl_role = CCL_ROLE_PASSTHRU;

ccl_role_t ccl_current_role(void)
{
    return ccl_role;
}

/* THE GATE IS A CHOICE OF TABLE, NOT AN INPUT.
 *
 * It used to be a pin: pin 10 driven out, read straight back in through
 * EVSYS, and combined at the combiner's IN2, so muting was
 * combinational and took effect within a LUT propagation delay. That
 * cost a pin the rotation needed -- only PC3 and PD6 can be CCL
 * outputs, SCL takes PC3 when the oscillator takes PA0, so Csync moves
 * to PD6, Mode V's Vsync to PD7, and the blink LED has nowhere to go
 * but the pin the gate was using.
 *
 * The pin version was kept behind GATE_VIA_PIN for a direct comparison
 * and has now been deleted: it needs a pin that does not exist on this
 * wiring, so it could not be built, and an option that cannot be
 * selected is not an option -- it is unreachable code that every future
 * edit has to be checked against.
 *
 * WHAT THE CHANGE COST, since it is no longer measurable by rebuilding
 * the other way: the combiner's truth table is enable-protected, so
 * changing it means disabling and re-enabling the LUT, and Csync is
 * released for those few cycles. Muting takes effect after a brief
 * release of the output rather than within a propagation delay. Going
 * INTO a mute that is harmless -- the output was about to go idle
 * anyway. Coming out it lands inside the settle window, while the
 * display is resyncing regardless. Judged acceptable on the bench.
 *
 * The truth index is IN2:IN1:IN0, and IN2 is masked to a constant 0,
 * so only indices 0 to 3 can ever be read. The gated pair were 0x90
 * (XNOR at indices 4 and 7) and 0x60 (XOR at 5 and 6); masking IN2
 * moves those live entries down to 0/3 and 1/2, giving 0x09 and 0x06.
 * All-zeros is idle. */
/* NAMED FOR THE JOB, NOT FOR A LUT NUMBER -- and that is a correction.
 *
 * Every constant and helper below used to be called TRUTH1_something,
 * because before the rotation the combiner WAS LUT1. It is LUT2 now.
 * The names did not follow, so this file reached the state where
 * grepping for "LUT1" returned the combiner's tables, while the code
 * that actually writes CCL.TRUTH1 -- the Hsync edge sense -- was called
 * hoff_lut3_truth(). Both halves of that pointed at the wrong LUT.
 *
 * This file has already paid three times for two writers on TRUTH2. A
 * name that sends the next reader to the wrong register is how a fourth
 * one gets added, so the names say COMB (the combiner, whichever LUT
 * that is) and EDGE (the restart edge sense) instead. */
#define COMB_MUTE       0x00 /* IN2 masked: output constant idle */
#define COMB_SAME       0x09 /* IN2 masked: XNOR of IN1, IN0 */
#define COMB_DIFF       0x06 /* IN2 masked: XOR  of IN1, IN0 */

/* One place decides what the combiner's XNOR/XOR table should be, so
 * the mute state and the polarity cannot be combined differently by two
 * callers. Both are passed in; neither is cached here. */
static uint8_t comb_xor_truth(uint8_t pol_flip, uint8_t gate_open)
{
    if (!gate_open) return COMB_MUTE;
    return pol_flip ? COMB_DIFF : COMB_SAME;
}

/* THE COMBINER'S TABLE, and the ONLY place it is decided.
 *
 * In DIVIDE the reshaped H arrives on IN2, not IN1 -- IN1 does
 * not deliver TCB1 to LUT2, whatever the shared INSEL table says -- so
 * the live entries move. The index is IN2:IN1:IN0 with the unused input
 * masked to 0: with H on IN1 that is 0x09 XNOR and 0x06 XOR, and with H
 * on IN2 it is 0x21 and 0x12.
 *
 * ccl_configure() and ccl_set_polarity() both write this register, and
 * the first version of the IN2 fix taught only one of them the new
 * table. Every source change re-measures polarity, so the second writer
 * put the IN1 table back and the output collapsed to a function of
 * Vsync alone -- a picture on entry to Mode S, 60Hz after any source
 * change. Two writers, one register, two conventions. */
/* MODE CS: H ALONE ON THE OUTPUT, section 17.1.
 *
 * PASS and FLIP drop V out of the function. With H on IN1 that is
 * "output follows IN1" -- indices 2 and 3, 0x0C -- and with H on IN2 it
 * is indices 4 and 5, 0x30. The inverses are indices 0 and 1 either way,
 * which is 0x03 in both. The collision is a coincidence of which input
 * is masked, not a shared meaning, so both are named.
 *
 * IN1 is masked in DIVIDE and IN2 is masked everywhere else, so only
 * half the table is reachable in each case and the unreachable bits are
 * left clear rather than filled. 0xF0 would work as well as 0x30 and
 * says something untrue about what the LUT can be asked. */
#define COMB_H_ON_IN1    0x0C /* IN2 masked: output = IN1        */
#define COMB_H_ON_IN2    0x30 /* IN1 masked: output = IN2        */
#define COMB_H_INVERT    0x03 /* either masking: output = NOT H  */

/* CSYNC WITHOUT SERRATIONS -- Mode CS method 3.
 *
 * The combiner's normal table is an exclusive-or, so through the Vsync
 * interval the Hsync pulses come back INVERTED: the broad pulse with
 * serrations cut into it, which is what a correctly formed Csync wants.
 * This variant drops them. Vsync simply overrides H, and the output
 * sits at the active level for the whole interval.
 *
 * It is the "AND logic" form from the Engineering CSYNC articles, and
 * it is offered because some displays want it, not because it is
 * better. It is not: a receiver's line PLL gets no edges to hold onto
 * through the Vsync interval and has to coast or re-lock, which is the
 * failure the serrations exist to prevent. Many CRTs do not care. Some
 * decoders visibly prefer it. It is a user choice and method 1 remains
 * the default.
 *
 * ONE INDEX IS HIGH AND EVERY OTHER IS LOW. The output is at its idle
 * level only when H is inactive AND V is inactive, so the table is a
 * single set bit at the index where both inputs sit at their idle
 * values -- 1 << (H_idle_position + V_idle_position). That needs the
 * ABSOLUTE idle levels, where the exclusive-or only ever needed whether
 * they agreed: XNOR is symmetric and gives the same answer for both-low
 * and both-high, and this is not.
 *
 * Both levels are recoverable from what is already passed in, so no new
 * argument was added. ccl_set_polarity() forces the combiner's H idle to
 * 0 outside PASSTHRU and keeps the raw measurement in h_pin_idle_high,
 * which is exactly the reconstruction below; and pol_flip is by
 * definition (h_idle != v_idle), so the V idle follows. Recomputing
 * them here rather than passing them keeps the caller's argument list
 * describing the SOURCE, and this file deriving what its own inputs
 * look like -- which is the split that stopped the IN2 table drifting
 * from the IN1 one. */

/* THE COMBINER'S TABLE, and the ONLY place it is decided.
 *
 * ...continued. "PASS" MEANS THE SOURCE'S SENSE, AND THE COMBINER IS
 * NOT ALWAYS LOOKING AT THE SOURCE. In PASSTHRU its H input is the pin
 * itself, so passing it through preserves whatever the source sends.
 * In RECON it is LUT3's rebuilt pulse and in DIVIDE it is TCB1's
 * reshaped one, and BOTH of those idle low with a positive-going pulse
 * regardless of the source -- which is exactly why ccl_set_polarity()
 * forces h_idle_high to 0 outside PASSTHRU and keeps the raw answer
 * separately in h_pin_idle_high.
 *
 * So on those two paths a negative-going source needs the table
 * INVERTED to come out the way it went in. Without that term, engaging
 * the H offset would silently flip the polarity of a "pass" output, and
 * it would look like the offset control breaking the picture rather
 * than like a polarity fault -- the same class of confusion the LUT3
 * freeze produced.
 *
 * pol_flip cannot carry this. It is (h_idle_high != v_idle_high) AFTER
 * both have been forced, so it is a statement about agreement between
 * two things the combiner sees; the raw pin sense is not recoverable
 * from it. h_pin_idle_high is the measurement and is passed in for it.
 *
 * FLIP is PASS inverted, and is computed that way rather than tabulated
 * so the two cannot drift apart. */
static uint8_t combiner_truth(uint8_t pol_flip, uint8_t gate_open,
                              ccl_role_t role, uint8_t h_pin_idle_high,
                              ccl_hmode_t hmode)
{
    /* The mute comes first for every role and every method: an output
     * that is not allowed to speak has nothing to say about polarity. */
    if (!gate_open) return COMB_MUTE;

    if (hmode == CCL_H_PASS || hmode == CCL_H_FLIP) {
        uint8_t inv = (uint8_t)(role != CCL_ROLE_PASSTHRU
                             && h_pin_idle_high);
        if (hmode == CCL_H_FLIP) inv = (uint8_t)!inv;

        if (inv) return COMB_H_INVERT;
        return (role == CCL_ROLE_DIVIDE) ? COMB_H_ON_IN2
                                         : COMB_H_ON_IN1;
    }

    if (hmode == CCL_H_FLAT) {
        /* The combiner's own view of the two idle levels, not the
         * source's -- the reconstruction and the divider both idle low
         * whatever arrived, which is what the forcing in
         * ccl_set_polarity() says and what this mirrors. */
        uint8_t hi = (uint8_t)(role == CCL_ROLE_PASSTHRU
                               ? (h_pin_idle_high ? 1 : 0) : 0);
        uint8_t vi = (uint8_t)(hi ^ (pol_flip ? 1 : 0));

        /* H sits on IN2 in DIVIDE and IN1 everywhere else; the masked
         * input is always 0, so it contributes nothing to the index. */
        uint8_t shift = (uint8_t)((role == CCL_ROLE_DIVIDE ? (hi << 2)
                                                           : (hi << 1))
                                  + vi);
        return (uint8_t)(1u << shift);
    }

    if (role == CCL_ROLE_DIVIDE) return pol_flip ? 0x12 : 0x21;  /* IN2 */
    return comb_xor_truth(pol_flip, gate_open);                  /* IN1 */
}


/* SEQ1's two feeder LUTs. A JK flip-flop toggles when J and K are both
 * high, so both LUTs must output a constant 1 and the division comes
 * entirely from the clock edge.
 *
 * 0xFF -- output 1 for every input combination -- rather than the more
 * usual "mask every input and set bit 0". The two are equivalent only
 * if IN2 is consumed by the clock mux and does not also reach the truth
 * table. The block diagram can be read either way, and 0xFF is constant
 * under both readings, so the ambiguity cannot bite. It costs nothing:
 * a truth table is a truth table whatever is written in it. */
#define TRUTH_CONST_ONE   0xFF

/* THE RESTART EDGE SENSE. LUT1 presents Hsync to TCE0's restart with a
 * rising edge where the restart is wanted. One input, masked elsewhere,
 * so the truth index is just IN0: bit 1 passes it through, bit 0
 * inverts it.
 *
 * This was called hoff_lut3_truth(), from the pre-rotation wiring where
 * LUT3 held the job. It writes CCL.TRUTH1.
 *
 * Always the pulse's LEADING edge, which is what the picture is
 * positioned by. For negative-going sync -- the usual case -- that is
 * the pin's falling edge, so the LUT inverts.
 *
 * The negative side pays for this: its pulse sits near the end of the
 * cycle and the next restart cuts it short, leaving 2.0us near centre
 * where the source sends 4.7us. Restarting on the trailing edge for
 * negative offsets fixes that, was tried, and was reverted -- it made
 * the emitted position depend on the measured pulse width, which is
 * quantised per line, and the picture ticked sideways whenever the
 * measurement moved by a count. */
static uint8_t edge_sense_truth(uint8_t h_pin_idle_high)
{
    return h_pin_idle_high ? 0x01 : 0x02;
}

void ccl_set_polarity(uint8_t pol_flip, uint8_t h_pin_idle_high,
                      uint8_t gate_open, ccl_hmode_t hmode)
{
    /* TRUTH1 is enable-protected: it can only be written while LUT1 is
     * disabled. Csync is muted by the settle window around this, and
     * the gate is combinational rather than gating the LUT's ENABLE, so
     * the pin is held at idle by the truth table itself throughout.
     *
     * Interrupts stay off across the disable/write/enable so the TWI
     * ISR cannot lengthen the gap. The hardware stretches SCL until the
     * ISR responds, which I2C hosts tolerate by design. */
    uint8_t s = SREG;
    cli();
    /* The combiner is LUT2, so the gate and the polarity live in
     * TRUTH2. */
    CCL.LUT2CTRLA &= (uint8_t)~CCL_ENABLE_bm;
    CCL.TRUTH2     = combiner_truth(pol_flip, gate_open, ccl_role,
                                    h_pin_idle_high, hmode);
    CCL.LUT2CTRLA |= CCL_ENABLE_bm;

    /* LUT3 decides which edge of the incoming pulse restarts TCE0, and
     * that depends on the source's sense -- so it has to move with the
     * polarity, not just be set once when the array was built. */
    if (ccl_role != CCL_ROLE_DIVIDE) {
        /* The edge sense is LUT1 -- LUT3 does the reconstruction
         * XOR, and its table never moves. */
        CCL.LUT1CTRLA &= (uint8_t)~CCL_ENABLE_bm;
        CCL.TRUTH1     = edge_sense_truth(h_pin_idle_high);
        CCL.LUT1CTRLA |= CCL_ENABLE_bm;
    }
    SREG = s;
}

/* LUT0: the emitted Vsync is the source's, held at idle while TCB1's
 * one-shot is high.
 *
 *   IN0  EVENTA  the raw Vsync, on CHANNEL5
 *   IN1  TCB1    the delay one-shot
 *   IN2  MASK    unused, forced low
 *
 * The output is IDLE LOW with a positive-going pulse whichever way the
 * source runs, which is why polarity_measure() forces the combiner's
 * v_idle_high to 0 while this is engaged -- the same forcing Mode V
 * needs for its regenerated pulse on pin 13, for the same reason.
 *
 * Truth index is IN2:IN1:IN0, and only IN2 = 0 rows can ever be taken.
 *   source idles HIGH (negative-going): active is IN0 = 0, so row 0.
 *   source idles LOW  (positive-going): active is IN0 = 1, so row 1.
 *
 * NO OUTEN, and this one matters more than usual: LUT0's output pin on
 * this part is in PORTA, where PA0 and PA1 are SDA and SCL. Enabling it
 * would drive the DDC lines exactly as TCE0's default routing did. The
 * CCL reads LUT0 through the event system, which needs no pin at all. */
void ccl_fo_lut0_build(uint8_t on, uint8_t trigger_falling)
{
    /* LUT0 HAS ONE OWNER AT A TIME, AND IN DIVIDE IT IS NOT THIS ONE.
     *
     * The Mode S divider's J-half is on LUT0, because SEQ1's even
     * half is the output stage. That was justified by DIVIDE
     * and Mode FO being mutually exclusive -- true of what the modes
     * MEAN, but nothing enforced it, and a source changing from 15kHz
     * interlaced to 31kHz arrives here with fo_active still set. It
     * then rebuilt LUT0 and destroyed the J-K that ccl_configure() had
     * just built, which is why Mode S divided correctly from boot and
     * not from a source change.
     *
     * Refusing here rather than at the caller keeps the rule with the
     * register: whoever calls this, LUT0 stays the divider's while the
     * divider is running. */
    if (ccl_role == CCL_ROLE_DIVIDE) return;
    CCL.LUT0CTRLA = 0;                  /* disable before reconfiguring */

    if (!on) return;

    /* SEQ0 spans LUT0 and LUT1. LUT1 is the Hsync EDGE SENSE -- not the
     * combiner, which is LUT2 -- and it must stay combinational, so the
     * sequencer stays off. Written rather than assumed, because it is
     * enable-protected by LUT0 and this is the only moment it can be
     * written. */
    CCL.SEQCTRL0  = CCL_SEQSEL_DISABLE_gc;

    CCL.LUT0CTRLB = CCL_INSEL0_EVENTA_gc | CCL_INSEL1_TCB1_gc;
    CCL.LUT0CTRLC = CCL_INSEL2_MASK_gc;
    CCL.TRUTH0    = (uint8_t)(trigger_falling ? 0x01 : 0x02);
    CCL.LUT0CTRLA = CCL_ENABLE_bm;
}

/* Builds the whole CCL for one role, rather than editing the parts that
 * differ. Almost everything is enable-protected -- writable only while
 * the LUT is disabled -- so a partial rebuild would mean tracking which
 * registers are currently writable in which role. Tearing the array
 * down and putting it back costs a few microseconds during a mute that
 * is happening anyway, and removes that whole class of mistake.
 *
 * Interrupts are held off across it. The TWI ISR would otherwise be
 * free to run while the array is half-built; I2C hosts tolerate the
 * clock stretching that causes, and a torn Csync output is worse. */
void ccl_configure(ccl_role_t role, uint8_t pol_flip,
                   uint8_t h_pin_idle_high, uint8_t gate_open,
                   ccl_hmode_t hmode)
{
    uint8_t s = SREG;
    cli();

    /* LUT2 has no default output pin on this package -- PD6 is its only
     * position and it is behind PORTMUX. Set here, with the LUTs, and
     * unconditionally: the routing exists in every build and only OUTEN
     * decides whether the pin is driven.
     *
     * CCLROUTEA is a distinct register from EVSYSROUTEA and TCEROUTEA,
     * so this clobbers neither. */
    PORTMUX.CCLROUTEA = PORTMUX_LUT2_ALT1_gc;

    CCL.CTRLA     = 0;
    CCL.LUT1CTRLA = 0;
    CCL.LUT2CTRLA = 0;
    CCL.LUT3CTRLA = 0;
    CCL.SEQCTRL1  = CCL_SEQSEL_DISABLE_gc;
    /* LUT0 IS TORN DOWN HERE TOO, which it was not before the
     * rotation. The divider moved to SEQ0, so LUT0 is half the
     * flip-flop -- and that makes ccl_configure() a second writer of a
     * LUT that ccl_fo_lut0_build() otherwise owns.
     *
     * They cannot collide: DIVIDE and Mode FO are mutually exclusive by
     * construction. Leaving DIVIDE therefore leaves LUT0 disabled, which
     * is exactly the state fo_lut0_build(0) produces, so Mode FO finds
     * what it expects when it next engages.
     *
     * BUT ONLY WHEN DIVIDE IS ON ONE SIDE OF THE CHANGE. The first
     * version of this cleared LUT0 unconditionally, which meant every
     * ordinary role change -- PASSTHRU to RECON when an H offset comes
     * off centre, and back -- silently switched Mode FO's field delay
     * off underneath it. ccl_role still holds the OUTGOING role here,
     * so both directions are covered without needing to remember one. */
    if (role == CCL_ROLE_DIVIDE || ccl_role == CCL_ROLE_DIVIDE) {
        CCL.LUT0CTRLA = 0;
        CCL.SEQCTRL0  = CCL_SEQSEL_DISABLE_gc;
    }

    if (role == CCL_ROLE_DIVIDE) {
        /* SEQ0 as the J-K, because SEQ1's even half is the output
         * stage now. J and K both high, so every rising edge of the
         * clock toggles it.
         *
         * LUT0's IN2 is the clock and carries Hsync on EVENTB. EVENTB
         * rather than EVENTA because EVENTA is permanently Vsync for
         * Mode FO: wiring both once and choosing between them with
         * INSEL means no event user has to be rewritten when the role
         * changes, which is one less register with two writers. */
        CCL.LUT0CTRLB = CCL_INSEL0_MASK_gc | CCL_INSEL1_MASK_gc;
        CCL.LUT0CTRLC = CCL_INSEL2_EVENTB_gc;      /* Hsync, = clock */
        CCL.TRUTH0    = TRUTH_CONST_ONE;           /* J = 1 */

        CCL.LUT1CTRLB = CCL_INSEL0_MASK_gc | CCL_INSEL1_MASK_gc;
        CCL.LUT1CTRLC = CCL_INSEL2_MASK_gc;
        CCL.TRUTH1    = TRUTH_CONST_ONE;           /* K = 1 */

        CCL.SEQCTRL0  = CCL_SEQSEL_JK_gc;

        /* The combiner takes TCB1's reshaped pulse as its H input, the
         * same substitution rev 1 made at LUT1. TCB1 sits in the input
         * mux on IN1 and IN2 only, and the V input holds IN0. */
        /* TCB1 ON IN2, NOT IN1. Both are documented as reaching TCB1 --
         * INSEL value 0x0A is TCB0 on IN0 but TCB1 on IN1 and IN2 --
         * and LUTnCTRLB is one table shared by all four LUTs, so IN1
         * here should have been identical to the IN1 rev 1 used at
         * LUT1. It is not: on IN1 the reshaped pulse never arrives, on
         * IN2 it does. Measured, not explained. */
        CCL.LUT2CTRLB = CCL_INSEL0_EVENTA_gc       /* Vsync via CH4 */
                      | CCL_INSEL1_MASK_gc;

    } else if (role == CCL_ROLE_RECON) {
        /* LINK reads LUT[n+1], so LUT2 sees LUT3 -- which is where the
         * reconstruction lives now. Same trick as rev 1, one LUT along:
         * the rebuilt pulse reaches the combiner without spending a pin
         * or an event channel. */
        CCL.LUT2CTRLB = CCL_INSEL0_EVENTA_gc       /* Vsync via CH4 */
                      | CCL_INSEL1_LINK_gc;        /* LUT3: rebuilt H */
    } else {
        /* Hsync cannot reach LUT2 from a pin -- LUT2 has no input pins
         * at all on this package, PD0 is not bonded out -- so PASSTHRU
         * takes it as an event where rev 1 took it as IN1. */
        CCL.LUT2CTRLB = CCL_INSEL0_EVENTA_gc       /* Vsync via CH4 */
                      | CCL_INSEL1_EVENTB_gc;      /* Hsync via CH0 */
    }
    /* ---- LUT3 rebuilds, LUT2 combines and drives pin 12 --------- */
    if (role != CCL_ROLE_DIVIDE) {
        /* LUT3 is the reconstruction XOR. TCE0 on IN0 is WO0 and on IN1
         * is WO1, exactly as it was at LUT2 in rev 1 -- LUTnCTRLB shares
         * one INSEL table across all four LUTs, so the mapping does not
         * change with the LUT. Both emitted edges are compare matches,
         * neither is BOTTOM, which is what keeps both of them hi-res. */
        CCL.LUT3CTRLB = CCL_INSEL0_TCE0_gc         /* WO0 */
                      | CCL_INSEL1_TCE0_gc;        /* WO1 */
        CCL.LUT3CTRLC = CCL_INSEL2_MASK_gc;
        CCL.TRUTH3    = 0x06;                      /* XOR of IN0, IN1 */
        CCL.LUT3CTRLA = CCL_ENABLE_bm;             /* no OUTEN: no pin */

        /* LUT1 presents Hsync to TCE0's restart as a rising edge at the
         * sync pulse's LEADING edge -- rev 1's LUT3 job.
         *
         * It cannot be dropped in favour of restarting straight off the
         * Hsync channel: TCE0's EVACTB offers RESTART_POSEDGE,
         * RESTART_ANYEDGE and RESTART_HIGHLVL and no negative edge, so
         * with negative-going sync the pin's rising edge is the pulse's
         * TRAILING edge. And it is a LUT rather than PORT's INVEN
         * because INVEN would invert the pin for everything reading it
         * -- the combiner, both capture timers, the polarity
         * measurement. */
        CCL.LUT1CTRLB = CCL_INSEL0_EVENTA_gc | CCL_INSEL1_MASK_gc;
        CCL.LUT1CTRLC = CCL_INSEL2_MASK_gc;
        CCL.TRUTH1    = edge_sense_truth(h_pin_idle_high);
        CCL.LUT1CTRLA = CCL_ENABLE_bm;             /* no OUTEN: PC3=SCL */
    } else {
        CCL.LUT3CTRLA = 0;                         /* unused in DIVIDE */
        CCL.LUT1CTRLA = CCL_ENABLE_bm;             /* K half of SEQ0 */
        CCL.LUT0CTRLA = CCL_CLKSRC_IN2_gc | CCL_ENABLE_bm;
    }

    /* The combiner's own table. IN0 is V and IN1 is H, the same
     * assignment rev 1 used at LUT1, so comb_xor_truth()'s values carry
     * across unchanged. */
    CCL.LUT2CTRLC = (role == CCL_ROLE_DIVIDE) ? CCL_INSEL2_TCB1_gc
                                              : CCL_INSEL2_MASK_gc;
    CCL.TRUTH2    = combiner_truth(pol_flip, gate_open, role,
                                   h_pin_idle_high, hmode);
    CCL.LUT2CTRLA = CCL_OUTEN_bm | CCL_ENABLE_bm;  /* Csync on PD6 */
    CCL.CTRLA     = CCL_ENABLE_bm;

    ccl_role = role;
    SREG = s;
}

/* Mute and unmute. Rewrites the combiner's truth table, with the same
 * disable/write/enable and the same interrupts-off window as a polarity
 * change, for the same reason: the register is enable-protected and the
 * TWI ISR must not be able to lengthen the gap.
 *
 * Callers pass both the gate state and the polarity because TRUTH1
 * encodes both. Caching either here would create a second copy that
 * could disagree with the code that owns it. */
void ccl_set_gate(uint8_t gate_open, uint8_t pol_flip,
                  uint8_t h_pin_idle_high, ccl_hmode_t hmode)
{
    uint8_t s = SREG;
    cli();
    /* The combiner is LUT2, so the gate and the polarity live in
     * TRUTH2 -- and the table has to come from combiner_truth(),
     * because in DIVIDE the reshaped H is on IN2 and the entries move.
     *
     * THIS WAS THE THIRD WRITER. ccl_configure() and ccl_set_polarity()
     * were unified onto combiner_truth() and this one was missed, so
     * every mute/unmute put the IN1 table back. Every source change and
     * every polarity change mutes and unmutes, which is why Mode S came
     * up correctly on entry and collapsed to 60Hz the moment anything
     * disturbed it. Three writers, one register. */
    CCL.LUT2CTRLA &= (uint8_t)~CCL_ENABLE_bm;
    CCL.TRUTH2     = combiner_truth(pol_flip, gate_open, ccl_role,
                                    h_pin_idle_high, hmode);
    CCL.LUT2CTRLA |= CCL_ENABLE_bm;
    SREG = s;
}
