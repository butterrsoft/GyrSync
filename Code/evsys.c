/* EVSYS -- SINGLE OWNER OF EVERY CHANNEL AND EVERY USER REGISTER.
 *
 * Before the split, EVSYS was written from three places: evsys_init(),
 * v_route_apply() and tcb1_set_job(). The first two were deliberate and
 * documented; the third was not, and USERTCB1CAPT being moved from
 * inside the TCB1 job table is exactly the shape of the CHANNEL4/5
 * fault that v_route_apply() was written to prevent.
 *
 * Nothing outside this file names EVSYS now. PORTMUX.EVSYSROUTEA is
 * here too: it is a distinct register from CCLROUTEA and TCEROUTEA,
 * and the only reason to touch it is EVOUT, which is an event concern.
 *
 * The channel map is in evsys.h so a reader can see all six at once
 * rather than reconstructing them from the writes below. */

#include "pins.h"
#include "evsys.h"

#include <avr/io.h>

/* ONE WRITER FOR THE VSYNC ROUTING, because there are now two things
 * that can change it and they must not each write the channel from
 * their own corner.
 *
 *   CHANNEL5  the RAW Vsync -- pin 6, or pin 12 when Mode V regenerates.
 *             Feeds LUT0's IN0 and TCB1's one-shot trigger.
 *   CHANNEL4  what the COMBINER sees. LUT0's delayed copy when Mode FO
 *             is engaged, otherwise the raw signal directly.
 *
 * CHANNEL5 is written whether or not Mode FO is engaged. It costs
 * nothing when nothing is listening, and the alternative -- writing it
 * only on engagement -- is the "routing built but never connected"
 * mistake with the producer and consumer swapped. */
void ev_vsync_route(uint8_t regen, uint8_t fo_active, uint8_t v_out)
{
    uint8_t src = regen ? EVSYS_CHANNEL_PORTD_EV1_gc
                          : EVSYS_CHANNEL_PORTC_EV1_gc;

    /* NOTHING TO DO IS NOT THE SAME AS DOING IT AGAIN.
     *
     * This function is destructive by construction: it tears EVOUTD
     * down before moving the channels and restores it after, so calling
     * it with unchanged arguments does NOT leave the pin unchanged --
     * it drops the event output for the dozen cycles in between and
     * hands pin 13 back to PORT.
     *
     * polarity_measure() calls it four times a second through
     * v_pin_apply(), and on a NEGATIVE-going Vsync -- pin idling high
     * under EVOUT -- that dropout is a half-microsecond notch to 0V,
     * four times a second, on the Vsync a monitor is trying to lock to.
     * The picture drops out; a positive-going source is untouched
     * because its idle level is where PORT was going to put it anyway.
     * That is the polarity asymmetry seen on the bench, and it is the
     * reason this early-out exists.
     *
     * Caching state about registers this file owns is normally the
     * wrong instinct and this file says so elsewhere. The distinction
     * is that the cache here does not describe what the hardware holds
     * -- it records the arguments of the last APPLIED call, so that a
     * destructive operation is not performed for a change that is not
     * one. A sentinel that no real argument set can produce forces the
     * first call through. */
    static uint8_t last = 0xFF;
    uint8_t want = (uint8_t)((regen ? 1 : 0)
                           | (fo_active ? 2 : 0)
                           | (v_out ? 4 : 0));
    if (want == last) return;
    last = want;

    /* TEAR DOWN FIRST. Leaving EVOUTD enabled across the switch to
     * PORTD_EV1 closes a latch that no later write can open, because
     * the pin would then be driving its own input. */
    EVSYS.USEREVSYSEVOUTD = EVSYS_USER_OFF_gc;

    EVSYS.CHANNEL5 = src;
    EVSYS.CHANNEL4 = fo_active ? EVSYS_CHANNEL_CCL_LUT0_gc : src;

    /* CHANNEL5 RATHER THAN CHANNEL4, and the difference matters on a
     * Mode FO source. CHANNEL4 is what the COMBINER sees, which under
     * Mode FO is LUT0's conditioned copy -- delayed, and normalised to
     * a positive-going pulse whatever the source sent. CHANNEL5 is the
     * Vsync pin itself. Pin 13 is meant to carry the source's Vsync,
     * not the combiner's working copy of it, so it takes the raw one
     * and the polarity is then PORT's business via INVEN.
     *
     * The regen case never reaches here: when Mode V is regenerating,
     * PORT is already driving pin 13 with the rebuilt pulse and that IS
     * the V output. Nothing to route. */
    if (v_out && !regen)
        EVSYS.USEREVSYSEVOUTD = EVSYS_USER_CHANNEL5_gc;
}


/* TCB1'S CAPTURE SOURCE -- the only event user tcb1 is allowed to move,
 * and it moves it by asking rather than by writing.
 *
 * USERTCB1CAPT used to be written from three places inside
 * tcb1_set_job(), which is how EVSYS came to have three writers in a
 * file whose comments insisted it had one. The job table stays in
 * tcb1's own code; the register does not. */
void ev_tcb1_capture_source(ev_tcb1_src_t s)
{
    switch (s) {
    case EV_TCB1_DIVIDED_HSYNC:
        EVSYS.USERTCB1CAPT = EVSYS_USER_CHANNEL2_gc; break;
    case EV_TCB1_HSYNC:
        EVSYS.USERTCB1CAPT = EVSYS_USER_CHANNEL0_gc; break;
    case EV_TCB1_VSYNC_RAW:
        EVSYS.USERTCB1CAPT = EVSYS_USER_CHANNEL5_gc; break;
    }
}

void evsys_init(void)
{
    /* CCLROUTEA is NOT here. It is a CCL concern and ccl_configure()
     * owns it -- this file owns EVSYSROUTEA only. Splitting the two
     * mattered: the header of ccl.c claimed CCLROUTEA while the write
     * was still in this function, which is a comment asserting an
     * ownership the code did not have. */

    /* Each port offers two event generators, each selecting one pin.
     * PORTC's two are spoken for by the syncs; PORTD's first carries
     * the mode-valid gate back into the CCL. */
    PIN_HSYNC_PORT.EVGENCTRLA = PIN_HSYNC_EVGEN   /* Hsync -> EVGEN0 */
                              | PIN_VSYNC_EVGEN;  /* Vsync -> EVGEN1 */
    /* PORTD's first generator is free: the mode-valid gate stopped
     * needing a pin when it became a choice of truth table. */
    PIN_VREGEN_PORT.EVGENCTRLA = PIN_VREGEN_EVGEN;   /* regen'd Vsync */

    EVSYS.CHANNEL0 = EVSYS_CHANNEL_PORTC_EV0_gc;   /* Hsync */
    /* No gate channel: pin 10 is the Mode-Valid LED.
     *
     * CHANNEL2 carries the divider, which appears at LUT0-OUT because
     * the sequencer sits in the EVEN LUT's output path and the pair is
     * SEQ0. */
    EVSYS.CHANNEL2 = EVSYS_CHANNEL_CCL_LUT0_gc;    /* divided Hsync */
    /* Vsync reaches LUT1 through a channel rather than straight off the
     * pin, so Mode V can substitute a regenerated copy and Mode FO can
     * interpose LUT0's delayed copy, each by changing ONE register.
     * Everything downstream -- the combiner, the polarity choice, the
     * mute -- is unaware which source it is looking at.
     *
     * CHANNEL5 carries the RAW signal to LUT0 and to TCB1's one-shot
     * trigger; CHANNEL4 carries whatever the combiner should see. Both
     * are written together by v_route_apply(), which owns them. */
    EVSYS.CHANNEL4     = EVSYS_CHANNEL_PORTC_EV1_gc;   /* the real one */
    EVSYS.CHANNEL5     = EVSYS_CHANNEL_PORTC_EV1_gc;
    /* The combiner is LUT2, so the Vsync the picture is built from goes
     * there; LUT1's EVENTA is Hsync, for the restart edge sense.
     *
     * LUT0 gets BOTH: EVENTA is Vsync for Mode FO, EVENTB is Hsync for
     * the divider's clock. Wiring both once and choosing with INSEL
     * means no event user is rewritten when the role changes. */
    EVSYS.USERCCLLUT2A = EVSYS_USER_CHANNEL4_gc;   /* Vsync, combiner */
    EVSYS.USERCCLLUT2B = EVSYS_USER_CHANNEL0_gc;   /* Hsync, PASSTHRU */
    EVSYS.USERCCLLUT1A = EVSYS_USER_CHANNEL0_gc;   /* Hsync, edge sense */
    EVSYS.USERCCLLUT0A = EVSYS_USER_CHANNEL5_gc;   /* Vsync, Mode FO */
    EVSYS.USERCCLLUT0B = EVSYS_USER_CHANNEL0_gc;   /* Hsync, JK clock */

    /* LUT1's INSEL0 must be EVENTA for any of this to matter -- see
     * ccl_configure -- and LUT0's INSEL0 likewise, which fo_lut0_build
     * sets when it enables the LUT. Routing a channel to a LUT input the
     * LUT is not listening to costs nothing and does nothing, which is
     * exactly what happened: the substitution was built, wired, and
     * never connected, so the combiner went on reading the pin while the
     * polarity was forced as though it were not. */

    EVSYS.USERTCB0CAPT = EVSYS_USER_CHANNEL0_gc;

    /* CHANNEL3 CARRIES LUT1'S OUTPUT TO TCE0'S RESTART.
     *
     * LUT1 is the edge sense. TCE0's EVACTB offers RESTART_POSEDGE,
     * RESTART_ANYEDGE and RESTART_HIGHLVL and no negative edge, so with
     * negative-going sync the pin's rising edge is the pulse's TRAILING
     * edge -- and restarting from that puts the counter's zero one sync
     * width past the edge the picture is positioned by. LUT1 inverts
     * when the source needs it so the restart lands on the leading edge
     * and the offsets stay within their own line.
     *
     * The inversion is done in a LUT rather than with PORT's INVEN
     * because INVEN would invert the pin for everything reading it --
     * the combiner, both capture timers and the polarity measurement.
     *
     * LUT1 already takes Hsync from its EVENTA above, so there is no
     * extra event USER here -- only the channel carrying its output out
     * to the restart. */
    /* EVOUTD TO PD7, UNCONDITIONALLY AND ONCE.
     *
     * Its DEFAULT position is PD2, which the SOIC-14 does not bond out.
     * So an EVOUTD connected to a channel without this write drives an
     * unbonded pad, PD7 is left to PORT, and pin 13 reads as a flat
     * level -- which is precisely how Mode CS's pass and flip failed on
     * the bench: 0V in methods 2 to 4 and 5V in 5 and 6, the signature
     * of PORT driving OUT=0 with INVEN doing the only visible work.
     *
     * It lived inside a diagnostic switch until then, so production had
     * never needed it and never made it. Now that a shipping feature
     * routes an event to that pin, the write belongs here in the
     * unconditional path -- written once, never changed, one owner. It
     * is also the last thing that switch was good for, and the switch
     * is gone. The bit selects
     * WHICH pin EVOUTD appears on; USEREVSYSEVOUTD is what decides
     * whether anything is driven, so setting this at init commits
     * nothing and costs one store.
     *
     * Wholesale rather than read-modify-write: EVOUTA through EVOUTF
     * each have their own field in this register and every one of them
     * wants its default except EVOUTD. Writing the group constant says
     * that, and leaves no partial state for a second writer to
     * inherit. */
    PORTMUX.EVSYSROUTEA = PORTMUX_EVOUTD_ALT1_gc;      /* pin 13 */

    EVSYS.CHANNEL3     = EVSYS_CHANNEL_CCL_LUT1_gc;
    EVSYS.USERTCE0CNTB = EVSYS_USER_CHANNEL3_gc;
    EVSYS.USERTCB1CAPT = EVSYS_USER_CHANNEL2_gc;
}
