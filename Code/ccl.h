#ifndef CSYNC_CCL_H
#define CSYNC_CCL_H

#include <stdint.h>

/* The combiner's shape, not a mode. PASSTHRU and RECON differ only in
 * where LUT1 takes its H input from; DIVIDE rebuilds LUT2/LUT3 as a JK
 * flip-flop to halve a fast source. */
typedef enum {
    CCL_ROLE_PASSTHRU,
    CCL_ROLE_DIVIDE,
    CCL_ROLE_RECON
} ccl_role_t;

/* WHAT THE COMBINER DOES WITH THE H INPUT -- Mode CS, section 17.1.
 *
 * COMBINE is the default and was the only behaviour before Mode CS:
 * Csync, H exclusive-or'd with V. FLAT is Csync with Vsync overriding H
 * rather than serrating it. PASS and FLIP drop V from the function
 * entirely and put H alone on pin 12, upright or inverted, so the V
 * output can be taken separately on pin 13.
 *
 * Deliberately NOT the method number. The seven methods pair an H
 * behaviour with a V behaviour, and the V half is arbitrated against
 * Mode V's use of pin 13 -- which is main.c's business and nothing to
 * do with a truth table. ccl.c is told what to do with H and is not
 * given the vocabulary to be told anything else. */
typedef enum {
    CCL_H_COMBINE = 0,   /* Csync, serrated through the Vsync interval */
    CCL_H_FLAT    = 1,   /* Csync, Vsync overriding H rather than
                          * serrating it -- the "AND logic" form */
    CCL_H_PASS    = 2,
    CCL_H_FLIP    = 3
} ccl_hmode_t;

/* Builds the whole array for one role. pol_flip, h_pin_idle_high and
 * hmode are PASSED, not read: this module does not get to hold an
 * opinion about the source's sense -- or about which combine method is
 * selected -- that differs from the code that owns it. */
void ccl_configure(ccl_role_t role, uint8_t pol_flip,
                   uint8_t h_pin_idle_high, uint8_t gate_open,
                   ccl_hmode_t hmode);

/* Rewrites the combiner's TRUTH2, and the edge sense's TRUTH1 when the
 * role uses it, without rebuilding the array. Both are enable-protected,
 * so this disables and re-enables the LUT with interrupts held off --
 * Csync is interrupted for the duration, which is why the caller mutes
 * around it. */
void ccl_set_polarity(uint8_t pol_flip, uint8_t h_pin_idle_high,
                      uint8_t gate_open, ccl_hmode_t hmode);

/* Mute and unmute Csync. The gate is a choice of TRUTH TABLE, not an
 * input: this rewrites the combiner's TRUTH2, which briefly releases
 * the output because the register is enable-protected. The pin version
 * that made muting purely combinational needed a pin this wiring does
 * not have, and is gone.
 *
 * h_pin_idle_high and hmode joined the signature with Mode CS, and they
 * are not optional. This is the THIRD WRITER of TRUTH2 -- the one that
 * was missed when the IN2 table landed -- so it has to be handed
 * everything the table is built from. A default here would be a second
 * opinion by another name. */
void ccl_set_gate(uint8_t gate_open, uint8_t pol_flip,
                  uint8_t h_pin_idle_high, ccl_hmode_t hmode);

/* LUT0, Mode FO's Vsync conditioner. Separate from ccl_configure()
 * because it engages and disengages independently of the role. */
void ccl_fo_lut0_build(uint8_t on, uint8_t trigger_falling);

/* Which role is installed. Read-only by design: the role changes only
 * as a result of ccl_configure() actually rebuilding the array, so a
 * caller cannot record an intention that the hardware did not carry
 * out. That pairing is the whole reason it moved here. */
ccl_role_t ccl_current_role(void);

#endif
