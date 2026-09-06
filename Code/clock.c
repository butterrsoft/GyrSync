/* CLKCTRL -- the PLL and the main clock. SINGLE OWNER OF CLKCTRL.
 *
 * Runs once, before anything else, and is never touched again. Split
 * out because it is the one module with no dependencies at all in
 * either direction, which makes it the honest first cut. */

#include "clock.h"

#include <avr/io.h>
#include <avr/cpufunc.h>   /* _PROTECTED_WRITE */
#include <stdint.h>

/* WHICH REFERENCE WON IS NO LONGER RECORDED, AND THAT IS A GAP RATHER
 * THAN A TIDY-UP -- read this before assuming it was an oversight.
 *
 * clk_external held it, and its only consumer was a diagnostic that put
 * the flag on pin 11. With the diagnostics gone the variable was
 * written and never read, so it went with them. But the two cases are
 * not equivalent: a fitted-but-dead oscillator and a working one both
 * produce a part running at 20MHz with the same constants, and the
 * fallback's OSCHF error is comparable to the width of Mode 4's
 * acceptance window (see F_PER_MEASURED in main.c). So a unit that has
 * fallen back may reject Mode 4 sources it should take, and there is
 * now nothing on the part that says it has.
 *
 * If that is ever wanted, the honest home for it is an indication the
 * user can see -- a distinct code in the power-on report -- rather than
 * a variable nothing reads. That is a UI decision, not a clock one, so
 * it is not made here. */

void clock_init(void)
{

    /* CLK_PER4 = 80MHz for the high-resolution extension, CLK_PER =
     * 20MHz for everything else. TCE cannot be clocked faster than the
     * peripheral clock -- the PLL feeds the hi-res extension, not the
     * counter -- so 80MHz exists solely so that TCE compare edges can
     * be placed to 6.25ns in increment 4.
     *
     * ORDER MATTERS. MCLKCTRLB is written FIRST, while CLK_MAIN is
     * still the 20MHz internal oscillator: PBDIV then makes CLK_PER
     * 5MHz, which is harmless. Writing CLKSEL first would put the whole
     * part at 80MHz for the handful of cycles until PBDIV arrived,
     * which is four times its rating.
     *
     * PEN stays 0, so prescaler A is bypassed and CLK_PER4 is CLK_MAIN
     * undivided; PBDIV alone gives the divide-by-four down to CLK_PER.
     * Note that MCLKCTRLB does NOT reset to zero -- it resets to 0x11,
     * PEN enabled with PDIV at DIV6, the familiar 20MHz/6 = 3.333MHz
     * default -- so this write is what removes that divider, not merely
     * an addition to it.
     *
     * Both registers are configuration-change protected. */
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PBDIV_bm);

    /* OSCHF runs at 20MHz (OSCCFG.OSCHFFRQ ships as 0). SOURCEDIV DIV4
     * takes it to 5MHz, inside the 2.5-5.5MHz PLL input range, and 16x
     * gives 80MHz -- the top of the 40-80MHz output range for 16x.
     *
     * RUNSTDBY IS LOAD-BEARING, and leaving it out is what stopped the
     * first silicon from running at all. With it clear the data sheet
     * is explicit that "the PLL will only run if requested by a
     * peripheral", and MCLKSTATUS agrees from the other side: its
     * status bits, SOSC excepted, "will be available only if the
     * respective source is requested as the main clock or by a
     * peripheral". So an earlier revision that configured the PLL and
     * then waited for MCLKSTATUS.PLLS before selecting it waited
     * forever -- nothing had requested the PLL yet, so it never
     * started. The part ran, but never left this function, leaving
     * every pin a high-impedance input. It looked like a dead board. */
    /* THE EXTERNAL OSCILLATOR IS TRIED FIRST, UNCONDITIONALLY -- AND UNDERSTAND WHAT FAILURE
     * COSTS BEFORE READING ANY FURTHER.
     *
     * Section 12.3.2: "When selecting an external clock source, a switch
     * to the chosen clock source will occur if a sufficient number of
     * edges are detected. If enough number of clock edges are not
     * detected, the clock source remains unchanged, and IT IS IMPOSSIBLE
     * TO CHANGE TO ANOTHER CLOCK SOURCE WITHOUT EXECUTING A RESET."
     *
     * So there is no software path back. An earlier version of this file
     * timed out correctly and then tried to write MCLKCTRLA back to
     * OSCHF; that write cannot take effect, and the wait for SOSC after
     * it never returned. Pins stayed high impedance because port_init()
     * had not run yet. It looked exactly like a dead board -- the same
     * symptom, and very nearly the same cause, as the PLLS wait
     * described above.
     *
     * The reset the data sheet names is therefore not a last resort, it
     * is the mechanism. On failure this function resets the part
     * deliberately and the next boot skips the attempt, which it knows
     * to do from RSTCTRL.RSTFR rather than from RAM -- RAM does not
     * survive, and the GPRs are documented with a reset value but not
     * with which resets clear them, which is not a thing to guess at.
     *
     * COST: on a board with no oscillator fitted, every power-up boots
     * twice and the first boot is dead for as long as the guard runs.
     * That is the price of one firmware running on both boards, and it
     * disappears the moment the part is fitted.
     *
     * THIS USED TO BE BEHIND CLOCK_EXTCLK, defaulting to 1. With the
     * oscillator fitted and the compile-time switches gone there is one
     * clock configuration, and the fallback above is a RUN-TIME answer
     * to a missing part rather than a build-time one -- which is what
     * the switch's own default was already saying. */
    uint8_t rstfr = RSTCTRL.RSTFR;
    RSTCTRL.RSTFR = rstfr;      /* flags accumulate until written back */

    if (!(rstfr & RSTCTRL_SWRF_bm)) {

        _PROTECTED_WRITE(CLKCTRL.PLLCTRLA, CLKCTRL_RUNSTDBY_bm
                                         | CLKCTRL_SOURCE_EXTCLK_gc
                                         | CLKCTRL_SOURCEDIV_DIV4_gc
                                         | CLKCTRL_MULFAC_16X_gc);
        _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_PLL_gc);

        /* SOSC CLEARING IS NOT ON ITS OWN A SUCCESS. If too few edges
         * are detected the switch may never start, in which case SOSC is
         * never set and a wait-for-clear would fall straight through and
         * report a crystal that is not there. So the test is positive:
         * the switch finished AND both the external clock and the PLL
         * report themselves stable.
         *
         * CLK_PER is 5MHz here -- CLK_MAIN is still OSCHF and PBDIV is
         * already dividing by four -- so this loop runs at a few hundred
         * kilohertz. 200000 iterations is comfortably past the 10ms
         * worst-case start-up both candidate oscillators quote. */
        uint32_t guard = 200000UL;
        uint8_t  st;
        do {
            st = CLKCTRL.MCLKSTATUS;
            if (!(st & CLKCTRL_SOSC_bm)
             && (st & CLKCTRL_EXTS_bm)
             && (st & CLKCTRL_PLLS_bm)) {
                return;                 /* running on the crystal */
            }
        } while (--guard);

        /* No way back. Reset, and let the next boot see SWRF. */
        _PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRE_bm);
        for (;;) ;                      /* the reset arrives here */
    }

    _PROTECTED_WRITE(CLKCTRL.PLLCTRLA, CLKCTRL_RUNSTDBY_bm
                                     | CLKCTRL_SOURCE_OSCHF_gc
                                     | CLKCTRL_SOURCEDIV_DIV4_gc
                                     | CLKCTRL_MULFAC_16X_gc);

    /* Selecting the PLL is itself what requests it, so the switch is
     * the trigger rather than something to be done after the PLL is
     * ready. SOSC is the flag that belongs here: it means the source
     * for CLK_MAIN "is undergoing a switch and will change as soon as
     * the new source is stable", so waiting for it to clear waits for
     * stability and the switch together. */
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_PLL_gc);
    while (CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm) ;
}
