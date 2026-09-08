#ifndef CSYNC_CLOCK_H
#define CSYNC_CLOCK_H

/* PLL to 80MHz CLK_PER4, 20MHz CLK_PER. One count = 50ns.
 * Call first: everything else assumes CLK_PER is already 20MHz. */
void clock_init(void);

/* clock_init() tries the external oscillator on PA0 first and falls
 * back to OSCHF, via a deliberate reset, if it does not start. Which
 * one won is not reported -- see the note in clock.c. */

#endif
