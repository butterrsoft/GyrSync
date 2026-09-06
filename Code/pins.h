#ifndef CSYNC_PINS_H
#define CSYNC_PINS_H

/* ===================================================================
 * Pin map -- AVR32EB14, SOIC14. THE AUTHORITATIVE COPY.
 * ===================================================================
 *
 * This replaces the map in main.c's header comment, which had drifted:
 * it still described pin 11 as the interlace indicator long after the
 * default became fo_active (spec v2 open item 5). One copy cannot
 * disagree with itself.
 *
 * THERE IS ONE MAP. Rev 1 -- the pre-rotation wiring, DDC on pins 4/5
 * and Csync on pin 9 -- was deleted once this map had had a full
 * regression pass on the bench. `BOARD_REV` is gone with it; a build
 * that passes -DBOARD_REV=1 now fails rather than silently producing
 * this map.
 *
 * | Pin | Port | Owner            | Function                          |
 * |-----|------|------------------|-----------------------------------|
 * |  1  | GND  | --               | --                                |
 * |  2  | PF6  | ui               | B1, active low, internal pull-up  |
 * |  3  | PF7  | --               | UPDI -- NEVER REPURPOSE           |
 * |  4  | PA0  | clock            | EXTCLK in -- GND until fitted     |
 * |  5  | PA1  | ui               | Blink LED                         |
 * |  6  | PC0  | evsys/ccl        | Vsync in -- PULLUPEN + BOTHEDGES  |
 * |  7  | PC1  | evsys/ccl        | Hsync in, TCB0 capture            |
 * |  8  | PC2  | ddc              | SDA (TWI0 client, PORTMUX ALT2)   |
 * |  9  | PC3  | ddc              | SCL                               |
 * | 10  | PD4  | video            | Mode-Valid LED                    |
 * | 11  | PD5  | --               | spare, driven low by port_init()  |
 * | 12  | PD6  | ccl              | Csync out <- LUT2 OUT, always     |
 * | 13  | PD7  | modev            | regenerated Vsync (Mode CS's V)   |
 * | 14  | VDD  | --               | --                                |
 *
 * PIN 6 IS WRITTEN ONCE, and that is load-bearing rather than tidy.
 * PULLUPEN and ISC share PINnCTRL, so a second plain write to arm the
 * edge interrupt takes the pull-up off -- which is what it did, and
 * what made the Mode-Valid LED tick with the Vsync lead off. See
 * port_init().
 *
 * PIN 12 IS THE ONE THAT HAS ACTUALLY GONE WRONG, twice -- both times
 * under the old pinout, where pin 12 was the regenerated Vsync. An
 * interlace indicator was left on it when Mode V took it, so
 * v_line_tick() raised the regenerated pulse and the stale indicator
 * flattened it microseconds later. A diagnostic then gave the same pin
 * to a LUT as a scope output, which overrides PORT silently -- a second
 * writer would not show as a conflict, it would show as the pulse
 * twitching low for a few hundred nanoseconds a frame, which is the
 * size of the thing being measured.
 *
 * So pin 12's ownership is asserted here rather than remembered. Every
 * claimant defines its symbol; claiming it twice fails the build. */

/* THIS HEADER DECIDES who owns pin 12, and that is the enforcement --
 * not an #error. An #error comparing two claims cannot help when both
 * are derived from the same switch, and preprocessor conditions cannot
 * see across translation units anyway. What is available is removing
 * the second opinion.
 *
 * Pin 12 is Csync, permanently, and Mode V is on pin 13. Nothing is
 * borrowed and nothing stands down. The three symbols that used to say
 * so -- PIN12_OWNER_CCL, PIN12_OWNER_MODEV and MODEV_YIELDS_PIN -- had
 * become constants with one reachable value, which is a false signal:
 * the next reader of `#if MODEV_YIELDS_PIN` would believe the pin was
 * still negotiated. They are gone, and so is the dead branch in main.c
 * that tested one of them. */

/* BOARD_REV IS GONE. It selected between this wiring and the one
 * before the rotation, and the old one has been deleted -- so a stale
 * command line carrying -DBOARD_REV=1 would otherwise build THIS map
 * while the person at the bench probed for the other one. Fail
 * instead. Delete this guard once no command line, script or note
 * anywhere still names it. */
#ifdef BOARD_REV
#error "BOARD_REV no longer exists -- rev 1 was deleted. Drop it from the command line."
#endif

/* GATE_VIA_PIN IS GONE TOO, for the same reason and in the same shape:
 * it drove pin 10, pin 10 is the Mode-Valid LED, so the option could
 * not be built. Guarded rather than silently ignored. */
#ifdef GATE_VIA_PIN
#error "GATE_VIA_PIN no longer exists -- pin 10 is the Mode-Valid LED. Drop it from the command line."
#endif

/* THE EVENT-OUTPUT DIAGNOSTIC IS GONE, AND ITS GUARD WITH IT.
 *
 * DIAG_EVOUT routed an arbitrary event channel to a pin. On this
 * package the only available one was EVOUTD on PD7 -- pin 13, which
 * Mode CS drives for its V output -- so diag.h carried an #error
 * forbidding exactly that combination while THIS header forced it. The
 * header order put the forcing after the test, so the #error could
 * never fire and the documented invocation built cleanly and collided.
 *
 * A guard that cannot fire is worse than no guard: it reads as
 * protection. Both the switch and the guard are removed rather than
 * resequenced, because the diagnostic had done its work and the
 * collision it caused is not worth carrying a guard for. */

/* ===================================================================
 * THE PIN MAP, as macros. Every pin the project touches is named here
 * and nowhere else.
 * ===================================================================
 *
 * Before this, pin numbers were raw PIN7_bm / EVGEN0SEL_PIN4_gc spread
 * across fifty-odd sites in four files, so moving a signal meant
 * finding all of them. That is the same shape as every ownership fault
 * this split exists to remove, and it was about to matter: the pinout
 * rotation moves five signals at once.
 *
 * A PORT-and-bit pair is not enough on its own -- the event system
 * selects a generator by pin NUMBER, and the port interrupt reads a
 * flag by bit -- so each signal carries whichever of the three forms
 * its users need, defined together so they cannot drift apart.
 *
 * TO MOVE A SIGNAL, edit only its block below. To swap two, swap their
 * blocks. Nothing outside this header names a pin.
 *
 * THE THREE PLAIN GPIOs -- the Mode-Valid LED, the Blink LED and the
 * diagnostic -- are freely interchangeable, because pins 5, 10 and 11
 * carry no peripheral any of them needs. Swapping them is swapping
 * three definitions.
 *
 * HSYNC AND VSYNC can swap with each other and go nowhere else. Both
 * must be event generators, a port has only two, and the other two
 * PORTC pins are DDC. */

/* ===================================================================
 * WHY THE MAP LOOKS LIKE THIS
 * ===================================================================
 *
 * The 20MHz oscillator has to go on pin 4, because EXTCLK exists only
 * on PA0. That single pin forces everything else: DDC has nowhere to
 * go but pins 8/9, which takes PC3 away from LUT1, and PD6 on pin 12
 * is then the only CCL output left on this package -- so Csync moves
 * there, Mode V's Vsync and Mode CS's V output move to pin 13, and the
 * LEDs shuffle.
 *
 * THE CCL ROLES ROTATE WITH IT, and they have to.
 *
 * Moving Csync to pin 12 is not a wiring change with a firmware
 * consequence, it is the same change as the CCL role rotation. Only
 * PC3 and PD6 can be CCL outputs; PC3 goes to SCL; so the output stage
 * must become LUT2, the reconstruction XOR must move to LUT3 (LINK on
 * LUT(n) reads LUT(n+1), so LUT2 can only take it from LUT3), and LUT1
 * is freed to become the Hsync edge sense.
 *
 * That last one cannot be dropped to make room. TCE0's EVACTB offers
 * RESTART_POSEDGE, RESTART_ANYEDGE and RESTART_HIGHLVL and no negative
 * edge, so a source with negative-going sync needs a LUT to present the
 * restart as a rising edge. On the OLD pinout that left five jobs for
 * four LUTs -- LUT1 had to buffer LUT2 out to pin 9 AND be the edge
 * sense -- which is why the rotation could not be tested before the
 * move, and why the two wirings were never a runtime choice.
 *
 * Mode S moves too. Its divide-by-two is a J-K flip-flop built from
 * SEQ1, the LUT2/LUT3 pair -- and LUT2 is the output stage here, so
 * the divider lives on SEQ0, the LUT0/LUT1 pair. That is available
 * because DIVIDE and RECON are mutually exclusive: role_wanted()
 * returns DIVIDE only for Mode S on a fast source and RECON only for
 * an H offset off centre, so during DIVIDE the edge sense is idle and
 * Mode FO cannot be running either. */

/* --- THE MAP. One wiring, no alternatives. --------------------- */

/* --- pin 2: B1 button ------------------------------------------- */
#define PIN_B1_PORT         PORTF
#define PIN_B1_bm           PIN6_bm
#define PIN_B1_CTRL         PIN6CTRL

/* --- pin 5: Blink LED. Freed by DDC moving to pins 8/9. ---------- */
#define PIN_BLINK_PORT      PORTA
#define PIN_BLINK_bm        PIN1_bm

/* --- pin 6: Vsync in -------------------------------------------- */
#define PIN_VSYNC_PORT      PORTC
#define PIN_VSYNC_bm        PIN0_bm
#define PIN_VSYNC_CTRL      PIN0CTRL
#define PIN_VSYNC_EVGEN     PORT_EVGEN1SEL_PIN0_gc

/* --- pin 7: Hsync in -------------------------------------------- */
#define PIN_HSYNC_PORT      PORTC
#define PIN_HSYNC_bm        PIN1_bm
#define PIN_HSYNC_CTRL      PIN1CTRL
#define PIN_HSYNC_EVGEN     PORT_EVGEN0SEL_PIN1_gc

/* --- pin 10: Mode-Valid LED. The gate that used to be driven out on
 * this pin is a choice of truth table now, which is what freed it. --- */
#define PIN_MODE_LED_PORT   PORTD
#define PIN_MODE_LED_bm     PIN4_bm

/* --- pin 11: diagnostic ----------------------------------------- */
#define PIN_DIAG_PORT       PORTD
#define PIN_DIAG_bm         PIN5_bm

/* --- pin 13: Vsync out. Mode V's regenerated Vsync and Mode CS's
 * pass/flip/block share it -- they are the same signal, "the vertical
 * sync this unit emits". Mode CS reaches it through EVOUTD with
 * PIN7CTRL.INVEN for flip, because pin 13 cannot be a CCL output.
 *
 * The loopback does not care which pin it uses -- software drives it
 * and PORTD's generator reads it back, so any plain PORTD GPIO with a
 * generator would serve. A diagnostic build once moved it to pin 11 to
 * free pin 13 for an event output. Pin 13 is the shipping position and
 * the only one now. */
#define PIN_VREGEN_PORT     PORTD
#define PIN_VREGEN_bm       PIN7_bm
#define PIN_VREGEN_CTRL     PIN7CTRL
#define PIN_VREGEN_EVGEN    PORT_EVGEN1SEL_PIN7_gc

/* Pin 4 is EXTCLK, pins 8/9 are DDC, pin 12 is Csync from CCL LUT2 --
 * all driven by a peripheral through PORTMUX, never by a PORT write.
 * No gate entry: pin 10 is the Mode-Valid LED now. */

#endif /* CSYNC_PINS_H */
