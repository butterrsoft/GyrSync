# CSync dongle -- AVR32EB14 build + flash
#
# Flashing uses avrdude's "serialupdi" programmer, which bit-bangs UPDI
# over a plain USB-serial adapter (the CH340), exactly as for the
# ATtiny824. Unlike that part, UPDI here is on its own dedicated pin
# (PF7, pin 3), so there is no conflict with using RESET as a button and
# no way to lose programming access through the reset-pin fuse.

# Overridable so switching between the EB14 as target and the EB20 as
# fallback is a command-line argument rather than an edit:
#   make MCU=avr32eb20 flash PORT=COM8
MCU     ?= avr32eb14
F_CPU   = 20000000UL
TARGET  = main

# Leave TOOLDIR empty to resolve avr-gcc/avr-objcopy/avr-size from PATH.
# AVR-EB support needs a recent toolchain -- avr-gcc 16.1.0 is known
# good, and older MPLAB/Arduino installs on PATH are the usual reason a
# build fails with "unknown MCU". Point TOOLDIR at the bin/ folder of a
# toolchain that does support it to bypass PATH entirely:
#   make TOOLDIR="E:/Programs/AVR32EB/avr-gcc-16.1.0-x64-windows/bin/"
# Trailing slash matters -- it is concatenated directly below.
TOOLDIR ?=

CC      = $(TOOLDIR)avr-gcc
OBJCOPY = $(TOOLDIR)avr-objcopy
SIZE    = $(TOOLDIR)avr-size
OBJDUMP = $(TOOLDIR)avr-objdump

# ONE BUILD, NO SWITCHES. There is no EXTRA and there are no diagnostic
# flags. Every DIAG_* switch, the EDID_FORCE_ON scaffolding, diag.h
# itself and the alternative targets that built different images have
# all been removed.
#
# They earned their removal rather than merely outliving their use. The
# last of them shipped a guard that could not fire: diag.h refused
# DIAG_EVOUT on PD7 because Mode CS drives that pin, and pins.h then
# forced DIAG_EVOUT_PORT to 'D' AFTER the refusal had already been
# tested -- so the invocation documented right here built cleanly and
# put a diagnostic on the V output. And `make hoff` had lost the code it
# named: HOFFSET_TEST appeared nowhere in the source, so the target
# flashed a byte-identical production image while its own comment
# described a B1-driven delay sweep.
#
# Both are the silently-wrong-measurement failure that FORCE and the
# build banner were added to prevent, arriving by a route neither
# covered. The protection now is that there is nothing to select. If a
# diagnostic is wanted again, add it, use it, and take it out with the
# question it answered.
#
# BOARD_REV IS GONE TOO, and was already. pins.h still #errors on it, in
# case a local script or note anywhere still names it.
#
# -fshort-enums IS NOT COSMETIC AND IS NOT OPTIONAL HERE.
#
# This toolchain does NOT default to it -- verified, a three-value enum
# occupies two bytes without it. Six enums are affected, and
# ccl_configure(), ccl_set_polarity(), ccl_set_gate() and
# combiner_truth() all take ccl_role_t or ccl_hmode_t as arguments, so
# the cost is paid at every call as well as in storage. Measured at 262
# bytes of Flash and 3 of RAM.
#
# It is an ABI flag, so it must apply to EVERY translation unit. That is
# safe here because all five are compiled in ONE invocation below, and
# no avr-libc interface this firmware touches takes an enum. If a
# pre-compiled library is ever linked in, this is the flag to check
# first.
CFLAGS  = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall -Wextra -std=gnu11 \
          -fshort-enums

# Override on the command line to match your system, e.g.:
#   make flash PORT=COM8          (Windows)
#   make flash PORT=/dev/ttyUSB0  (Linux, typical default)
PORT    ?= /dev/ttyUSB0
BAUD    ?= 115200

#   make flash AVRDUDE="E:/Programs/AVR32EB/avr-gcc-16.1.0-x64-windows/bin/avrdude.exe" ...
AVRDUDE ?= avrdude

# When avrdude is invoked by absolute path rather than found on PATH it
# sometimes cannot locate avrdude.conf. Point this at the file (usually
# alongside the executable) if that happens; it expands to nothing and
# is harmless when left empty:
#   make flash AVRDUDE_CONF="E:/Programs/AVR32EB/avr-gcc-16.1.0-x64-windows/bin/avrdude.conf"
AVRDUDE_CONF ?=
ifneq ($(strip $(AVRDUDE_CONF)),)
AVRDUDE_C = -C "$(AVRDUDE_CONF)"
endif

AVRDUDE_FLAGS = $(AVRDUDE_C) -c serialupdi -p $(MCU) -P $(PORT) -b $(BAUD)

all: $(TARGET).hex

# EVERY BUILD RECOMPILES, AND THAT IS DELIBERATE.
#
# This was added because EXTRA is not a file, so make could not see it
# change and two successive builds with different flags compiled once
# and flashed the same image twice. EXTRA is gone and make's file
# dependencies are honest again, so this is no longer load-bearing.
#
# It stays anyway. The cost is recompiling five translation units --
# about a second -- and what it buys is that a stale object file can
# never reach the bench. Every expensive mistake this project has made
# was a measurement of an image that was not the one being described,
# and a second per build is not a trade worth thinking about twice.
.PHONY: FORCE
FORCE:

# MULTI-FILE FROM HERE, AND -flto IS NOT OPTIONAL.
#
# main.c was one 4800-line translation unit, so every call was a
# candidate for inlining and the level-1 capture ISR in particular got
# its callee folded in. Splitting the file without LTO would put real
# calls back into that path -- the one interrupt in this design that
# cannot be late. LTO lets the compiler see across the modules again, so
# the split is a change to where the SOURCE lives and not to what the
# part runs.
#
# The consequence for verification: `make lst` output is no longer a
# straight translation of one file, so the Rev. A0 write-loss check --
# that writes below address 0x40 compile to OUT rather than ST -- has to
# be re-read after any change here, not assumed to survive it.
SRCS = $(TARGET).c tick.c clock.c evsys.c ccl.c
HDRS = pins.h tick.h clock.h evsys.h ccl.h

# THE SIX EDIDs ARE BUILD INPUTS, AND THEY ARE THE ONLY EDID INPUT.
#
# edid_data.h is deleted. main.c pulls these straight into .rodata with
# .incbin, so an edited image is dropped in under its existing name and
# `make flash` serves it -- no generator step, no header to regenerate.
#
# THE NAMES ARE FIXED. They are written into the .incbin block in
# main.c; a renamed file fails the build with `file not found`. The
# trailing character is the MODE LABEL, not an index: 1..5 and S.
#
# EACH MUST BE 128 OR 256 BYTES, enforced by name at assembly time. A
# 128-byte file is padded with the CEA block the shipped images carry.
# Nothing checks the CONTENT: the firmware recomputes both checksums on
# every rebuild, so a wrong file is served looking valid.
#
# Listed as prerequisites so `make` names the missing one itself rather
# than leaving it to the assembler, and so an edited image rebuilds --
# which FORCE below already guarantees, but this is the honest reason.
EDIDS = EDID-Mode1.bin EDID-Mode2.bin EDID-Mode3.bin \
        EDID-Mode4.bin EDID-Mode5.bin EDID-ModeS.bin

# eeprom_blank.bin is NOT a build input -- nothing compiles against it.
# It is a programming input, consumed by the flash target only, so it is
# deliberately not in HDRS or EDIDS, and a missing copy breaks
# `make flash` rather than `make`. It is still part of the delivered file
# set: EIGHTEEN files -- ten sources, this Makefile, six EDID images and
# the blank.

CFLAGS += -flto
LDFLAGS = -flto -Os -mmcu=$(MCU)

# .incbin resolves its argument against the ASSEMBLER's working
# directory, which is make's, not the directory main.c lives in. So the
# .bin files must sit beside the Makefile, and building from elsewhere
# with `make -C` or -f will not find them.
$(TARGET).elf: $(SRCS) $(HDRS) $(EDIDS) FORCE
	"$(CC)" $(CFLAGS) $(LDFLAGS) -o $@ $(SRCS)

$(TARGET).hex: $(TARGET).elf
	"$(OBJCOPY)" -O ihex -R .eeprom $< $@
	@$(SIZECMD)

# avr-size's Berkeley output is misleading on AVR: its "data" column
# adds .eeprom to .data, so its "dec" total counts the EEPROM image as
# though it were Flash. The sysv format lists sections individually,
# which is what you actually want to track:
#
#   Flash  = .text + .rodata + .data
#   RAM    = .data + .bss
#
# THE RAM FIGURE WAS WRONG UNTIL NOW: 4096, where the AVR32EB14 has
# 3072. The device header agrees -- INTERNAL_SRAM_SIZE is 3072 -- so
# only this report was ever mistaken and nothing was ever mislinked.
# It under-stated utilisation by a quarter, which did not matter at
# 263 bytes and would have mattered once Mode ED starts shadowing an
# EDID block in RAM.
#   EEPROM = .eeprom
#
# .rodata CARRIES THE SIX EDID IMAGES: 1536 bytes of the Flash figure,
# against 605 for the base+patch scheme it replaced. If that line ever
# moves by 256, an image has been added or lost.
#
# HELD IN A VARIABLE AND PRINTED BY EVERY BUILD, not just by `make
# size`, so the byte count always comes out of the same invocation that
# produced the hex. The separate target stays for reading the number
# without flashing.
SIZECMD = "$(SIZE)" -A $(TARGET).elf | awk '\
	  /^\.text/    {t=$$2} \
	  /^\.rodata/  {r=$$2} \
	  /^\.data/    {d=$$2} \
	  /^\.bss/     {b=$$2} \
	  /^\.eeprom/  {e=$$2} \
	  END { \
	    printf "Flash : %6d of 32768 bytes (%.1f%%)\n", t+r+d, (t+r+d)*100/32768; \
	    printf "RAM   : %6d of  3072 bytes (%.1f%%)\n", d+b, (d+b)*100/3072; \
	    printf "EEPROM: %6d of   512 bytes (%.1f%%)\n", e, e*100/512; \
	  }'

size: $(TARGET).elf
	@$(SIZECMD)

# Disassembly, kept as a target rather than a one-off command because
# two things in this design are verified by reading it rather than by
# testing: that writes below address 0x40 (SREG restores, and anything
# that reached for VPORT) compile to OUT rather than ST, which is the
# documented work-around for the Rev. A0 write-loss erratum, and that no
# VPORT access crept in at all.
lst: $(TARGET).elf
	"$(OBJDUMP)" -d -S $< > $(TARGET).lst
	@echo "wrote $(TARGET).lst"

# WHAT THE EEPROM DOES ACROSS A REFLASH -- read this before changing it.
#
# THE CHIP ERASE DOES NOT CLEAR THE EEPROM. Measured on the bench: a
# programming run with EESAVE CLEAR left the contents intact. That is
# the opposite of what this file asserted for several revisions, and the
# reversal is why the blank is now written explicitly by the flash
# target below rather than left to the erase.
#
# The claim that replaced: that avrdude's UPDI erase goes through the
# ASI chip-erase key, which must clear everything or it would be a way
# round the lock, and so ignores EESAVE. Plausible, and it was believed
# because a dump came back full of 0xff -- but those were bytes no code
# had ever written, not bytes an erase had cleared. One observation, two
# stories, and the wrong one was taken. See the flash target.
#
# WHAT THIS MEANS FOR EESAVE: with the erase not touching EEPROM, the
# fuse has nothing to protect it from, which is part of why SYSCFG0 is
# back at its shipping 0xD0. The blank file does the wiping now, on
# purpose and visibly.
#
# ONE THING TRIED AND REJECTED, KEPT BECAUSE IT MUST NOT COME BACK:
#
#    -D, to suppress the erase. THIS DOES NOT WORK.
#    It suppresses the erase and then issues a plain page WRITE,
#    not the erase-and-write it was assumed to do. Flash programming can
#    only clear bits, so every bit that needed to go 0 -> 1 stayed 0 and
#    verification failed:
#
#        device 0xe1 != input 0xe3 at addr 0x008e
#
#    0xe3 wanted, 0xe1 found: bit 1 missing. That is the signature, and
#    it leaves the part carrying a corrupt image that will not run.
#    Recover by flashing again: the erase clears the bad image.
#
# `make flash` IS THE WHOLE PROGRAMMING OPERATION. Fuses, firmware and a
# blanked EEPROM, in that order, from one command. Nobody has to
# remember a second step, and a unit cannot leave the bench with the
# firmware updated and the fuses still at their factory values -- which
# is exactly what happened before this was folded in, and cost two bench
# rounds chasing a brown-out fault on a part whose brown-out detector
# had never been enabled.
#
# ONE avrdude INVOCATION, ONE UPDI SESSION. The -U operations run in the
# order written and avrdude ABORTS ON THE FIRST FAILURE, so a fuse write
# that does not take means the flash is never written -- the part keeps
# the image it had rather than gaining new firmware on factory fuses.
# This was three separate invocations; make stopped between them and got
# the same result, but three sessions cost three UPDI handshakes and the
# ordering guarantee lived in make rather than where the operations are.
#
# THE ORDER IS FUSES FIRST, then flash, then EEPROM. Fuses survive the
# chip erase, so writing them first is safe; the EEPROM blank goes last
# because the erase that accompanies the flash write must not follow it.
# `fuses` still exists as its own target for re-writing them alone; both
# use FUSE_WRITE so the values cannot drift apart.
#
# NO `-` PREFIXES ANYWHERE. Every operation must succeed or the run
# stops. The previous version prefixed both EEPROM steps, so a failed
# read left a stale eeprom_backup.bin and the restore then wrote THAT --
# a unit could silently inherit an older snapshot, or another board's
# settings, with nothing in the output to say so.
#
# ------------------------------------------------------------------
# THE EEPROM IS BLANKED EXPLICITLY, BECAUSE THE CHIP ERASE DOES NOT DO
# IT. This is a correction: the comment that used to sit here claimed
# avrdude's UPDI erase ignores EESAVE and clears everything, and cited
# a dump full of 0xff as evidence. That was wrong. MEASURED ON THE
# BENCH: a `make flash` with EESAVE clear left the EEPROM contents
# intact. The 0xff bytes in that dump were bytes no code had ever
# written, not bytes an erase had cleared -- the same observation
# supports both stories, and the wrong one was picked.
#
# So the blank is written as a file, and nothing is inferred:
#
#   eeprom_blank.bin -- 512 bytes of 0xff, the whole EEPROM
#
# WHY 0xff AND NOT AN IMAGE OF THE DEFAULTS. A defaults image would have
# to be regenerated whenever a default or the layout moved, and would
# then be a second place where the defaults live. Blank needs no
# maintenance: byte 0 is not MODE_MAGIC, so load_settings() takes its
# fresh-part branch, and every loader below range-checks and treats an
# erased byte as "use the default". One representation, in main.c.
#
# WHAT A BLANKED PART COMES UP AS, checked against the initialisers
# rather than assumed:
#
#   Mode 1                current_mode  = MODE_1
#   Profile 1             current_profile = PROFILE_DEFAULT
#   button LOCKED         set explicitly by the fresh-part branch
#   EDID off              set explicitly by the fresh-part branch
#   Csync method 1        cs_method = CS_METHOD_DEFAULT
#   H offset centre       h_offset = H_OFFSET_DEFAULT
#   V offset centre       v_offset = V_OFFSET_DEFAULT
#   FO offset centre      f_offset = F_OFFSET_DEFAULT
#   Mode ED centre        ed_pos, ed_size = 0
#
# That is a factory reset in everything but name, and it is what every
# programming run now produces.
#
# CONFIRM IT with `make readeeprom`: every byte 0xff.
EEPROM_BLANK ?= eeprom_blank.bin

#
# THE BLANK IS A PREREQUISITE so make names it itself if it is missing,
# rather than letting avrdude fail three steps in with the fuses already
# written. It is a prerequisite of `flash` and NOT of `$(TARGET).elf`,
# which keeps the intent above: a missing copy breaks `make flash` and
# leaves `make` alone.
flash: $(TARGET).hex $(EEPROM_BLANK)
	"$(AVRDUDE)" $(AVRDUDE_FLAGS) \
	  $(FUSE_WRITE) \
	  -U flash:w:$(TARGET).hex:i \
	  -U eeprom:w:$(EEPROM_BLANK):r

# ---------------------------------------------------------------------
# Fuses.
#
# Only BODCFG is written. Every other fuse ships at a value this design
# wants, and two of them are actively dangerous to touch -- see below.
#
# BODCFG SHIPS AS 0x00 -- BROWN-OUT DETECTOR COMPLETELY DISABLED -- AND
# A PART THAT HAS NEVER HAD `make fuses` RUN ON IT IS STILL LIKE THAT.
# Confirmed on a unit in the field: bodcfg read back 0x00. Check with
# `make readfuses` before believing any value written here.
#
#   0x64 = LVL BODLEVEL3 (4.30V), ACTIVE continuous   <-- shipping
#   0x44 = LVL BODLEVEL2 (2.60V), ACTIVE continuous
#   0x24 = LVL BODLEVEL1 (1.90V), ACTIVE continuous
#
# Encoding checked against the fuse map: LVL[2:0] at bits 7:5, SAMPFREQ
# at bit 4, ACTIVE[1:0] at bits 3:2, SLEEP[1:0] at bits 1:0. BODLEVEL3
# is LVL 0x3, ACTIVE continuous is 0x1, so 0110 0100.
#
# 4.30V, AND THE ARGUMENT THAT USED TO SIT HERE FOR 2.60V WAS WRONG.
#
# It ran: the supply is a port's 5V rail less a Schottky drop, about
# 4.7V typical and as low as ~4.45V at the bottom of the USB tolerance,
# which leaves no margin over 4.30V and buys nuisance resets instead of
# stability.
#
# The hole in that is the data sheet's clock-frequency-versus-VDD table:
#
#     0 - 5 MHz    VDD >= 1.8V
#     0 - 10 MHz   VDD >= 2.7V
#     0 - 20 MHz   VDD >= 4.5V
#
# and the PLL's 80MHz CLK_PER4 carries the same 4.5V requirement. This
# design runs both. So if the rail really can reach 4.45V, THE PART IS
# ALREADY OUT OF SPECIFICATION at that voltage, BOD or no BOD. The old
# reasoning was not an argument that 2.60V is safe; it was an argument
# that the supply is marginal, and it used the BOD setting to avoid
# noticing.
#
# 2.60V also leaves a 1.9V-wide band, 2.60V to 4.50V, in which the BOD
# is satisfied and the part is executing 20MHz and 80MHz out of spec.
# That band is where the parasitic-supply failure lives: with the
# dongle's own supply removed but sync still driven, current through the
# input clamp diodes holds VDD near 3V -- enough to light an LED at half
# brightness and to keep the CPU running on garbage. At 4.30V the part
# is simply held in reset for the whole of that state, and comes up
# cleanly when the supply returns. That is why the fix needs no series
# resistors on the sync lines.
#
# IF 4.30V PRODUCES NUISANCE RESETS, the answer is not to lower it back.
# It is that the supply is not delivering what a 20MHz part requires,
# and the Schottky or the feed needs attention.
#
# ACTIVE = 0x3 (ENABLEWAIT) is also available -- continuous, with code
# execution halted until the BOD is ready -- which would be 0x6C. It
# closes the window between reset release and the BOD arming. Not taken
# here, because every value this file has ever recommended used ACTIVE =
# 0x1 and changing two things at once would make a nuisance-reset result
# ambiguous. Worth revisiting once 0x64 has been on the bench.
BODCFG ?= 0x64

# SYSCFG0 -- WRITTEN TO ITS SHIPPING VALUE, AND READ THIS BEFORE
# RUNNING IT.
#
# It ships as 0xD0 and stays 0xD0. It is still written rather than left
# alone, so `make fuses` leaves every fuse this design cares about at a
# known value rather than at whatever the part happened to arrive with:
#
#   0xD0 = 1101 0000
#          ||   |||
#          ||   ||+- EESAVE     0  EEPROM ERASED by a chip erase  <-- deliberate
#          ||   |+-- BROWSAVE   0  boot row erased by a chip erase
#          ||   +--- (reserved) 0
#          |+------- RSTPINCFG  0  INPUT: PF6 is a plain input, which is
#          |                       what B1 needs. No fuse change required.
#          +-------- UPDIPINCFG 1  PF7 is UPDI.
#        CRCSRC 0b11 = NOCRC, CRCSEL 0. CRC must stay off: the Rev. A0
#        errata make a CRC scan of part of the Flash non-functional, and
#        enabling CRC by fuse also constrains BOOTSIZE/CODESIZE.
#
# BIT 4 IS UPDIPINCFG AND MUST STAY 1. Writing a 0 there turns PF7 into a
# GPIO and the only way back is a 12V programmer. Do not hand-edit this
# value without checking that bit.
#
# EESAVE IS NOW DELIBERATELY CLEAR, WHICH IS A REVERSAL. It was set so
# that settings survived a reflash. The bench wants the opposite: the
# EEPROM is wiped on every programming run, so a reprogrammed part
# always starts from the defaults its own build defines. See the flash
# target above.
#
# What the reversal costs, stated plainly: bench settings do not survive
# reprogramming, so a mode, profile or offset set by hand has to be set
# again after a flash. What it buys is that a stale layout can never
# outlive the build that wrote it, and MODE_MAGIC stops being the only
# thing standing between the two.
#
# AND EESAVE HAD NOTHING TO PROTECT AGAINST. The fuse tells NVMCTRL's
# CHER command to skip the EEPROM when it erases Flash. But the erase
# that runs during a programming run was measured NOT to clear the
# EEPROM in the first place, with the fuse already clear -- so setting
# it changed nothing that could be observed.
#
# THE ARGUMENT THAT USED TO SIT HERE WAS THE OTHER WAY ROUND, and it was
# wrong: that avrdude's UPDI erase goes through the ASI chip-erase key,
# which has to clear everything or it would be a way round the lock, and
# so ignores EESAVE. It reads well and it is not what the bench shows.
# The block above the flash target has the full correction; it is
# repeated here only because this is where somebody deciding to set the
# fuse again would look.
#
# The remaining fuses are left alone: WDTCFG 0x00 (watchdog off),
# OSCCFG 0x00 (OSCHF 20MHz, which is the PLL's source here), SYSCFG1
# 0x07 (the longest 64ms start-up delay), CODESIZE/BOOTSIZE 0x00 (whole
# Flash is one section, which is what the CRCSCAN erratum wants).
SYSCFG0 ?= 0xD0

FUSE_WRITE = -U bodcfg:w:$(BODCFG):m -U syscfg0:w:$(SYSCFG0):m

fuses:
	"$(AVRDUDE)" $(AVRDUDE_FLAGS) $(FUSE_WRITE)

# Read the EEPROM out, to stdout as hex.
#
# This exists because the flash target no longer reads the EEPROM as a
# side effect. It used to back it up on every programming run, so a dump
# was always lying around; now nothing produces one unless asked.
#
#   make readeeprom            hex to the terminal
#   make readeeprom EEDUMP=x.bin   binary to a file, if a byte-by-byte
#                                  decode against the map is wanted
#
# WHAT TO EXPECT. Straight after a flash, every byte should read 0xff --
# that is the wipe, and confirming it is worth doing once. On a unit
# that has been used, the first 25 bytes are the settings and the rest
# stays 0xff. Byte 0 is MODE_MAGIC and should read 0xe5; if it does not,
# load_settings() discards everything and applies defaults.
#
# The map is in main.c beside the EE_ defines. Bytes 4 onward are per
# profile, three each, in profile order.
#
#   0 magic  1 mode  2 b1_locked  3 profile
#   4 edid   7 hoff  10 voff  13 foff  16 edpos  19 edsize  22 csmeth
#
# A byte reading 0xff is NOT a fault: every loader range-checks and
# treats an erased byte as "use the default". It means nothing has
# written that setting since the last programming run.
EEDUMP ?= -

readeeprom:
	"$(AVRDUDE)" $(AVRDUDE_FLAGS) -U eeprom:r:$(EEDUMP):$(if $(filter -,$(EEDUMP)),h,r)

# Read back every fuse, to confirm the above took and nothing else moved.
#
# `:r:-:h` sends the VALUE to stdout while avrdude's progress bars go to
# stderr, so the numbers arrive interleaved with the bars and unlabelled,
# in the order requested. Each is echoed with its name first so the
# output can be read rather than counted.
#
# Expected, on a part these fuses have been written on:
#   wdtcfg   0x00   watchdog off
#   bodcfg   0x64   see BODCFG above
#   osccfg   0x00   OSCHF 20MHz, the PLL's source
#   syscfg0  0xd0   as shipped; EESAVE clear so the EEPROM is wiped
#   syscfg1  0x07   longest start-up delay
#   codesize 0x00   whole Flash is one section
#   bootsize 0x00
#
# To capture just the values:  make readfuses ... > fuses.txt
readfuses:
	@echo wdtcfg:
	"$(AVRDUDE)" $(AVRDUDE_FLAGS) -U wdtcfg:r:-:h
	@echo bodcfg:
	"$(AVRDUDE)" $(AVRDUDE_FLAGS) -U bodcfg:r:-:h
	@echo osccfg:
	"$(AVRDUDE)" $(AVRDUDE_FLAGS) -U osccfg:r:-:h
	@echo syscfg0:
	"$(AVRDUDE)" $(AVRDUDE_FLAGS) -U syscfg0:r:-:h
	@echo syscfg1:
	"$(AVRDUDE)" $(AVRDUDE_FLAGS) -U syscfg1:r:-:h
	@echo codesize:
	"$(AVRDUDE)" $(AVRDUDE_FLAGS) -U codesize:r:-:h
	@echo bootsize:
	"$(AVRDUDE)" $(AVRDUDE_FLAGS) -U bootsize:r:-:h

clean:
	rm -f $(TARGET).elf $(TARGET).hex $(TARGET).lst

.PHONY: all size lst flash fuses readfuses readeeprom clean
