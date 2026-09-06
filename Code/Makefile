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

# THE SIX EDIDs ARE BUILD INPUTS NOW, AND THEY ARE THE ONLY EDID INPUT.
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
# .rodata NOW CARRIES THE SIX EDID IMAGES: 1536 bytes of the Flash
# figure, against 605 for the base+patch scheme it replaced. If that
# line ever moves by 256, an image has been added or lost.
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

# HOW EEPROM SURVIVES A REFLASH -- read this before changing it.
#
# avrdude chip-erases before writing flash, which wipes EEPROM, so
# MODE_MAGIC stops matching and load_settings() applies defaults. The
# unit then boots LOCKED with EDID OFF after every program.
#
# TWO THINGS HAVE BEEN TRIED. The second one is what is here.
#
# 1. EESAVE in SYSCFG0. Necessary, and NOT sufficient. The data sheet is
#    specific about what honours it: NVMCTRL's CHER command "erases
#    Flash and EEPROM. EEPROM is skipped if the EESAVE fuse is set".
#    That is the COMMAND. avrdude's UPDI erase does not necessarily go
#    through it -- the ASI chip-erase KEY is what unlocks a locked part,
#    so it has to clear everything or it would be a way round the lock,
#    and it ignores EESAVE by design. The fuse is still set, because it
#    costs nothing and may well be doing its job; it just cannot be
#    relied on alone.
#
# 2. -D, to suppress the erase. THIS DOES NOT WORK AND MUST NOT COME
#    BACK. It suppresses the erase and then issues a plain page WRITE,
#    not the erase-and-write it was assumed to do. Flash programming can
#    only clear bits, so every bit that needed to go 0 -> 1 stayed 0 and
#    verification failed:
#
#        device 0xe1 != input 0xe3 at addr 0x008e
#
#    0xe3 wanted, 0xe1 found: bit 1 missing. That is the signature, and
#    it leaves the part carrying a corrupt image that will not run.
#    Recover with `make flashfull`.
#
# What is here instead depends on avrdude honouring nothing: read EEPROM
# out, do a normal erasing flash, write EEPROM back. Three UPDI sessions
# rather than one, which costs a couple of seconds and is worth it on a
# bench where the unit is reprogrammed constantly.
#
# Both EEPROM steps are prefixed with - so a part with nothing to save
# still flashes. The consequence is that a FAILED RESTORE IS NOT AN
# ERROR, so if settings stop persisting, look at this output rather than
# at load_settings().
#
# THE BACKUP FILE IS NOT PER-PART. Flash one board, then another, and
# the second gets the first one's settings. Delete it, or use
# `make flashfull`, when moving the programmer between units.
EEPROM_BACKUP ?= eeprom_backup.bin

flash: $(TARGET).hex
	-"$(AVRDUDE)" $(AVRDUDE_FLAGS) -U eeprom:r:$(EEPROM_BACKUP):r
	"$(AVRDUDE)" $(AVRDUDE_FLAGS) -U flash:w:$(TARGET).hex:i
	-"$(AVRDUDE)" $(AVRDUDE_FLAGS) -U eeprom:w:$(EEPROM_BACKUP):r

# Erase everything and keep nothing. A new part, a stale EEPROM layout,
# a unit that will not boot, or moving the programmer to another board.
flashfull: $(TARGET).hex
	"$(AVRDUDE)" $(AVRDUDE_FLAGS) -U flash:w:$(TARGET).hex:i

# ---------------------------------------------------------------------
# Fuses.
#
# Only BODCFG is written. Every other fuse ships at a value this design
# wants, and two of them are actively dangerous to touch -- see below.
#
# BODCFG ships as 0x00: brown-out detector completely disabled. Two
# consequences follow, and together they explain settings being lost on
# a rapid unplug/replug. The CPU is released from reset at the ~1.4V
# power-on threshold rather than at a voltage where 20MHz is in spec;
# and, more importantly, a fast replug may not drop VDD far enough to
# trip the power-on reset at all, so no reset happens and the part
# carries on with whatever state it picked up while the rail sagged
# through the region where the NVM controller is unreliable. Enabling
# the BOD makes any dip below the threshold produce a real reset.
#
#   0x44 = LVL BODLEVEL2 (2.60V), ACTIVE continuous   <-- recommended
#   0x24 = LVL BODLEVEL1 (1.90V), ACTIVE continuous
#   0x64 = LVL BODLEVEL3 (4.30V), ACTIVE continuous
#
# 2.60V is the pragmatic choice, as on the ATtiny824. 4.30V looks more
# correct for a part running at 20MHz, but the supply here is a port's
# 5V rail less a Schottky drop -- about 4.7V typical and as low as
# ~4.45V at the bottom of the USB tolerance -- which leaves no margin
# over a 4.30V threshold once its own tolerance is counted, and buys
# nuisance resets instead of stability. 2.60V keeps a wide margin while
# still guaranteeing a reset before VDD reaches the region where the NVM
# controller misbehaves.
BODCFG ?= 0x44

# SYSCFG0 -- WRITTEN ONLY TO SET EESAVE, AND READ THIS BEFORE RUNNING IT.
#
# It ships as 0xD0. Exactly one bit changes:
#
#   0xD1 = 1101 0001
#          ||   |||
#          ||   ||+- EESAVE     1  EEPROM PRESERVED by a chip erase  <-- the change
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
# Why EESAVE is wanted: avrdude's UPDI programming does a chip erase, so
# every `make flash` wiped EEPROM. MODE_MAGIC then failed to match,
# load_settings() applied defaults, and the unit came back LOCKED with
# EDID OFF -- every single time, which is why DDC appeared to keep
# turning itself off. With EESAVE set, settings survive a reflash.
#
# The consequence to be deliberate about: stale settings now survive a
# firmware change too. MODE_MAGIC in main.c is the layout version that
# guards that, and it must be bumped whenever a stored value's range or
# meaning moves. A part carrying an older layout then discards it rather
# than reading yesterday's byte as though it meant today's thing.
#
# The remaining fuses are left alone: WDTCFG 0x00 (watchdog off),
# OSCCFG 0x00 (OSCHF 20MHz, which is the PLL's source here), SYSCFG1
# 0x07 (the longest 64ms start-up delay), CODESIZE/BOOTSIZE 0x00 (whole
# Flash is one section, which is what the CRCSCAN erratum wants).
SYSCFG0 ?= 0xD1

fuses:
	"$(AVRDUDE)" $(AVRDUDE_FLAGS) -U bodcfg:w:$(BODCFG):m \
	  -U syscfg0:w:$(SYSCFG0):m

# Read back every fuse, to confirm the above took and nothing else moved.
#
# `:r:-:h` sends the VALUE to stdout while avrdude's progress bars go to
# stderr, so the numbers arrive interleaved with the bars and unlabelled,
# in the order requested. Each is echoed with its name first so the
# output can be read rather than counted.
#
# Expected, on a part these fuses have been written on:
#   wdtcfg   0x00   watchdog off
#   bodcfg   0x44   see BODCFG above
#   osccfg   0x00   OSCHF 20MHz, the PLL's source
#   syscfg0  0xd1   0xd0 as shipped, plus EESAVE
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

.PHONY: all size lst flash flashfull fuses readfuses clean
