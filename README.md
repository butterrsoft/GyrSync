# 📺 GyrSync 
CRT TV/monitor protection and diagnostic dongle for 240p/15kHz, 25kHz and/or 480p/31kHz, with EDID, h-pos, v-pos, field-offset, and selectable sync pass/combine/polarity methods. H-sync signals outside the desired range are blocked. Diagnoses and reports missing sync. RGB is still passed through, so you may see something harmless on screen.

((picture))

Powered via your VGA port, or any USB-C cable. Or both; USB cannot backfeed VGA. Updates/reprograms via a single Gitbash Make command through an $8USD UPDI friend.

### Acknowledgments/Confessions:
Built after Gambaman’s superb Ultimate VGA to Scart Adapter. I got carried away. Code by Claude, and, yes, I am embarrassed. All free and open-source, but who doesn't like seeing their name in print? :)

## UI
Single button-presses change basic modes and adjust settings. Button-holds are timed to move between secondary modes, activating on release. Set the dongle as you want, then lock it so you can’t bump anything.

Dongle starts at Factory Reset – Mode 1, Profile 1, button locked out. 

UI topology:
```text
Lockout ──▶ Factory Reset
   │
   ▼
Modes 1-5 ──▶ Profile 1/2/3 ──▶ Mode S ──▶ Mode CS ──▶ Mode ED ──▶ Lockout ──▶ Factory Reset
                                    │
                                    ▼
                                 Mode H ──▶ Mode V ──▶ Mode FO
```

Lockout Mode UI tables:
| Short press | No change |
|:---|:---|
| UI LED | Blinks mode, profile number |

| Hold, release | 3s | 10s |
|:---|:---|:---|
| UI LED | Blinking | Dark |
| Goes to | Mode 1-5 | Factory reset |

Mode 1-5 UI tables:
| Short press | Changes mode |
|:---|:---|
| UI LED | Blinks mode, profile number |

| Hold, release | 2s | 4s | 6s | 8s | 11s | 13s | 16s | 20s |
|:---|:---|:---|:---|:---|:---|:---|:---|:---|
| UI LED | 1 blnk | 2 blnks | 2 blnks | 2+2 blnks | Solid | Dark | Blinking | Dark |
| Goes to | Profile 1 | Profile 2 | Profile 3 | Mode S | Mode CS | Mode ED | Lockout (in current mode) | Factory reset |

Mode S UI tables:
| Short press | Goes to Mode 1 |
|:---|:---|
| UI LED | Blinks Mode 1, profile number |

| Double press | Toggle EDID on/off |
|:---|:---|
| UI LED | Accel/decel pulses |

| Hold, release | 2s | 4s | 6s |
|:---|:---|:---|:---|
| UI LED | Solid | Dark | Blinking |
| Goes to | Mode H | Mode V | Mode FO |

Adjustment Mode UI tables:
| Mode | H, V | ED, FO | CS |
|:---|:---|:---|:---|
| Short press | Step adj. | Step adj. | Sync pass/combine to next method |
| UI LED | Long blink middle and ends only | Rate blink slower in middle, faster at ends. Long blink middle and ends. | Pulses method number |

| Hold | 2s | 4s | 6s | 8s (Mode ED only) |
|:---|:---|:---|:---|:---|
| UI LED | Solid | Dark | Solid | Pulsing |
| Action on release | Change adj. direction | Save adj. | Reset to default | Switch between pos/width (ED only) |


## Mode descriptions
➕ Mode 1-5, Mode S information table:
| Mode | Mode 1 | Mode 2 | Mode 3 | Mode 4 | Mode 5 | Mode S |
|:---|:---|:---|:---|:---|:---|:---|
| Passes | 15kHz | 31kHz | 15/31kHz | 25kHz | 15/25/31kHz | 15kHz + |
| EDID name | Mode1 - 15kHz | Mode 2 - 31kHz | Mode 3 - dualsyn | Mode 4 - 25kHz | Mode 5 - Trisync | Mode S - special |
| DTD 1 | 640x240p | 640x480p | 640x240p | 496x384p | 1280x240p | 640x240p |
| DTD 2 | 1280x240p | — | 1280x240p | — | 496x384p | 1280x240p |
| DTD 3 | 720x480i | — | 720x480i | — | 720x480i | 720x480i |

If you push right up to the edges of any acceptable sync range, detection is inconsistent and the output and your picture will not be usable. Back off and move your modeline a notch or two inside the safe zone.

There is also one jumper inside the Gyrsync - allowing UPDI programming via VGA Pin 11 on the output side. If you have a really, really old monitor that needs Monitor ID bit 0, you may need to remove this. Also, if the dongle seems stuck in some sort of loop.

### ➕ Mode S - special
Mode S passes 15kHz as normal, but takes 31kHz and divides it by two to make it 15kHz too - side-by-side images but at true 15kHz. This lets you use Safe Mode, or get out of trouble if Windows keeps grabbing 480p and pissing you off.

If there is no user activity, Mode S reports sync status each 5s (but still blocks sync out of range).

Mode S sync report table:
| UI LED | 1 blink | 2 blinks | 3 blinks | Does nothing |
|:---|:---|:---|:---|:---|
| Means | No V-sync | No H-sync | No sync (both) | Sync ok |

### Adjustment modes
Adjustments are possible without a picture – power the dongle from any USB-C cable. The UI blinks SOS/cannot-adjust in only three scenarios – Mode FO if no interlaced mode is detected (progressive, or just missing vsync), Mode ED if EDID is off, Mode V if the source FPS is unstable.

Adj. modes time out back to the mode you came from in 20s, or 30s for Mode ED. Same happens on reboot. Unsaved changes are discarded.

Adj. mode h-sync pass table: (Prev. mode = if you were in Mode 4, 25kHz, Mode ED is still at 25kHz.)
|  | Mode CS | Mode ED | Mode H | Mode V | Mode FO |
|:---|:---|:---|:---|:---|:---|
| Passes | Per prev. mode | Per prev. mode | 15/25/31kHz | 15/25/31kHz | 15/25/31kHz |

➕ **Mode CS** - sync output method table: (H-/C-sync on Pin 13, V-sync on Pin 14 of the female DB15 output. Flip refers to the input sync polarity. Sync must be in range, still)
| Method | M1 (default) | M2 | M3 | M4 | M5 | M6 | M7 |
|:---|:---|:---|:---|:---|:---|:---|:---|
| Pin13 | Csync | Csync | Csync (no serr.) | pass | flip | pass | flip |
| Pin14 | block | pass | pass | pass | pass | flip | flip |

➕ **Mode ED** - moves the h-pos and h-size in the EDID. This affects all DTD’s over the current profile, not per-mode. Hit “Detect” under Windows Display Settings to see any changes, or hot plug dongle while USB-powered.

➕ **Mode H** - moves h-pos. May jitter as the AVR is only granular to 50ns. Better to adjust the source, the CRT, or use Mode ED.

➕ **Mode V** - moves v-pos.
 
➕ **Mode FO** - field-offset! Interlaced only. Moves the odd field up and down vs the even field, to adjust flicker. I really wanted to try this, and it works, though YMMV.

## EDID
Extended Display Identification Data. Your OS reads it from any monitor and then sends the video mode the monitor asked for. The Gyr sync transmits 15kHz, 25kHz and 31kHz modes based on how you set it. Modern OS’s are finicky, won’t do interlaced, but *should* grab the base 240p mode from the 15kHz DTD’s and simply display it. Or 384p/480p depending on what you set. Wait for the green LED on the Gyrsync before plugging in your CRT.

**Troubleshooting EDID**
*Make sure EDID is on on the GyrSync.
*Windows Display Settings > Advanced Display Settings for Desktop Resolution vs Active Signal Resolution.
*Turn off scaling in your GPU settings – Nvidia CP has it under desktop size & position – no scaling.
*Some HDMI dongles may mux-in EDID, which is ok, or simply provide their own, which is not.
*Something like crt_emudriver, or setting a cmdline.txt EDID in Linux, can override any physical EDID, and you will not see the GyrSync or it’s modelines listed anywhere. (Sync-protection and other features will still work as normal)


## Reprogramming:
Edit your EDID bin files using a free program like Deltacast, but make sure you keep the same filenames. Or modify main.c. The build needs all 6 EDID files, the main.c file and the makefile. The Gyrsync targets an AVR32EB14. Get [ZakKemble's latest AVR build](https://github.com/ZakKemble/avr-gcc-build) along with [Git](https://git-scm.com/install/windows) and [Make](https://gnuwin32.sourceforge.net/packages/make.htm), and install.

Programming is via UPDI, a 3-wire serial protocol. Probably best done while unplugged from anything else like your GPU or VGA source. You need a low-voltage UPDI friend/clone from aliexpress. Or even cheaper a USB-to-serial adapter set up as this:


<img width="600" height="285" alt="UPDI" src="https://github.com/user-attachments/assets/4fc254f3-601a-4799-928e-6fe3038ee7a7" />

On the Gyrsync, stick pins into the VGA female end – pin 9 is 5V, pin 6, 7 or 8 for GND, and pin 11 for UPDI. (Remembering the 1.27mm UPDI jumper) Then open a command prompt or gitbash where you have the project files and type `make clean` then `make TOOLDIR="C:/path/to/avr-gcc-16.1.0-x64-windows/bin/" AVRDUDE="C:/path/to/avr-gcc-16.1.0-x64-windows/bin/avrdude.exe" flash PORT=COM6` remembering to check those paths and com port.

Most USB-to-serial adapters should work, like a CP2102, or even those CH341 eeprom programmers like below (just flip the jumper to TTL and use the pins as marked on the reverse). I did have trouble with a CH340-based cable though.


<img width="300" height="230" alt="CH341" src="https://github.com/user-attachments/assets/8f8ad10a-edc0-4bf3-a797-b003165cab01" />








