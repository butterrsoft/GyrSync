# GyrSync
CRT monitor protection and diagnostic dongle for 240p/15kHz, 25kHz and/or 480p/31kHz, with EDID, h-pos, v-pos, field-offset, and selectable sync pass/combine/polarity methods. H-sync signals outside the desired range block all sync output. Diagnoses and reports missing sync. RGB is still passed through, so you may see something harmless on screen.


Powered via your VGA port, or any USB-C cable. Or both; USB cannot backfeed VGA. Updates/reprograms via a single Gitbash Make command through an $8USD UPDI friend.

### Acknowledgments/Confessions:
Built after Gambaman’s superb Ultimate VGA to Scart Adapter. I got carried away. Code by Claude, and, yes, I am embarrassed. All free and open-source, but who doesn't like seeing their name in print? :)

## UI
Single button-presses change basic modes and adjust settings. Button-holds are timed to move between secondary modes, activating on release. Set the dongle as you want, then lock it so you can’t bump anything.

Dongle starts at Factory Reset – Mode 1, Profile 1, button locked out. 

### UI topology
```text
Lockout ──▶ Factory Reset
   │
   ▼
Modes 1-5 ──▶ Profile 1/2/3 ──▶ Mode S ──▶ Mode CS ──▶ Mode ED ──▶ Lockout ──▶ Factory Reset
                                    │
                                    ▼
                                 Mode H ──▶ Mode V ──▶ Mode FO
```

### Lockout Mode UI tables:
| Short press | No change |
|:---|:---|
| UI LED | Blinks mode, profile number |

| Hold, release | UI LED | Goes to |
|:---|:---|:---|
| 3s | Blinking | Mode 1-5 |
| 10s | Dark | Factory reset |

### Mode 1-5 UI tables:
| Short press | Changes mode |
|:---|:---|
| UI LED | Blinks mode, profile number |
