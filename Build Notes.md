## CAUTION
This build is a bitch. I will probably update with a larger through-hole PCB, or one that can be assembled for you by a fab, but this is it for now...

Order PCB at 1.0mm thick (or less, and edit the case body step file). Make sure you get VGA plugs/sockets with straight, thin pins. NOT solder pins with the cup-shape. Hard to find, but required here.

If you don't build in the following order, you will run into problems. Get some kapton tape. Go slowly. Check continuity to make sure you haven't shorted anything at each step. Work out *why* you are trimming each leg or VGA pin before doing so. Measure twice, cut once.

## Solder components in this order:
1. Schottky
2. Resistor for Green LED
3. Oscillator
4. Tactile switch (trim legs before soldering so they don't poke through at all, then insulate the pad closest to the top middle with kapton tape, so the USB-C socket doesn't short it to ground)
5. All other SMD components on the top side but NOT the 5k1 resistors
6. USB-C (make sure not to ground the switch!)
7. 5k1 resistors
8. All sMD components on the lower side, but not the AVR23EB14 main IC
9. Male/female DB15 3-row plug/socket (making sure to trim pins as below)
10. RGB wires (I like 28AWG solid-core high-temp wire for this). Connect pin1 to pin1, pin2 to pin2, pin3 to pin3.
11. 1.27mm 2-pin header
12. Blue LED (you will probably want to insulate the legs so the Red wire doesn't short to them, can use 1.5/3mm heatshrink)
13. Green LED (trim the legs before soldering so the AVR23EB14 can still sit flush)
14. AVR32EB14!
15. Red LED


## Trim VGA pins so:

### Male DB15 3-row

| Pin | Length (after trimming) |
| --- | --- |
| 1 | 3mm or bend out |
| 2 | 1.8mm |
| 3 | 1.8 |
| 4 | remove/trim flush |
| 5 | remove/trim flush |
| 6 | 1.8 |
| 7 | 1.8 |
| 8 | 1.8 |
| 9 | 1.8 |
| 10 | 2.2 |
| 11 | remove/trim flush |
| 12 | 1.8 |
| 13 | 1.8 |
| 14 | 1.8 |
| 15 | 2.2 |

### Female DB15 3-row

| Pin | Length (after trimming) |
| --- | --- |
| 1 | 3mm or bend out |
| 2 | 1.8mm |
| 3 | 1.8 |
| 4 | remove/trim flush |
| 5 | remove/trim flush |
| 6 | 1.8 |
| 7 | 1.8 |
| 8 | 1.8 |
| 9 | 1.8 |
| 10 | remove/trim flush |
| 11 | 1.8 |
| 12 | remove/trim flush |
| 13 | 2.2 |
| 14 | 2.2 |
| 15 | remove/trim flush |
