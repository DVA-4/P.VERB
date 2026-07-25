# Build Guide

You only need  the Amyboard, M5stack 8-angle 8-pot peripheral and any SH1106 I2C OLED and a Y-cable for connecting both OLED and 8-angle unit to the Amyboard to get you started.
There are two versions of the Eurorack frontpanel. One uses the stock 8-angle unit PCB (just remove it from its case). Thats the easiest to get started. The other frontpanel is for breaking out all the potentiometers of the 8-angle unit as 7 external potentiometers and a slide pot for decay.you can use the 8Angle Emulator for that or hack the original 8Angle.
If you just want to try out the code first, just connect amyboard, OLED and 8-angle unit and upload the code. 
You'll have a fully working unit.


## 1. Parts
See `docs/BOM.md`.

## 2. Print the panel and knobs
There are **two front-panel variants** in `hardware/panel/`:
- `P.VERB_Panel_1_Stock8Angle.stl` — for an unmodified 8Angle unit (PCB
  still in its own case). This is the easiest to build.
- `P.VERB_Panel_2_Custom.stl` — an 8angle Emulator running on the STM32F411 BlackPill
  that allows for an additional toggle switch and custom layout of 7 potentiometers and a fader.

Knob STLs are in `hardware/knobs/`.

I used PETG filament on a Bambu Lab P2S with a standard 0.4mm nozzle. No
supports were needed.

The 8Angle PCB and OLED are mounted to the panel with hot glue.

## 3. Wire the front-panel bus
- M5Stack 8Angle and SH1106 OLED both go on the AMYboard's **front-panel**
  Grove I2C port (SDA = GPIO 17, SCL = GPIO 18) — not the back "host" port.
  You need to solder or get a Y-cable for the 4 I2c pins to connect both peripherals.


## 4. Set up the Arduino toolchain
1. Install Arduino IDE and add the AMYboard board support (arduino-esp32
   core **3.3.8 or newer** — the AMYboard board target doesn't exist in
   older core versions).
2. Install the AMY library, **version 1.2.14+ or `main`**
   (github.com/shorepine/amy) — via Library Manager or "Add .ZIP Library."
3. Keep the core and AMY library versions in step.

## 5. Flash the firmware
1. Open `firmware/Pverb_DattorroReverb_AMYboard.ino` in the Arduino IDE
   (`eltro_font5x7.h` must sit alongside it in the same folder).
2. Select the AMYboard target board.
3. Upload.
4. On boot, the serial monitor should print a `Ready [BUILD: ...]` marker —
   confirm it matches the version you flashed.

## 6. First power-up checklist
- Confirm audio passes through in bypass before enabling the effect.
- For full operating instructions once it's running, see
  `docs/P.VERB_Manual.pdf`.

## 7. Mount in the case
The skiff case STL is in `hardware/case/` — it's a remix of a third-party
CC BY 4.0 design, see `hardware/CASE_NOTE.md` for attribution and what was
changed.
