# Bill of Materials

Parts confirmed from the firmware and technical notes. Fill in exact sources,
part numbers, and prices where you'd like to point builders to what you
actually used.

| Part | Role | Notes | Source / link |
|------|------|-------|----------------|
| AMYboard (ESP32-S3) | Main host, audio I/O | Eurorack-format synth/DSP board running the AMY library. Includes the ADS1115 CV-input ADC onboard — not a separate part | |
| M5Stack 8Angle | 8 potentiometers + toggle + 9 RGB LEDs | I2C `0x43`, connects to AMYboard's **front-panel** Grove I2C (not the back host port) | |
| SH1106 128×64 OLED | Display | I2C `0x3C`, same Grove bus as the 8Angle, you need to solder a Y-cable for the four I2c pins | |
| Eurorack panel (3D printed) | Front panel, 26HP / 3U — two variants, see `hardware/panel/` | |
| 3U/28HP skiff case (3D printed) | Enclosure | Remix of a CC BY 4.0 MakerWorld design — see `hardware/CASE_NOTE.md` | printed by you |


** all files 3D printed on a Bambulab P2S with PETG filament, no supports, 0,4mm nozzle**
