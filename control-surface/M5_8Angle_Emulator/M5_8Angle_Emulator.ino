/*
 * M5Stack 8Angle Unit emulator (extended) — STM32F411CE Black Pill
 * STM32duino core. I2C slave @ 0x43.
 *
 * BACKWARDS COMPATIBLE with the stock M5 8Angle driver:
 *   - Pots, switch, LEDs, firmware & address registers are byte-identical
 *     to the original unit, so the standard driver works unchanged.
 *   - A SECOND toggle switch lives in a NEW register (0x22) the stock
 *     driver never reads. Only host code that looks at 0x22 sees it.
 *
 * Board: select "Generic STM32F4 series" -> "BlackPill F411CE".
 *
 * Hardware:
 *   Pots:     PA0..PA7   (ADC1, 12-bit native)
 *   Switch 1: PB0        (stock, register 0x20)
 *   Switch 2: PB1        (extension, register 0x22)   [NEW]
 *   LEDs:     PB6   (9x SK6812/WS2812: 8 pots + switch LED)
 *   I2C1:     SCL=PB8, SDA=PB9  -> to host (3.3V)
 *
 * Both toggles are debounced in firmware.
 * Powered from the host's 3.3V Grove pin.
 */
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#define I2C_ADDR     0x43
#define FW_VERSION   1
#define NUM_CH       8
#define NUM_LED      9        // 8 pots + switch LED
#define LED_PIN      PB6

// ---- toggle switches ----
#define SWITCH1_PIN  PB0      // stock switch  -> register 0x20
#define SWITCH2_PIN  PB1      // second switch -> register 0x22  [NEW]
#define DEBOUNCE_MS  20

// ---- register map ----
#define REG_ADC12_BASE 0x00      // 8x 12-bit ADC, little-endian (stock)
#define REG_ADC8_BASE  0x10      // 8x 8-bit ADC (stock)
#define REG_SWITCH     0x20      // toggle switch 1 (stock)
#define REG_SWITCH2    0x22      // toggle switch 2  [NEW, non-stock]
#define REG_RGB_BASE   0x30      // RGB+brightness per LED (stock)
#define REG_FW_VER     0xFE      // firmware version (stock)
#define REG_I2C_ADDR   0xFF      // I2C address (stock)

const uint32_t POT_PIN[NUM_CH] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};

volatile uint8_t regs[256];
volatile uint8_t regPtr = 0;

// debounce state, one slot per switch: [0]=switch1, [1]=switch2
const uint32_t SW_PIN[2] = {SWITCH1_PIN, SWITCH2_PIN};
uint8_t  swStable[2]    = {1, 1};   // 1 = open (pull-up)
uint8_t  swLastRaw[2]   = {1, 1};
uint32_t swChangedAt[2] = {0, 0};

Adafruit_NeoPixel strip(NUM_LED, LED_PIN, NEO_GRB + NEO_KHZ800);

// I2C1 on PB8/PB9. TwoWire(SDA, SCL) on STM32duino.
TwoWire WireSlave(PB9, PB8);

void onReceive(int n) {
  if (n <= 0) return;
  regPtr = WireSlave.read();
  while (WireSlave.available()) {
    regs[regPtr++] = WireSlave.read();   // writes: RGB, brightness, addr
  }
}

void onRequest() {
  // host reads sequentially from regPtr
  for (uint8_t i = 0; i < 16; i++)
    WireSlave.write(regs[(uint8_t)(regPtr + i)]);
}

void setup() {
  pinMode(SWITCH1_PIN, INPUT_PULLUP);
  pinMode(SWITCH2_PIN, INPUT_PULLUP);

  analogReadResolution(12);            // native 12-bit, 0..4095

  strip.begin();
  strip.show();

  regs[REG_FW_VER]   = FW_VERSION;
  regs[REG_I2C_ADDR] = I2C_ADDR;
  for (uint8_t i = 0; i < NUM_LED; i++)
    regs[REG_RGB_BASE + i*4 + 3] = 100;  // default brightness 100

  WireSlave.begin(I2C_ADDR);           // join bus as slave/target
  WireSlave.onReceive(onReceive);
  WireSlave.onRequest(onRequest);
}

// returns debounced stable level (1=open, 0=closed) for switch index i
uint8_t debounceSwitch(uint8_t i, uint32_t now) {
  uint8_t raw = digitalRead(SW_PIN[i]);
  if (raw != swLastRaw[i]) {                 // raw changed -> restart timer
    swLastRaw[i]   = raw;
    swChangedAt[i] = now;
  }
  else if ((now - swChangedAt[i]) >= DEBOUNCE_MS) {
    swStable[i] = raw;                        // held long enough -> accept
  }
  return swStable[i];
}

void loop() {
  // 1. sample pots -> 12-bit (little-endian) + 8-bit registers
  for (uint8_t ch = 0; ch < NUM_CH; ch++) {
    uint16_t v = analogRead(POT_PIN[ch]);   // 0..4095
    regs[REG_ADC12_BASE + ch*2]     = v & 0xFF;
    regs[REG_ADC12_BASE + ch*2 + 1] = (v >> 8) & 0xFF;
    regs[REG_ADC8_BASE  + ch]       = v >> 4;   // 12 -> 8 bit
  }

  // 2. debounced toggle switches (pressed/closed = 1)
  uint32_t now = millis();
  regs[REG_SWITCH]  = (debounceSwitch(0, now) == LOW) ? 1 : 0;   // 0x20 (stock)
  regs[REG_SWITCH2] = (debounceSwitch(1, now) == LOW) ? 1 : 0;   // 0x22 (new)

  // 3. push RGB registers to LED chain
  for (uint8_t i = 0; i < NUM_LED; i++) {
    uint8_t r  = regs[REG_RGB_BASE + i*4 + 0];
    uint8_t g  = regs[REG_RGB_BASE + i*4 + 1];
    uint8_t b  = regs[REG_RGB_BASE + i*4 + 2];
    uint8_t br = regs[REG_RGB_BASE + i*4 + 3];   // 0..100
    strip.setPixelColor(i, (uint16_t)r*br/100, (uint16_t)g*br/100, (uint16_t)b*br/100);
  }
  //strip.show();

  delay(5);   // ~200 Hz
}
