// ============================================================================
//  I2C_8Angle_Diag — minimal I2C diagnostic for the 8Angle (real or emulator)
//  on the AMYboard (ESP32-S3). Converts "it doesn't work" into "THIS transaction
//  fails on THIS line." Prints a bus scan, a bare connect probe, the version
//  register, and a few pot reads — each with explicit PASS/FAIL.
//
//  Uses the SAME pins/address as the reverb sketch (SDA=17, SCL=18, 0x43) so the
//  results transfer directly. The read pattern mirrors Rob Tillaart's M5ANGLE8
//  library (write reg -> endTransmission() STOP -> requestFrom), which is the
//  code that WORKS on an Arduino host — so a FAIL here isolates the ESP32 master
//  <-> STM32 slave interaction rather than our framing.
//
//  HOW TO READ THE OUTPUT
//   - Bus scan lists every address that ACKs. Expect 0x3C (OLED) always; 0x43 is
//     the 8Angle/emulator. If 0x43 is ABSENT from the scan, the master never even
//     gets an address ACK -> electrical/addressing at this master, not framing.
//   - "connect probe" is the simplest possible transaction (no register, no read).
//     PASS + reads FAIL  -> the READ path is the problem (pointer-set or requestFrom).
//     connect FAIL       -> addressing/ACK problem; reads can't work either.
//   - Version reg 0xFE and pot reads exercise the actual read path the reverb uses.
//
//  amy_start() ORDERING: the reverb sketch brings up I2C only AFTER amy_start()
//  (a documented cold-boot latch on this board). If the diagnostic sees NOTHING
//  on the bus (not even the OLED at 0x3C), flip USE_AMY_START to 1 to reproduce
//  that ordering. Default 0 keeps this sketch dependency-free; try it first.
// ============================================================================

#include <Wire.h>
#include "USB.h"

// ---- manual USB-CDC (AMYboard does NOT set ARDUINO_USB_CDC_ON_BOOT) ----------
// Same pattern as the reverb sketch: instantiate a USBCDC, alias it to Serial,
// and call USB.begin() in setup(). Without this the board never enumerates as a
// serial device and you see nothing — which is exactly what happened.
#if !ARDUINO_USB_CDC_ON_BOOT
USBCDC USBSerial;
#define Serial USBSerial
#endif

#define I2C_SDA_PIN   17
#define I2C_SCL_PIN   18
#define ANGLE8_ADDR   0x43
#define OLED_ADDR     0x3C

#define I2C_CLOCK     100000     // match the reverb sketch; try 50000/10000 too
#define USE_AMY_START 0          // 1 = init AMY before I2C (see note above)

#if USE_AMY_START
  #include "amy.h"
#endif

// -- Tillaart-style primitives (STOP framing, exactly like the working library) --
static int  g_lastErr = 0;

static bool i2cConnected(uint8_t addr){
  Wire.beginTransmission(addr);
  g_lastErr = Wire.endTransmission();     // no register, no read: pure ACK test
  return (g_lastErr == 0);
}

// returns -1 on failure, else the byte
static int i2cRead8(uint8_t addr, uint8_t reg){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  g_lastErr = Wire.endTransmission();     // STOP (matches M5ANGLE8::read8)
  if(g_lastErr != 0) return -1;           // pointer-set failed
  if(Wire.requestFrom(addr,(uint8_t)1) != 1) return -1; // no data byte
  return Wire.read();
}

// returns -1 on failure, else the 12-bit value (lo then hi<<8, like read16)
static long i2cRead16(uint8_t addr, uint8_t reg){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  g_lastErr = Wire.endTransmission();
  if(g_lastErr != 0) return -1;
  if(Wire.requestFrom(addr,(uint8_t)2) != 2) return -1;
  uint16_t v = Wire.read();
  v += (uint16_t)Wire.read() << 8;
  return v;
}

// endTransmission() error codes (ESP32/Arduino): 0=ok,1=data too long,
// 2=NACK on addr, 3=NACK on data, 4=other, 5=timeout.
static const char* endErrStr(int e){
  switch(e){
    case 0: return "OK";
    case 1: return "data-too-long";
    case 2: return "NACK-on-address";
    case 3: return "NACK-on-data";
    case 4: return "other";
    case 5: return "timeout";
    default:return "?";
  }
}

void busScan(){
  Serial.println("\n--- I2C bus scan ---");
  int found=0;
  for(uint8_t a=1; a<127; a++){
    Wire.beginTransmission(a);
    int e=Wire.endTransmission();
    if(e==0){
      Serial.printf("  ACK at 0x%02X%s\n", a,
        a==ANGLE8_ADDR ? "  <- 8Angle/emulator" :
        a==OLED_ADDR   ? "  <- OLED" : "");
      found++;
    }
  }
  if(!found) Serial.println("  (nothing ACKed — bus/master/electrical problem)");
  Serial.printf("--- scan done: %d device(s) ---\n", found);
}

void probe8Angle(){
  Serial.println("\n--- 8Angle probe (0x43) ---");

  // 1) bare connect (simplest transaction possible)
  bool conn = i2cConnected(ANGLE8_ADDR);
  Serial.printf("[1] connect probe : %s (endTransmission=%d %s)\n",
                conn?"PASS":"FAIL", g_lastErr, endErrStr(g_lastErr));

  // 2) version register 0xFE (single-byte read path)
  int ver = i2cRead8(ANGLE8_ADDR, 0xFE);
  Serial.printf("[2] version 0xFE  : %s",
                ver>=0?"PASS":"FAIL");
  if(ver>=0) Serial.printf(" -> 0x%02X (%d)", ver, ver);
  else       Serial.printf(" (endTransmission=%d %s)", g_lastErr, endErrStr(g_lastErr));
  Serial.println();

  // 3) switch register 0x20 (single-byte read path)
  int sw = i2cRead8(ANGLE8_ADDR, 0x20);
  Serial.printf("[3] switch 0x20   : %s",
                sw>=0?"PASS":"FAIL");
  if(sw>=0) Serial.printf(" -> 0x%02X", sw);
  else      Serial.printf(" (endTransmission=%d %s)", g_lastErr, endErrStr(g_lastErr));
  Serial.println();

  // 4) three 12-bit pot reads at 0x00,0x02,0x04 (the two-byte read path)
  for(uint8_t ch=0; ch<3; ch++){
    long v = i2cRead16(ANGLE8_ADDR, (uint8_t)(ch*2));
    Serial.printf("[4] pot ch%u @0x%02X: %s",
                  ch, ch*2, v>=0?"PASS":"FAIL");
    if(v>=0) Serial.printf(" -> %ld", v);
    else     Serial.printf(" (endTransmission=%d %s)", g_lastErr, endErrStr(g_lastErr));
    Serial.println();
  }

  Serial.println("--- probe done ---");
}

void setup(){
#if !ARDUINO_USB_CDC_ON_BOOT
  USB.begin();                    // AMYboard: bring up USB-CDC before Serial
#endif
  Serial.begin(115200);
  delay(3000);                    // CDC enumerate + emulator spin-up
  Serial.println("\n=== I2C 8Angle Diagnostic ===");
  Serial.printf("pins SDA=%d SCL=%d, clock=%d Hz, USE_AMY_START=%d\n",
                I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK, USE_AMY_START);

#if USE_AMY_START
  // Reproduce the reverb sketch's cold-boot ordering (I2C after AMY).
  amy_config_t cfg = amy_default_config();
  cfg.features.startup_bleep = 0;
  amy_start(cfg);
  Serial.println("amy_start() done (USE_AMY_START=1)");
#endif

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK);
}

void loop(){
  busScan();
  probe8Angle();
  Serial.println("\n(repeating in 3 s — flip the switch / turn a pot between runs)\n");
  delay(3000);
}
