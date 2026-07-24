// ============================================================================
//  Dattorro Plate Reverb for the AMYboard (ESP32-S3 + AMY)
//  Real-time audio-in -> reverb -> audio-out, 8Angle control, SH1106 OLED.
//
//  Hardware bring-up (AMY config, osc activation, 8Angle I2C, library-free
//  SH1106 driver) comes from the Eltro MK3 reference sketch
//  (eltro_amyboard_v12.ino) -- that sketch is ground truth and overrides the
//  markdown guides wherever they differ. The reverb DSP is the Dattorro plate;
//  only the parameter set and display content are new.
//
//  Delay lengths are Dattorro's originals (@29761 Hz) pre-scaled to 44100 Hz.
//  BUILD: rev-24
//
//  Signal chain:
//    in -> bitcrusher -> predelay -> LPF -> HPF -> 4x input diffusion allpass
//       -> figure-8 tank (2 modulated allpasses + delay + damping per side,
//                         cross-coupled feedback)
//       -> fixed output taps x 0.6 -> mix
//
//  Two UI pages selected by the physical PAGE toggle (position IS the page).
//  Page A: decay(slide) damp LPF HPF crunch predelay diffusion mix
//  Page B: in-gain out-gain rate1 depth1 rate2 depth2 midi-ch
// ============================================================================

#include <AMY-Arduino.h>          // Arduino library wrapper; pulls AMY sources
                                  // into the LINK. NOT bare "amy.h" (that only
                                  // declares -> "undefined reference to amy_*").
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "esp_heap_caps.h"        // PSRAM allocation for the delay buffers
#include "esp_timer.h"            // esp_timer_get_time() for the real load gauge
#include <Preferences.h>          // ESP32 NVS flash storage for persistent Page-B settings
#include "USB.h"
#include "esp_rom_gpio.h"         // esp_rom_gpio_connect_out_signal -- MIDI TX inverted mirror
#include "soc/uart_periph.h"      // uart_periph_signal[] -- UART1 TX signal index
#include "hal/gpio_hal.h"         // gpio_set_direction (differential MIDI-out drive)

// ---- manual USB-CDC (board does not set ARDUINO_USB_CDC_ON_BOOT) -----------
#if !ARDUINO_USB_CDC_ON_BOOT
USBCDC USBSerial;
#define Serial USBSerial
#endif

// ---- I2C bus / peripheral addresses (front Grove: SDA17 SCL18) -------------
#define I2C_SDA_PIN   17
#define I2C_SCL_PIN   18
#define ANGLE8_ADDR   0x43
#define ANGLE8_POTS   8
#define ANGLE8_REG_SW1 0x20    // toggle 1 (page select): stock, present on real unit
#define ANGLE8_REG_SW2 0x22    // toggle 2 (bypass): custom 8Angle-emulator only

// Physical bypass switch is a COMPILE-TIME choice.
//   0 = stock M5stack 8angle unit (no 0x22): serial 'b' + MIDI CC116 drive bypass.
//   1 = custom 8angle emulator running on BlackPill with the bypass toggle wired to 0x22 (PB1). The switch is
//       AUTHORITATIVE: it re-asserts every uiTask pass, so serial/MIDI bypass
//       become inert while it is held.
#define HAS_BYPASS_SWITCH 1
#define OLED_ADDR     0x3C
#define POT_DEADBAND  10
#define DISPLAY_MS    100

// 8Angle orientation (matches the reference mounting). Channel order and turn
// direction are independent flags.
#define ANGLE8_REVERSED   1
#define ANGLE8_INVERT_DIR 1
static inline uint8_t HW_CH(uint8_t logical){
#if ANGLE8_REVERSED
  return (ANGLE8_POTS-1)-logical;
#else
  return logical;
#endif
}

// ---- Dattorro delay lengths, pre-scaled 29761 -> 44100 Hz -------------------
#define IN142     210
#define IN107     159
#define IN379     562
#define IN277     410
#define MOD_L     996
#define D1L      6598
#define AP2L     2667
#define D2L      5512
#define MOD_R    1345
#define D1R      6249
#define AP2R     3936
#define D2R      4687
#define EXC_MAX    32     // stage-1 buffer headroom, samples (modL/modR).
                           // pModDepth1 is live-adjustable but clamped to this;
                           // raise + reflash if you want to sweep depth higher.
#define EXC2_MAX   16     // stage-2 buffer headroom, samples (ap2L/ap2R)
// Diffusion (knob 7) range -- single source of truth, used by both the knob
// mapping (applyPotPageA case 7) and Plate::setDiffusion(), which scales the
// tank's second diffusion stage proportionally. Change here, not in two places.
#define DIFF_LO 0.25f
#define DIFF_HI 0.90f
// Decay-knob taper -- single source of truth, applied in decayFromNorm() so knob
// and MIDI CC share it. The knob maps LINEARLY in the feedback COEFFICIENT
// (0.20..0.98) but what you HEAR is RT60 ~= 1.34/-ln(decay), which explodes near
// 1.0 -- so the whole long-tail/freeze region lands in the top ~10% of travel.
// t -> t^DECAY_GAMMA spreads it out; endpoints fixed, so freeze is preserved.
// Evens travel in the COEFFICIENT, not in RT60: a cheap heuristic, not a
// calibrated perceptual map. SETTLED BY EAR at 1.6 -- the earlier 2.5 came from
// the coefficient math alone and left the bottom ~40% of slider travel audibly
// dead. Raise toward 2.5 for more top-end room, lower toward 1.0 for linear.
#define DECAY_GAMMA 1.6f
// LPF/HPF knob ceilings -- single source of truth. 8000 Hz on the LPF leaves a
// gentle-darkening middle ground; a lower ceiling makes the knob jump straight
// from "off" to "drastic". 1000 Hz on the HPF covers rumble cleanup through
// deliberately thin/telephone.
#define LPF_MAX_HZ 8000.0f
#define HPF_MAX_HZ 1000.0f
// Bitcrusher range -- single source of truth. pCrunch sweeps 1.0 (clean) down to
// 0.0 (max crunch); Plate::setCrunch() maps it to these two ranges on different
// curves. Bit depth is linear (each bit halves the noise floor, so it already
// feels even); decimation is SQUARED, because a linear map front-loaded all the
// audible crunch into the top quarter of the knob.
#define CRUNCH_BITS_MIN 8.0f     // bit depth at pCrunch=0 (max crunch)
#define CRUNCH_BITS_MAX 16.0f    // bit depth at pCrunch=1 (clean; 16-bit
                                  // quantization is already inaudible, so the
                                  // last stretch of "clean" knob travel is a
                                  // deliberate flat/transparent zone)
#define CRUNCH_DECIM_MAX 5.0f   // sample-and-hold factor at pCrunch=0 (max
                                  // crunch); ~44100/50 = 882Hz effective rate
// LFO shape: 0 = sine (Dattorro default), 1 = triangle (constant slew
// rate -- no fast-moving zero-crossing, tends to sound less "zippery" at
// the same peak depth; cheap since it's just a fabsf, no sinf).
#define MOD_SHAPE_TRIANGLE 0
#define PREDELAY_MAX 10600 // ~240 ms @44100 (pre-delay line capacity, samples).
                            // MUST cover knob 5's full range: setPredelay()
                            // clamps to this, so if the knob maps beyond it the
                            // top of the travel silently does nothing.

// ============================================================================
//  MIDI — in + out, all params, fully interchangeable with the knobs.
// ============================================================================
// One CC per parameter, flat-indexed 0..13 across both pages, contiguous in the
// MIDI-spec "undefined" range 102..119 (won't collide with mod=1, vol=7,
// expr=11, sustain=64, or the channel-mode messages 120..127). Each CC drives
// the SAME normalized mapping law as its knob (see the *FromNorm cores in the
// PARAMETER MAPPING section), so a CC value and the equivalent knob position
// produce identical parameter values -- MIDI and hand are interchangeable.
//   Page A knobs 0..7 -> CC 102..109   Page B knobs 0..5 -> CC 110..115
// Page B knob 6 (MIDI channel) intentionally has NO CC -- setting the RX channel
// via a CC on the channel you're listening to is a footgun. Bypass (not a knob)
// gets CC 116 as a toggle (val>=64 = on). CC65 stays the page toggle.
#define MIDI_CC_BASE   102        // Page A knob 0; params run BASE..BASE+13
#define MIDI_CC_BYPASS 116        // bypass toggle (>=64 on); not an owned param
#define MIDI_CC_PAGE    65        // page toggle (val>=64 -> Page B), mirrors slider
#define MIDI_NUM_PARAMS 14        // 8 Page-A + 6 Page-B (channel excluded)

// V11-style differential MIDI-out: AMY brings the TRS current loop to two GPIOs,
// one per DIN contact. A DIN-4/5 opto needs BOTH legs actively driven -- single-
// ended (one pin, other floating) never closes the loop; a static-LOW return
// closes it but garbles. Fix (bench-confirmed on the Eltro build): drive the loop
// push-pull -- GPIO14 = UART1 TX normal, GPIO15 = UART1 TX INVERTED via the
// ESP32-S3 GPIO matrix. RX is on AMY's dedicated MIDI-in pin, unaffected.
#define DATT_MIDI_TX_PIN     14   // UART1 TX, normal   -- data leg  (tip, DIN 5)
#define DATT_MIDI_RETURN_PIN 15   // UART1 TX, INVERTED -- return leg (ring, DIN 4)

// ============================================================================
//  DSP building blocks (mono float; buffers in PSRAM)
// ============================================================================
struct DelayLine {
  float* buf; int len; int w;
  void init(int n){
    len=n+1;
    // internal SRAM first -- kills PSRAM access latency in the tank
    // (measured ~53 cycles/access; allpasses alone are 24 of ~46 buffer
    // accesses/sample). Falls back to SPIRAM, then plain heap, so a full
    // 152 KB tank still boots on a build where internal RAM is tight.
    buf=(float*)heap_caps_calloc(len,sizeof(float),MALLOC_CAP_INTERNAL);
    if(!buf) buf=(float*)heap_caps_calloc(len,sizeof(float),MALLOC_CAP_SPIRAM);
    if(!buf) buf=(float*)calloc(len,sizeof(float));   // last-resort fallback
    w=0;
  }
  inline void write(float x){ buf[w]=x; }
  inline void advance(){ if(++w>=len) w=0; }
  inline float readInt(int d){ int i=w-d; if(i<0)i+=len; return buf[i]; }
  inline float readFrac(float d){
    int i=(int)d; float f=d-i;
    int a=w-i; if(a<0)a+=len;
    int b=a-1; if(b<0)b+=len;
    return buf[a]+f*(buf[b]-buf[a]);
  }
  inline float tap(int pos){ int i=w-pos; if(i<0)i+=len; return buf[i]; }
};

struct Allpass {
  DelayLine dl; int base; float g;
  // exc = max |mod| this instance will ever be called with; buffer must be
  // sized for it or readFrac() walks off the allocated range under full
  // modulation swing. Defaults to EXC_MAX (stage-1 ceiling) for source compat.
  void init(int n,float gain,int exc=EXC_MAX){ dl.init(n+exc+4); base=n; g=gain; }
  inline float process(float x,float mod){
    float d=base+mod;
    float delayed=dl.readFrac(d);
    float v=x+g*delayed;
    float y=-g*v+delayed;
    dl.write(v); dl.advance();
    return y;
  }
};

struct OnePole { float damp,z; void init(float d){damp=d;z=0;}
  inline float process(float x){ z=(1.f-damp)*x+damp*z; return z; } };

// One-pole highpass, built as the spectral complement of the one-pole
// lowpass above: y_hp = x - y_lp. This reuses the exact same damp<->Hz
// relationship as OnePole/setLPF (damp = exp(-2*pi*fc/sr)), so both filters
// respond to their Hz controls the same intuitive way (low Hz on the HPF's
// knob = barely filtering; high Hz = aggressive). An earlier version of this
// used a differentiator topology (y=damp*(yz+x-xz)) where damp's relationship
// to cutoff runs the OPPOSITE direction from the LPF's -- structurally a
// valid highpass, but confusing on a knob labeled directly in Hz, and (per
// standard DSP references) a one-pole differentiator is a known-poor choice
// for a controllable low-cutoff HPF/DC-blocker for exactly this reason.
struct OnePoleHP { float damp,z; void init(float d){damp=d;z=0;}
  inline float process(float x){ z=(1.f-damp)*x+damp*z; return x-z; } };

// LPF/HPF are 1-pole (6dB/oct), not the cascaded 2-pole tried earlier. At
// 2-pole (12dB/oct) per filter, overlapping LPF/HPF cutoff ranges compounded
// into an audibly deep notch/gain dip in the passband between them -- not
// resonance (cascaded real poles don't add a Q/peak), but two skirts
// stacking steeply enough to sound like distortion when the gap between
// cutoffs was narrow. 1-pole (6dB/oct) keeps the same topology and Hz-based
// controls but with a gentler slope, so overlapping ranges interact far less
// dramatically. Struct names (TwoPoleLP/TwoPoleHP) kept as-is so Plate's
// members, setLPF/setHPF, and process() don't need touching -- only the
// internals changed from 2 cascaded stages to 1.
struct TwoPoleLP { OnePole a;
  void init(float d){ a.init(d); }
  inline void setDamp(float d){ a.damp=d; }
  inline float process(float x){ return a.process(x); } };
struct TwoPoleHP { OnePoleHP a;
  void init(float d){ a.init(d); }
  inline void setDamp(float d){ a.damp=d; }
  inline float process(float x){ return a.process(x); } };

// Bitcrusher: sample-and-hold decimation + bit-depth quantization,
// combined into one process() so both effects share the same held-sample
// state (a real bitcrusher quantizes whatever it's currently holding, not
// the incoming sample every tick -- quantizing before the hold would lose
// the "stepped" character decimation is known for).
struct Crusher {
  float held;       // last sample-and-held value
  float counter;     // ticks since last hold-update
  float decim;        // current hold length, in samples (>=1)
  float qStep;         // quantization step size (2 / 2^bits); 0 = bypass (clean)
  void init(){ held=0.f; counter=0.f; decim=1.f; qStep=0.f; }
  inline float process(float x){
    counter+=1.f;
    if(counter>=decim){ counter=0.f; held=x; }
    if(qStep>0.f) return qStep*floorf(held/qStep+0.5f);   // quantize the HELD value
    return held;
  }
};

// Tank modulation: stage-1 (modL/modR) and stage-2 (ap2L/ap2R) each
// get independent rate+depth, live-adjustable via serial -- see uiTask's 'm'
// command and pModRate1/2, pModDepth1/2. Buffer headroom is fixed at compile
// time (EXC_MAX/EXC2_MAX); depth is clamped to that ceiling every block.

struct Plate {
  Crusher crush;                  // first thing the signal hits. Mono at this
                                   // point (dry is L+R summed before
                                   // Plate::process), so one instance suffices.
  DelayLine predelay;
  int predSamps;                 // current pre-delay in samples (set per block)
  TwoPoleLP bwLP;                 // 1-pole internally despite the name -- see struct
  TwoPoleHP bwHP;
  Allpass in1,in2,in3,in4;
  Allpass modL,ap2L,modR,ap2R;
  DelayLine d1L,d2L,d1R,d2R;
  OnePole dampL,dampR;
  float fbL,fbR, phL,phR, dphi;      // stage-1 LFO (modL/modR), quadrature
  float ph2L,ph2R, dphi2;            // stage-2 LFO (ap2L/ap2R): independent rate
                                      // + NON-quadrature phase, so the two
                                      // stages never beat together
  float depth1,depth2;               // live excursion, samples; clamped to
                                      // EXC_MAX/EXC2_MAX (the allocated headroom)
  float sampleRate;                  // cached so setModRates() can recompute
                                      // dphi/dphi2 live, same pattern as setDamp()

  void init(float sr){
    sampleRate=sr;
    crush.init();
    predelay.init(PREDELAY_MAX); predSamps=0;
    bwLP.init(0.0005f); bwHP.init(0.0005f);
    in1.init(IN142,0.75f); in2.init(IN107,0.75f);
    in3.init(IN379,0.625f);in4.init(IN277,0.625f);
    modL.init(MOD_L,0.70f);     ap2L.init(AP2L,0.50f,EXC2_MAX);
    modR.init(MOD_R,0.70f);     ap2R.init(AP2R,0.50f,EXC2_MAX);
    d1L.init(D1L); d2L.init(D2L); d1R.init(D1R); d2R.init(D2R);
    dampL.init(0.25f); dampR.init(0.25f);
    fbL=fbR=phL=0; phR=1.5708f;
    // stage-2: starting phases chosen so ph2R-ph2L ~= 117 deg (not the 90 deg
    // quadrature stage-1 uses) so the two LFOs never line up periodically.
    ph2L=0.7f; ph2R=2.75f;
    depth1=24.0f; depth2=6.0f;
    setModRates(0.40f,0.27f);          // sets dphi/dphi2; called again live
  }
  // Live rate control -- call once per block (like setDamp/setBandwidth) with
  // pModRate1/pModRate2. Recomputing dphi from rate+sr each call is 2 divides,
  // negligible next to the ~46 buffer accesses/sample already in the tank.
  inline void setModRates(float rate1Hz,float rate2Hz){
    dphi =2.f*3.14159265f*rate1Hz/sampleRate;
    dphi2=2.f*3.14159265f*rate2Hz/sampleRate;
  }
  // Live depth control -- clamped to the buffer headroom allocated at init,
  // so a runaway serial value can't walk readFrac() off the delay buffer.
  inline void setModDepths(float d1,float d2){
    depth1 = d1<0.f?0.f:(d1>EXC_MAX ?EXC_MAX :d1);
    depth2 = d2<0.f?0.f:(d2>EXC2_MAX?EXC2_MAX:d2);
  }
  inline void setDamp(float d){ dampL.damp=d; dampR.damp=d; }
  // Bitcrusher: t = pCrunch (1.0 clean .. 0.0 max crunch). Bit depth linear;
  // decimation SQUARED so the knob's clean end stays near-transparent before
  // crunch ramps up (a linear map front-loaded it all into the top quarter).
  inline void setCrunch(float t){
    if(t<0.f)t=0.f; if(t>1.f)t=1.f;
    float bits=CRUNCH_BITS_MIN+t*(CRUNCH_BITS_MAX-CRUNCH_BITS_MIN);
    crush.qStep = (bits>=16.0f) ? 0.f : (2.0f/powf(2.0f,bits)); // 16-bit+ = bypass quantizer
    float u=1.f-t; float u2=u*u;
    crush.decim = 1.f+u2*(CRUNCH_DECIM_MAX-1.f);
  }
  // LPF/HPF cutoffs given directly in Hz (0..LPF_MAX_HZ / 0..HPF_MAX_HZ), not
  // a 0..1 "brightness" param. damp = exp(-2*pi*fc/sr) per pole. ONE pole per
  // filter: a cascaded 2-pole version compounded into an audible notch when the
  // LPF and HPF cutoffs overlapped. So this damp value is directly the filter's
  // actual -3dB point, with no compounding to account for.
  inline void setLPF(float hz){
    if(hz<1.f)hz=1.f;   // avoid log(0)/divide issues at the very bottom of the knob
    float d=expf(-2.f*3.14159265f*hz/sampleRate);
    bwLP.setDamp(d);
  }
  inline void setHPF(float hz){
    if(hz<1.f)hz=1.f;
    float d=expf(-2.f*3.14159265f*hz/sampleRate);
    bwHP.setDamp(d);
  }
  inline void setDiffusion(float g){       // decay diffusion 1 (modulated aps)
    modL.g=g; modR.g=g;
    // g2 (ap2L/ap2R, second diffusion stage) tracks g directly, unclamped --
    // deliberately, to let pDiffusion's full DIFF_LO..DIFF_HI knob range sweep
    // g2 past Dattorro's own stable bound (~0.5) up toward 0.9, for testing
    // allpass behavior outside that range (increasing ring/resonance rather
    // than smearing, as g approaches 1.0). float throughout means this won't
    // overflow/hard-fail the way fixed-point would, but watch output level
    // as g climbs -- resonant gain through the loop increases with g.
    ap2L.g=g; ap2R.g=g;
  }
  inline void setPredelay(int samps){
    if(samps<0)samps=0; if(samps>PREDELAY_MAX)samps=PREDELAY_MAX;
    predSamps=samps;
  }

  inline void process(float x,float decay,float& outL,float& outR){
    x=crush.process(x);   // bitcrusher, first thing in the chain --
                           // before predelay, LPF/HPF, everything. Predelay
                           // stores the already-crunched signal, so the held/
                           // quantized character rides through pre-delay too.
    // pre-delay: write, read predSamps back (0 = passthrough), advance
    predelay.write(x);
    float xd=predelay.readInt(predSamps);
    predelay.advance();
    float v=bwHP.process(bwLP.process(xd));   // LPF then HPF, in series
    v=in1.process(v,0); v=in2.process(v,0);
    v=in3.process(v,0); v=in4.process(v,0);

#if MOD_SHAPE_TRIANGLE
    // triangle: constant slew rate, no fast zero-crossing like sine has.
    // tri(ph) in [-1,1] from a phase-folded ramp; cheap (fabsf, no sinf).
    float trL=(phL*(1.f/3.14159265f))-1.f;  float mL=depth1*(2.f*fabsf(trL)-1.f)*-1.f;
    float trR=(phR*(1.f/3.14159265f))-1.f;  float mR=depth1*(2.f*fabsf(trR)-1.f)*-1.f;
    float tr2L=(ph2L*(1.f/3.14159265f))-1.f; float m2L=depth2*(2.f*fabsf(tr2L)-1.f)*-1.f;
    float tr2R=(ph2R*(1.f/3.14159265f))-1.f; float m2R=depth2*(2.f*fabsf(tr2R)-1.f)*-1.f;
#else
    float mL =depth1*sinf(phL),  mR =depth1*sinf(phR);
    float m2L=depth2*sinf(ph2L), m2R=depth2*sinf(ph2R);
#endif
    phL+=dphi;   if(phL>6.2831853f)  phL-=6.2831853f;
    phR+=dphi;   if(phR>6.2831853f)  phR-=6.2831853f;
    ph2L+=dphi2; if(ph2L>6.2831853f) ph2L-=6.2831853f;
    ph2R+=dphi2; if(ph2R>6.2831853f) ph2R-=6.2831853f;

    float a=modL.process(v+fbR,mL);
    d1L.write(a); float da=d1L.readInt(D1L); d1L.advance();
    da=dampL.process(da)*decay;
    a=ap2L.process(da,m2L);
    d2L.write(a); float lo=d2L.readInt(D2L); d2L.advance();
    fbL=lo*decay;

    float b=modR.process(v+fbL,mR);
    d1R.write(b); float db=d1R.readInt(D1R); d1R.advance();
    db=dampR.process(db)*decay;
    b=ap2R.process(db,m2R);
    d2R.write(b); float ro=d2R.readInt(D2R); d2R.advance();
    fbR=ro*decay;

    outL = 0.6f*( d1R.tap(197)+d1R.tap(2204)-ap2R.dl.tap(2835)
                 +d2R.tap(2958)-d1L.tap(2949)-ap2L.dl.tap(277)-d2L.tap(1580) );
    outR = 0.6f*( d1L.tap(523)+d1L.tap(5375)-ap2L.dl.tap(1820)
                 +d2L.tap(3961)-d1R.tap(3128)-ap2R.dl.tap(496)-d2R.tap(179) );
  }
};
static Plate gPlate;

// ============================================================================
//  Shared params: UI core writes, audio core reads. volatile + word-sized.
// ============================================================================
// Page A (8 pots): decay, damp, LPF, HPF, crunch, predelay, diffusion, mix
// Page B (7 pots): in-gain, out-gain, rate1, depth1, rate2, depth2, midi-ch
volatile float pDecay      = 0.72f;   // Page A knob 0
volatile float pDamp       = 0.25f;   // Page A knob 1
volatile float pLPFHz      = LPF_MAX_HZ; // Page A knob 2: 0..LPF_MAX_HZ (see setLPF)
volatile float pHPFHz      = 0.0f;       // Page A knob 3: 0..HPF_MAX_HZ (see setHPF)
// Bitcrusher: one combined knob driving both bit-depth and sample-rate
// decimation. Curves live in Plate::setCrunch().
volatile float pCrunch     = 1.0f;    // Page A knob 4: 1=clean .. 0=max crunch
volatile float pPredelayMs = 0.0f;    // Page A knob 5: 0..240 ms
volatile float pDiffusion  = 0.70f;   // Page A knob 6: 0.25..0.90 (tank allpass
                                       // g1; g2 derived proportionally, see setDiffusion)
volatile float pMix        = 0.30f;   // Page A knob 7
// Bypass is serial-only ('b'/'B', see handleModKeys) -- the slider
// drives page select instead (see g_pageB below), fully decoupled from this.
volatile bool  pBypass     = false;
// Page B: in-gain, out-gain, then the 4 mod params. The mod params are
// NVS-persisted and bank-able; the two gains are live-only.
volatile float pInGain    = 1.0f;     // Page B knob 0
volatile float pOutGain   = 1.0f;     // Page B knob 1
volatile float pModRate1  = 0.40f;    // Page B knob 2: stage-1 LFO rate, Hz
volatile float pModDepth1 = 24.0f;    // Page B knob 3: stage-1 excursion, samples
volatile float pModRate2  = 0.27f;    // Page B knob 4: stage-2 LFO rate, Hz
volatile float pModDepth2 = 6.0f;     // Page B knob 5: stage-2 excursion, samples
// MIDI RX/TX channel (1..16), Page B knob 6, NVS-persisted. Config value,
// NOT a mod param -- it lives in the live NVS keys but stays OUT of the preset
// bank (the 4-slot bank only stores rate1/rate2/depth1/depth2). No CC controls it.
volatile uint8_t pMidiChannel = 1;    // Page B knob 6
// Slider position directly IS the page (on=B, off=A), read live in
// uiTask every pass -- no longer a toggled/latched flag driven by an edge
// detector. Declared volatile for consistency with the other UI-core-writes/
// audio-core-reads params, though nothing in the audio core currently reads it.
volatile bool  g_pageB     = false;   // false=Page A (main), true=Page B (mod)
volatile uint32_t g_blockCount = 0;   // audio core bumps; UI core reports
volatile uint32_t g_blockUs    = 0;   // us spent rendering one block

// blocks/s is pinned by I2S pacing: it reads ~172 at ANY load and only drops
// once you have already MISSED realtime. It is a cliff detector, not a headroom
// gauge. THIS is the headroom gauge: block render time vs the realtime deadline.
#define BLOCK_BUDGET_US 5805          // 256/44100 s

// ============================================================================
//  AUDIO CORE (loop) — touches NO I2C.
//  Buffer type + I/O calls copied from the reference sketch.
// ============================================================================
static output_sample_type s_extBuf[AMY_BLOCK_SIZE * AMY_NCHANS];

void processBlock(output_sample_type* b){
  uint32_t t0=(uint32_t)esp_timer_get_time();      // REAL load gauge
  float decay=pDecay, mix=pMix, ig=pInGain, og=pOutGain;
  bool bypass=pBypass;
  // apply coefficient params once per block (cheap; avoids a dirty-flag dance)
  gPlate.setDamp(pDamp);
  gPlate.setLPF(pLPFHz);
  gPlate.setHPF(pHPFHz);
  gPlate.setCrunch(pCrunch);
  gPlate.setDiffusion(pDiffusion);
  gPlate.setPredelay((int)(pPredelayMs*0.001f*AMY_SAMPLE_RATE));
  gPlate.setModRates(pModRate1,pModRate2);
  gPlate.setModDepths(pModDepth1,pModDepth2);

  for(int n=0;n<AMY_BLOCK_SIZE;n++){
    int16_t L=b[2*n], R=b[2*n+1];
    float dry=0.5f*((float)L+(float)R)*ig;
    // Always run the tank so its state stays LIVE — un-bypassing then resumes on
    // a real tail, not a frozen/dead one (the reverb analogue of the reference
    // keeping its tape recording while bypassed).
    float wL,wR; gPlate.process(dry,decay,wL,wR);
    float oL,oR;
    if(bypass){
      oL=oR=dry*og;                          // dry passthrough (mono to L+R)
    } else {
      oL=((1.f-mix)*dry + mix*wL)*og;
      oR=((1.f-mix)*dry + mix*wR)*og;
    }
    oL=oL>32767?32767:(oL<-32768?-32768:oL);
    oR=oR>32767?32767:(oR<-32768?-32768:oR);
    b[2*n]=(int16_t)oL; b[2*n+1]=(int16_t)oR;
  }
  g_blockCount++;
  g_blockUs=(uint32_t)esp_timer_get_time()-t0;   // time to render one block
}

void loop(){
  amy_get_input_buffer(s_extBuf);
  processBlock(s_extBuf);
  amy_set_external_input_buffer(s_extBuf);
  amy_update();                    // renders + I2S write; blocks ~1 block
}

// ============================================================================
//  8Angle pot / LED  (I2C shape copied from reference: stop condition, HW_CH)
// ============================================================================
uint16_t pot_raw[ANGLE8_POTS], pot_prev[ANGLE8_POTS];

// Dropped-read counters. Every 8Angle read below keeps its previous
// value (or a default) when requestFrom() returns short -- which is correct
// behaviour, but SILENT: a wedged bus is indistinguishable from a hand that
// isn't touching the knobs. Frozen pots, no error, nothing on screen. That's the
// same class of ambiguity as the constant-4095 trap (a dead link reading as
// "pot at max") that cost days during emulator bring-up. Counting the failures
// turns "is it stuck or am I imagining it?" into a number: dump with [c].
// Healthy bus = these stay 0 forever. UI-core only, so no atomics needed.
volatile uint32_t g_i2cPotDrops  = 0;   // readAllPots()  short/failed reads
volatile uint32_t g_i2cSwDrops   = 0;   // readToggle() + readSwitch2() failures
volatile uint32_t g_i2cPotReads  = 0;   // total attempts, for a rate not a raw count

void readAllPots(){
  for(int ch=0; ch<ANGLE8_POTS; ch++){
    uint8_t hw=HW_CH(ch);
    Wire.beginTransmission(ANGLE8_ADDR);
    Wire.write((uint8_t)(hw*2));
    Wire.endTransmission();                  // stop (not repeated-start)
    g_i2cPotReads++;
    if(Wire.requestFrom(ANGLE8_ADDR,2)==2){
      uint8_t lo=Wire.read(), hi=Wire.read();
      uint16_t v=(uint16_t)((hi<<8)|lo);
#if ANGLE8_INVERT_DIR
      v=(v>4095)?0:(4095-v);
#endif
      pot_raw[ch]=v;
    } else {
      g_i2cPotDrops++;                       // keep last value; just record it
    }
  }
}

void setLed(uint8_t ch,uint8_t r,uint8_t g,uint8_t b,uint8_t bright){
  uint8_t hw=HW_CH(ch);
  Wire.beginTransmission(ANGLE8_ADDR);
  Wire.write((uint8_t)(0x30+hw*4));
  Wire.write(r); Wire.write(g); Wire.write(b); Wire.write(bright);
  Wire.endTransmission(); 
}

// Latching slider on the 8Angle: register 0x20, bit 0. (From the reference.)
bool readToggle(){
  Wire.beginTransmission(ANGLE8_ADDR);
  Wire.write(ANGLE8_REG_SW1);
  Wire.endTransmission();
  if(Wire.requestFrom(ANGLE8_ADDR,1)==1) return (Wire.read()&1)!=0;
  g_i2cSwDrops++;   // note the drop. A failed read forces Page A below,
  return false;     // which is a REAL page flip, not just a stale value -- so a
}                   // climbing count here explains spontaneous page changes.

#if HAS_BYPASS_SWITCH
// Bypass toggle at register 0x22, compiled in only for the emulator build
// (HAS_BYPASS_SWITCH 1). Returns the switch level: true = closed (bypassed). A
// failed read holds the caller's last state (see uiTask) rather than flipping, so
// a momentary bus glitch can't toggle bypass. No presence detection here anymore
// -- the build flag already asserts the switch exists, so we simply trust 0x22.
static bool readSwitch2(){
  Wire.beginTransmission(ANGLE8_ADDR);
  Wire.write(ANGLE8_REG_SW2);
  Wire.endTransmission();
  if(Wire.requestFrom(ANGLE8_ADDR,1)==1) return (Wire.read()&1)!=0;
  g_i2cSwDrops++;   // count it; state is held, so this one is benign
  return pBypass;   // read failed -> keep current state (no glitch flip)
}
#endif

// ============================================================================
//  Library-free SH1106 driver (framebuffer + 5x7 font), from the reference.
// ============================================================================
#define OLED_W 128
#define OLED_H 64
#define OLED_PAGES (OLED_H/8)
#define SH1106_COL_OFFSET 2
static uint8_t s_fb[OLED_W*OLED_PAGES];
#include "eltro_font5x7.h"        // font5x7[(c-0x20)*5 + col]

static bool oledCmd(uint8_t c){
  Wire.beginTransmission(OLED_ADDR);
  Wire.write((uint8_t)0x00); Wire.write(c);
  return Wire.endTransmission()==0;
}
static void oledChar(int x,int y,char ch){
  if(ch<0x20||ch>0x7A) ch='?';
  const uint8_t* g=&font5x7[(ch-0x20)*5];
  for(int col=0;col<5;col++){
    uint8_t bits=g[col]; int px=x+col;
    if(px<0||px>=OLED_W) continue;
    for(int row=0;row<7;row++)
      if(bits&(1<<row)){ int py=y+row; if(py<0||py>=OLED_H)continue;
                         s_fb[px+(py/8)*OLED_W]|=(1<<(py&7)); }
  }
}
static int oledStr(int x,int y,const char* s){ while(*s){ oledChar(x,y,*s++); x+=6; } return x; }
static int oledStrWidth(const char* s){ int n=0; while(s[n])n++; return n*6-(n>0?1:0); }
static void oledClear(){ memset(s_fb,0,sizeof(s_fb)); }
static inline void oledPixel(int x,int y,bool on){
  if(x<0||x>=OLED_W||y<0||y>=OLED_H) return;
  uint8_t* p=&s_fb[x+(y/8)*OLED_W]; uint8_t m=1<<(y&7);
  if(on)*p|=m; else *p&=~m;
}
static void oledHLine(int x,int y,int w){ for(int i=0;i<w;i++) oledPixel(x+i,y,true); }
static void oledVLine(int x,int y,int h){ for(int i=0;i<h;i++) oledPixel(x,y+i,true); }
static void oledFrame(int x,int y,int w,int h){
  oledHLine(x,y,w); oledHLine(x,y+h-1,w); oledVLine(x,y,h); oledVLine(x+w-1,y,h);
}
static void oledBox(int x,int y,int w,int h){ for(int j=0;j<h;j++) oledHLine(x,y+j,w); }

static void oledFlush(){
  for(uint8_t page=0;page<OLED_PAGES;page++){
    if(!oledCmd(0xB0|page)) return;
    if(!oledCmd(0x00|(SH1106_COL_OFFSET&0x0F))) return;
    if(!oledCmd(0x10|(SH1106_COL_OFFSET>>4)))   return;
    const uint8_t* src=&s_fb[page*OLED_W];
    int col=0;
    while(col<OLED_W){
      Wire.beginTransmission(OLED_ADDR);
      Wire.write((uint8_t)0x40);
      int chunk=OLED_W-col; if(chunk>16)chunk=16;
      for(int i=0;i<chunk;i++) Wire.write(src[col+i]);
      if(Wire.endTransmission()!=0) return;
      col+=chunk;
    }
  }
}
static bool oledInit(){
  delay(50);
  static const uint8_t seq[]={
    0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,0xAD,0x8B,0xA1,0xC8,
    0xDA,0x12,0x81,0x80,0xD9,0x22,0xDB,0x40,0xA4,0xA6,0xAF };
  bool ok=true;
  for(uint8_t i=0;i<sizeof(seq);i++){ ok&=oledCmd(seq[i]); delay(2); }
  delay(50); return ok;
}

// horizontal fill gauge, norm 0..1. UNUSED -- bars were replaced by numeric
// readouts; kept deliberately in case a bar is ever wanted back.
__attribute__((unused)) static void drawGauge(int x,int y,int w,int h,float norm){
  oledFrame(x,y,w,h);
  if(norm<0)norm=0; if(norm>1)norm=1;
  int vw=(int)(norm*(w-2));
  if(vw>0) oledBox(x+1,y+1,vw,h-2);
}

// 4-char labels, panel-ordered. Page A row 1 = the three top-row knobs,
// row 2 = the four bottom-row knobs; DECAY is the slider, on its own line.
// NAMES[0] is drawn only on the Page-A slider line, which is centred and has the
// full 128px to itself -- so it can spell DECAY in full. The other seven stay
// 4-char because they must fit a 32px grid cell.
const char* NAMES[8]={"DECAY","DAMP","LPF","HPF","BITR","PDLY","DIFF","MIX"};
// Includes MCH (MIDI channel). Drawn in panel order, see drawDisplay.
const char* MODNAMES[7]={"IN","OUT","RAT1","DEP1","RAT2","DEP2","MCH"};

// ---- numeric readout --------------------------------------------------------
// The bars are gone. A pot's own physical position is the analog indicator, so a
// gauge duplicated it; a number instead fits in ~5 chars, which is what lets the
// screen be laid out to MATCH THE PANEL (3-wide row over 4-wide row) rather than
// as an arbitrary 2-column list. Numbers also carry units a bar never could.
// Defined further down with the console reporting; declared here for drawDisplay.
static inline float rt60_est(float decay);
static inline float onepole_hz(float pole);

// Compact value formatter: <=5 chars so a 4-column (32px) cell never overruns.
// Deliberately terser than the console's fmtHz -- "4.8k" not "4.8kHz".
static void fmtCellHz(char* out,int n,float hz){
  if(hz>=20000.f)     snprintf(out,n,"open");
  else if(hz>=9950.f) snprintf(out,n,"%.0fk",hz/1000.f);
  else if(hz>=1000.f) snprintf(out,n,"%.1fk",hz/1000.f);
  else                snprintf(out,n,"%.0f",hz);
}
// Seconds, <=5 chars: "0.35s" / "2.4s" / "12s" / "FRZ" at the freeze endpoint.
static void fmtCellSec(char* out,int n,float s){
  if(s>=20.f)      snprintf(out,n,"FRZ");
  else if(s>=10.f) snprintf(out,n,"%.0fs",s);
  else if(s>=1.f)  snprintf(out,n,"%.1fs",s);
  else             snprintf(out,n,"%.2fs",s);
}
// Abstract 0..1 params read as plain percent -- no unit is meaningful for them.
static void fmtCellPct(char* out,int n,float norm){
  if(norm<0.f)norm=0.f; if(norm>1.f)norm=1.f;
  snprintf(out,n,"%d%%",(int)(norm*100.f+0.5f));
}

// One grid cell: 4-char label on top, value beneath, both left-aligned at x.
// Two text lines per cell (7px glyphs) = 17px per grid row, which is what makes
// a 4-wide row readable at 32px per column.
// UNUSED -- all cells are centred now; kept as the left-aligned variant.
__attribute__((unused)) static void drawCell(int x,int y,const char* label,const char* val){
  oledStr(x,y,label);
  oledStr(x,y+9,val);
}
// Same cell, but label and value each centred on the column. Used for the 3-wide
// rows, which are additionally indented by half a cell (GRID_C3_X) so three cells
// sit centred over four -- matching how the panel's three top knobs sit centred
// over its four bottom ones.
static void drawCellC(int x,int y,const char* label,const char* val){
  oledStr(x+(32-oledStrWidth(label))/2,y,   label);
  oledStr(x+(32-oledStrWidth(val))  /2,y+9, val);
}
// Column origins: four cells span the full 128px; three cells are inset 16px.
#define GRID_C4_X(i) ((i)*32)
#define GRID_C3_X(i) (16+(i)*32)

// Preset-bank slot state. Only consumer is the serial bank machinery further
// down (the OLED footer that used to show the slot name was dropped).
#define BANK_SLOTS 4
static char s_slotName[BANK_SLOTS][16] = { "SLOT0","SLOT1","SLOT2","SLOT3" };
static int  s_lastSlot = -1;   // -1 = none loaded/saved yet this session

void drawDisplay(uint32_t blkPerSec,uint32_t cpuPct){
  oledClear();
  oledStr(0,1,g_pageB?"MOD PAGE":"DATTORRO");
  if(pBypass){ const char* badge="BYP"; oledStr(64-oledStrWidth(badge)/2,1,badge); }
  char hdr[16]; snprintf(hdr,sizeof(hdr),"cpu%lu%%",(unsigned long)cpuPct);
  oledStr(127-oledStrWidth(hdr),1,hdr);
  oledHLine(0,9,128);

  if(g_pageB){
    // Page B: laid out to MATCH THE PANEL. The physical top knob row is
    // IN LEVEL / OUT LEVEL / MOD RATE1; the bottom row is MOD DEPTH1 / MOD RATE2
    // / MOD DEPTH2 / MIDI CH. Param indices are unchanged (0..6, still
    // parameter-ordered everywhere else) -- only this draw order is geometric.
    //   row 1 (3 cells): 0 IN    1 OUT   2 RAT1
    //   row 2 (4 cells): 3 DEP1  4 RAT2  5 DEP2  6 MCH
    char v[12];
    // -- top row: three cells, inset half a cell so they sit centred over the
    //    four below, exactly as the panel's three top knobs sit over its four.
    fmtCellPct(v,sizeof(v),(pInGain-0.2f)/2.8f);   drawCellC(GRID_C3_X(0),13,MODNAMES[0],v);
    fmtCellPct(v,sizeof(v),(pOutGain-0.2f)/1.8f);  drawCellC(GRID_C3_X(1),13,MODNAMES[1],v);
    snprintf(v,sizeof(v),"%.2f",pModRate1);        drawCellC(GRID_C3_X(2),13,MODNAMES[2],v);
    // -- bottom row: four cells. Depths are in samples (integer, small); rates in Hz.
    snprintf(v,sizeof(v),"%.0f",pModDepth1);       drawCellC(GRID_C4_X(0),35,MODNAMES[3],v);
    snprintf(v,sizeof(v),"%.2f",pModRate2);        drawCellC(GRID_C4_X(1),35,MODNAMES[4],v);
    snprintf(v,sizeof(v),"%.0f",pModDepth2);       drawCellC(GRID_C4_X(2),35,MODNAMES[5],v);
    snprintf(v,sizeof(v),"%d",(int)pMidiChannel);  drawCellC(GRID_C4_X(3),35,MODNAMES[6],v);
    // No "slot:NAME" footer here by design: it would go stale the instant a mod
    // knob moved (no dirty tracking) and the bank is serial-only anyway.
  } else {
    // Page A: DECAY is the slide pot and sits alone above the grid, as it
    // does on the panel. Then the panel's own two knob rows:
    //   row 1 (3 cells): DAMP  LPF   HPF
    //   row 2 (4 cells): BITR  PDLY  DIFF  MIX
    char v[12];
    // -- slider line: DECAY spans the full width, so label + value are centred
    //    together as one group (the slide pot spans the panel the same way).
    fmtCellSec(v,sizeof(v),rt60_est(pDecay));
    {
      int lw=oledStrWidth(NAMES[0]), vw=oledStrWidth(v);
      int x0=(128-(lw+6+vw))/2;              // 6px gap between label and value
      oledStr(x0,12,NAMES[0]); oledStr(x0+lw+6,12,v);
    }
    // -- top knob row: three cells, inset half a cell (centred over the four below)
    fmtCellPct(v,sizeof(v),pDamp/0.93f);        drawCellC(GRID_C3_X(0),24,NAMES[1],v);
    fmtCellHz (v,sizeof(v),pLPFHz);             drawCellC(GRID_C3_X(1),24,NAMES[2],v);
    fmtCellHz (v,sizeof(v),pHPFHz);             drawCellC(GRID_C3_X(2),24,NAMES[3],v);
    // -- bottom knob row: four cells, full width
    fmtCellPct(v,sizeof(v),pCrunch);            drawCellC(GRID_C4_X(0),44,NAMES[4],v);
    snprintf(v,sizeof(v),"%.0fms",pPredelayMs); drawCellC(GRID_C4_X(1),44,NAMES[5],v);
    fmtCellPct(v,sizeof(v),(pDiffusion-DIFF_LO)/(DIFF_HI-DIFF_LO));
                                                drawCellC(GRID_C4_X(2),44,NAMES[6],v);
    fmtCellPct(v,sizeof(v),pMix);               drawCellC(GRID_C4_X(3),44,NAMES[7],v);
  }
  if(pBypass) oledBox(0,62,128,2);   // bottom bypass bar (like the reference)
  oledFlush();
}

// ============================================================================
//  UI CORE (core 0) — sole owner of the I2C bus.
// ============================================================================
int lastRaw[8]={-99,-99,-99,-99,-99,-99,-99,-99};

// ============================================================================
//  PARAMETER MAPPING — normalized cores shared by knobs AND MIDI.
// ============================================================================
// Every param's law is factored into a *FromNorm(t) core taking a normalized
// t in [0,1]. The physical pot (t = raw/4095) and an incoming MIDI CC
// (t = cc/127) BOTH map through exactly this same math, so a CC and the
// equivalent knob position produce identical parameter values. The two
// applyPot* functions below now just call these; nothing about the knob
// behavior changed, the math simply moved here so MIDI can share it.
//
// Flat param index (used for MIDI CC offset + per-param ownership):
//   0..7  = Page A knobs 0..7 (decay,damp,LPF,HPF,crunch,predelay,diffusion,mix)
//   8..13 = Page B knobs 0..5 (in-gain,out-gain,rate1,depth1,rate2,depth2)
// (Page B knob 6 = MIDI channel has no flat index here -- it's knob-only.)
#define PID_PAGEA_BASE 0
#define PID_PAGEB_BASE 8

// -- Page A cores --
static inline float decayFromNorm(float t){ return 0.20f + 0.78f*powf(t,DECAY_GAMMA); } // -> RT60 (gamma-tapered, see DECAY_GAMMA)
static inline float dampFromNorm (float t){ return 0.0005f + 0.93f*t; }      // HF damping
static inline float lpfFromNorm  (float t){ return t*LPF_MAX_HZ; }           // 0..LPF_MAX_HZ
static inline float hpfFromNorm  (float t){ return t*HPF_MAX_HZ; }           // 0..HPF_MAX_HZ
static inline float crunchFromNorm(float t){ return t; }                     // 1=clean..0=max
static inline float predelayMsFromNorm(float t){ return 240.f*t; }           // 0..240 ms
static inline float diffusionFromNorm(float t){ return DIFF_LO+(DIFF_HI-DIFF_LO)*t; }
static inline float mixFromNorm  (float t){ return t; }                      // wet fraction
// -- Page B cores -- (ranges match the serial-key ceilings so all control
//    paths land in the same space: rate 0..5 Hz, depth 0..EXC_MAX/EXC2_MAX)
static inline float inGainFromNorm (float t){ return 0.2f + t*2.8f; }         // 0.2..3.0
static inline float outGainFromNorm(float t){ return 0.2f + t*1.8f; }         // 0.2..2.0
static inline float rate1FromNorm  (float t){ return t*5.0f; }               // 0..5 Hz
static inline float depth1FromNorm (float t){ return t*(float)EXC_MAX; }     // 0..EXC_MAX
static inline float rate2FromNorm  (float t){ return t*5.0f; }               // 0..5 Hz
static inline float depth2FromNorm (float t){ return t*(float)EXC2_MAX; }    // 0..EXC2_MAX
// MIDI channel core (knob-only, no CC): 1..16 from t.
static inline uint8_t midiChanFromNorm(float t){
  int v=(int)(1.0f + t*15.999f); if(v<1)v=1; if(v>16)v=16; return (uint8_t)v;
}

// ---- MIDI ownership: last-writer-wins, per parameter ----------------------
// One flag per flat param (0..13). A received CC sets ownership to MIDI for
// that param; a physical knob move (past POT_DEADBAND, detected in uiTask)
// clears it back to knob. No catch threshold -- whichever source produced the
// most recent real event owns the value. Independent per param: a CC on decay
// doesn't touch who owns mix. Written by the UI core only (MIDI RX + pot read
// both run there), so no cross-core race.
static bool s_midiOwned[MIDI_NUM_PARAMS] = { false };

// Single source of truth for Page-A knob -> param mapping. Used at boot (so
// params match the PHYSICAL knob positions rather than the compile-time
// defaults) and on every move thereafter. Routes through the
// shared *FromNorm cores above.
void applyPotPageA(int c,uint16_t raw){
  float f=raw/4095.f;
  switch(c){
    case 0: pDecay     = decayFromNorm(f); break;      // loop feedback gain -> RT60
    case 1: pDamp      = dampFromNorm(f); break;       // HF damping (in-loop)
    case 2: pLPFHz     = lpfFromNorm(f); break;        // LPF cutoff, 0..LPF_MAX_HZ
    case 3: pHPFHz     = hpfFromNorm(f); break;        // HPF cutoff, 0..HPF_MAX_HZ
    case 4: pCrunch    = crunchFromNorm(f); break;     // bitcrusher, 0=max..1=clean
    case 5: pPredelayMs= predelayMsFromNorm(f); break; // pre-delay 0..240 ms
    case 6: pDiffusion = diffusionFromNorm(f); break;  // tank allpass diffusion
    case 7: pMix       = mixFromNorm(f); break;        // wet fraction
  }
}

// ============================================================================
//  Page B: tank modulation rate/depth, NVS-persisted.
//  Pattern (catch/pickup + debounced NVS save) taken from the Eltro V12
//  reference (eltro_amyboard_v12.ino) — same mechanism, trimmed to 4 knobs
//  and 4 float params instead of 8.
// ============================================================================
static Preferences s_prefs;

#define PAGEB_POTS         7        // in-gain, out-gain,
                                     // rate1, depth1, rate2, depth2, MIDI-channel.
// Pot 0 is a SLIDE potentiometer dedicated
// to Page-A decay; on Page B it does nothing, so all 7 Page-B params sit on the
// seven ROTARY pots 1..7. This is the single source of truth for that shift:
// Page-B parameter i (0..6) is physically read from pot_raw[i + PAGEB_POT0].
// The catch arrays, CC map, flat param IDs, ownership flags and OLED all remain
// PARAMETER-ordered (0..6) -- only the physical pot lookup is offset -- so a
// DAW's CC110 is still in-gain regardless of which pot carries it.
#define PAGEB_POT0         1        // first Page-B param lives on this physical pot
#define PAGEB_CATCH_THRESH 40       // raw ADC move (of 4095) to "catch" a pot
#define PAGEA_PICKUP_THRESH 40      // raw ADC move (of 4095) to "pick up" a pot

static bool     s_pbCaught[PAGEB_POTS] = { false,false,false,false,false,false,false };
static uint16_t s_pbRefRaw[PAGEB_POTS] = { 0 };
static bool     s_pbDirty              = false;  // unsaved changes pending
static uint32_t s_pbDirtySince         = 0;       // millis of first pending change

// V12-style pickup on Page-A re-entry: Page A is LIVE at boot (knobs take
// their physical position immediately), but re-entering Page A after a visit
// to Page B holds each knob until moved past PAGEA_PICKUP_THRESH, so a knob
// sitting somewhere else physically doesn't snap pDecay/pMix/etc. on return.
static bool     s_pageAArmed             = false;  // false at boot -> live
static bool     s_paPicked[8]            = { false };
static uint16_t s_paRefRaw[8]            = { 0 };

// Load persisted Page-B values (or leave compile-time defaults if never saved).
static void loadPageBSettings(){
  s_prefs.begin("dattorro",true);   // read-only
  pModRate1  = s_prefs.getFloat("mrate1", pModRate1);
  pModRate2  = s_prefs.getFloat("mrate2", pModRate2);
  pModDepth1 = s_prefs.getFloat("mdepth1",pModDepth1);
  pModDepth2 = s_prefs.getFloat("mdepth2",pModDepth2);
  pMidiChannel = s_prefs.getUChar("mch", pMidiChannel);  // RX/TX channel
  s_prefs.end();
  // re-clamp: EXC_MAX/EXC2_MAX are fixed at compile time and could in theory
  // differ from a value saved by an older build with looser ceilings.
  if(pModRate1<0.f)pModRate1=0.f;  if(pModRate1>5.f)pModRate1=5.f;
  if(pModRate2<0.f)pModRate2=0.f;  if(pModRate2>5.f)pModRate2=5.f;
  if(pModDepth1<0.f)pModDepth1=0.f; if(pModDepth1>EXC_MAX) pModDepth1=EXC_MAX;
  if(pModDepth2<0.f)pModDepth2=0.f; if(pModDepth2>EXC2_MAX)pModDepth2=EXC2_MAX;
  if(pMidiChannel<1)pMidiChannel=1; if(pMidiChannel>16)pMidiChannel=16;
}

// Persist current Page-B values to flash.
static void savePageBSettings(){
  s_prefs.begin("dattorro",false);  // read-write
  s_prefs.putFloat("mrate1", pModRate1);
  s_prefs.putFloat("mrate2", pModRate2);
  s_prefs.putFloat("mdepth1",pModDepth1);
  s_prefs.putFloat("mdepth2",pModDepth2);
  s_prefs.putUChar("mch", pMidiChannel);   // RX/TX channel (config, not banked)
  s_prefs.end();
  s_pbDirty=false;
}

// ============================================================================
//  Page-B preset bank: 4 named slots, each holding the 4 mod params.
//  Separate from the "live" mrate1/mrate2/mdepth1/mdepth2 keys above -- those
//  always hold whatever you last left the knobs at; slots are for named
//  alternatives you explicitly save/load, same NVS namespace ("dattorro"),
//  short keys since Preferences caps key names at 15 chars.
//    p<N>r1 / p<N>r2 / p<N>d1 / p<N>d2 / p<N>nm   for slot N = 0..3
//  s_slotName/s_lastSlot/BANK_SLOTS are declared earlier (with drawDisplay)
//  so the Page-B OLED footer can show the current slot without a fwd-decl.
// ============================================================================

// Save current Page-B state into slot `n`. Optional name (nullptr/empty keeps
// existing name, or the SLOTn default on first save).
static void saveBankSlot(int n, const char* name=nullptr){
  if(n<0 || n>=BANK_SLOTS) return;
  if(name && name[0]){
    strncpy(s_slotName[n], name, sizeof(s_slotName[n])-1);
    s_slotName[n][sizeof(s_slotName[n])-1]='\0';
  }
  char key[16];
  s_prefs.begin("dattorro",false);
  snprintf(key,sizeof(key),"p%dr1",n); s_prefs.putFloat(key, pModRate1);
  snprintf(key,sizeof(key),"p%dr2",n); s_prefs.putFloat(key, pModRate2);
  snprintf(key,sizeof(key),"p%dd1",n); s_prefs.putFloat(key, pModDepth1);
  snprintf(key,sizeof(key),"p%dd2",n); s_prefs.putFloat(key, pModDepth2);
  snprintf(key,sizeof(key),"p%dnm",n); s_prefs.putString(key, s_slotName[n]);
  s_prefs.end();
  s_lastSlot=n;
  Serial.printf("Saved slot %d '%s': r1=%.2f r2=%.2f d1=%.1f d2=%.1f\n",
                n, s_slotName[n], pModRate1, pModRate2, pModDepth1, pModDepth2);
}

// Load slot `n` into the live mod params (through the same clamp path as
// knobs/serial keys -- setModRates/setModDepths re-clamp every block anyway,
// but clamping here too means a corrupted/out-of-range saved value can't even
// transiently misbehave).
static void loadBankSlot(int n){
  if(n<0 || n>=BANK_SLOTS) return;
  char key[16];
  s_prefs.begin("dattorro",true);
  snprintf(key,sizeof(key),"p%dr1",n); float r1=s_prefs.getFloat(key, pModRate1);
  snprintf(key,sizeof(key),"p%dr2",n); float r2=s_prefs.getFloat(key, pModRate2);
  snprintf(key,sizeof(key),"p%dd1",n); float d1=s_prefs.getFloat(key, pModDepth1);
  snprintf(key,sizeof(key),"p%dd2",n); float d2=s_prefs.getFloat(key, pModDepth2);
  snprintf(key,sizeof(key),"p%dnm",n);
  String nm=s_prefs.getString(key, s_slotName[n]);
  s_prefs.end();
  strncpy(s_slotName[n], nm.c_str(), sizeof(s_slotName[n])-1);
  s_slotName[n][sizeof(s_slotName[n])-1]='\0';
  if(r1<0.f)r1=0.f; if(r1>5.f)r1=5.f;
  if(r2<0.f)r2=0.f; if(r2>5.f)r2=5.f;
  if(d1<0.f)d1=0.f; if(d1>EXC_MAX) d1=EXC_MAX;
  if(d2<0.f)d2=0.f; if(d2>EXC2_MAX)d2=EXC2_MAX;
  pModRate1=r1; pModRate2=r2; pModDepth1=d1; pModDepth2=d2;
  s_lastSlot=n;
  s_pbDirty=true; s_pbDirtySince=millis();   // loaded state becomes the new "live" NVS value too
  Serial.printf("Loaded slot %d '%s': r1=%.2f r2=%.2f d1=%.1f d2=%.1f\n",
                n, s_slotName[n], pModRate1, pModRate2, pModDepth1, pModDepth2);
}

// Print all 4 slots' saved values without loading them. Reads whatever names
// are cached in s_slotName (populated by prior loads this session, or by
// loadBankNames() at boot) plus the persisted numeric values from NVS.
static void printBank(){
  char key[16];
  s_prefs.begin("dattorro",true);
  Serial.println("Bank:");
  for(int n=0;n<BANK_SLOTS;n++){
    snprintf(key,sizeof(key),"p%dr1",n); float r1=s_prefs.getFloat(key, -1.f);
    if(r1<0.f){ Serial.printf("  [%d] %-8s (empty)\n", n, s_slotName[n]); continue; }
    snprintf(key,sizeof(key),"p%dr2",n); float r2=s_prefs.getFloat(key, 0.f);
    snprintf(key,sizeof(key),"p%dd1",n); float d1=s_prefs.getFloat(key, 0.f);
    snprintf(key,sizeof(key),"p%dd2",n); float d2=s_prefs.getFloat(key, 0.f);
    Serial.printf("  [%d] %-8s r1=%.2f r2=%.2f d1=%.1f d2=%.1f%s\n",
                  n, s_slotName[n], r1, r2, d1, d2, n==s_lastSlot?"  <-- current":"");
  }
  s_prefs.end();
}

// Load just the 4 slot names from NVS at boot, so printBank()/the OLED footer
// show real names before any slot has been touched this session.
static void loadBankNames(){
  char key[16];
  s_prefs.begin("dattorro",true);
  for(int n=0;n<BANK_SLOTS;n++){
    snprintf(key,sizeof(key),"p%dnm",n);
    String nm=s_prefs.getString(key, s_slotName[n]);
    strncpy(s_slotName[n], nm.c_str(), sizeof(s_slotName[n])-1);
    s_slotName[n][sizeof(s_slotName[n])-1]='\0';
  }
  s_prefs.end();
}

// Re-arm Page-B catch: all 4 Page-B pots become "waiting" at their current
// reading. Called on boot and whenever Page B is (re)entered, so the physical
// knob position doesn't overwrite the persisted value until you grab it.
static void rearmPageBCatch(){
  // param i (0..6) reads physical pot i+PAGEB_POT0 (slide-pot layout)
  for(int i=0;i<PAGEB_POTS;i++){ s_pbCaught[i]=false; s_pbRefRaw[i]=pot_raw[i+PAGEB_POT0]; }
}

// Re-arm Page-A pickup: mirrors rearmPageBCatch for the 8 Page-A knobs. NOT
// called on boot (Page A is live at power-on); called only on Page-A re-entry.
static void rearmPageACatch(){
  s_pageAArmed=true;
  for(int i=0;i<8;i++){ s_paPicked[i]=false; s_paRefRaw[i]=pot_raw[i]; }
}

// Returns true if Page-B parameter `p` (0..6) may drive its value: already
// caught, or its physical pot (p+PAGEB_POT0) just moved past the catch
// threshold (latched caught now). Catch state stays parameter-indexed; only the
// pot_raw lookup carries the slide-pot offset.
static bool pageBCaught(int p){
  if(s_pbCaught[p]) return true;
  int d=(int)pot_raw[p+PAGEB_POT0]-(int)s_pbRefRaw[p]; if(d<0)d=-d;
  if(d>PAGEB_CATCH_THRESH){ s_pbCaught[p]=true; return true; }
  return false;
}

// Returns true if Page-A pot `ch` may drive its parameter (pickup gate).
static bool potDrivesPageA(int ch){
  if(!s_pageAArmed) return true;         // live at boot
  if(s_paPicked[ch]) return true;
  int d=(int)pot_raw[ch]-(int)s_paRefRaw[ch]; if(d<0)d=-d;
  if(d>PAGEA_PICKUP_THRESH){ s_paPicked[ch]=true; return true; }
  return false;
}

// Page-B knob -> mod param mapping. Ranges match the serial-key ceilings from
// the serial keys (rate 0..5 Hz, depth 0..EXC_MAX/EXC2_MAX) so both paths land
// in the same space. Each param updates only once its pot is caught, and any
// change marks settings dirty for the debounced NVS save in uiTask.
void applyPotPageB(){
  // each param i (0..6) reads physical pot i+PAGEB_POT0 -- the slide
  // pot (pot 0) is Page-A decay and does nothing here, so params sit on the
  // seven rotaries (pots 1..7). pageBCaught(i) already applies the same offset.
  // Knob 6 (MIDI channel) has no CC/ownership; it's knob-only, still NVS-saved.
  float f;
  if(pageBCaught(0) && !s_midiOwned[PID_PAGEB_BASE+0]){ f=pot_raw[0+PAGEB_POT0]/4095.f;
    float v=inGainFromNorm(f);  if(v!=pInGain){ pInGain=v; } }
  if(pageBCaught(1) && !s_midiOwned[PID_PAGEB_BASE+1]){ f=pot_raw[1+PAGEB_POT0]/4095.f;
    float v=outGainFromNorm(f); if(v!=pOutGain){ pOutGain=v; } }
  if(pageBCaught(2) && !s_midiOwned[PID_PAGEB_BASE+2]){ f=pot_raw[2+PAGEB_POT0]/4095.f;
    float v=rate1FromNorm(f);   if(v!=pModRate1){ pModRate1=v; s_pbDirty=true; } }
  if(pageBCaught(3) && !s_midiOwned[PID_PAGEB_BASE+3]){ f=pot_raw[3+PAGEB_POT0]/4095.f;
    float v=depth1FromNorm(f);  if(v!=pModDepth1){ pModDepth1=v; s_pbDirty=true; } }
  if(pageBCaught(4) && !s_midiOwned[PID_PAGEB_BASE+4]){ f=pot_raw[4+PAGEB_POT0]/4095.f;
    float v=rate2FromNorm(f);   if(v!=pModRate2){ pModRate2=v; s_pbDirty=true; } }
  if(pageBCaught(5) && !s_midiOwned[PID_PAGEB_BASE+5]){ f=pot_raw[5+PAGEB_POT0]/4095.f;
    float v=depth2FromNorm(f);  if(v!=pModDepth2){ pModDepth2=v; s_pbDirty=true; } }
  if(pageBCaught(6)){ f=pot_raw[6+PAGEB_POT0]/4095.f;
    uint8_t v=midiChanFromNorm(f); if(v!=pMidiChannel){ pMidiChannel=v; s_pbDirty=true; } }
  if(s_pbDirty && s_pbDirtySince==0) s_pbDirtySince=millis();
}

// Page-aware dispatcher: Page A drives 8 knobs (with pickup gating once
// armed); Page B drives all of its knobs (in-gain, out-gain, the 4 mod
// params, then MIDI channel), with catch gating.
void applyPot(int c,uint16_t raw){
  if(g_pageB) return;   // Page B is applied in bulk via applyPotPageB()
  if(potDrivesPageA(c)) applyPotPageA(c,raw);
}

// ============================================================================
//  MIDI engine — RX + TX, all params, fully interchangeable w/ knobs.
//  RX: a running-status-aware byte parser feeds Control Change on our channel
//  into midiHandleCC(), which maps the CC through the SAME *FromNorm core as
//  the matching knob and marks that param MIDI-owned. TX: midiTxPageParam()
//  emits a knob's CC when it PHYSICALLY moves (called from uiTask), skipping
//  MIDI-owned params so an incoming CC is never echoed. Ownership is
//  last-writer-wins: a CC grabs it, a physical knob move (which clears the flag
//  in uiTask) takes it back. Both RX and TX run on the UI core, sole owner of
//  Serial1, so there is no cross-core contention on the port or the flags.
// ============================================================================

// Set param `pid` (flat 0..13) from a normalized t in [0,1], through the shared
// core, and mark it MIDI-owned. Page-A params write their volatile directly
// (independent of the knob-pickup gate -- MIDI is its own owner). Page-B mod
// params also flag the NVS-dirty timer so a CC-driven change still persists.
static void midiSetParamFromNorm(uint8_t pid, float t){
  switch(pid){
    // -- Page A --
    case 0: pDecay      = decayFromNorm(t);      break;
    case 1: pDamp       = dampFromNorm(t);       break;
    case 2: pLPFHz      = lpfFromNorm(t);        break;
    case 3: pHPFHz      = hpfFromNorm(t);        break;
    case 4: pCrunch     = crunchFromNorm(t);     break;
    case 5: pPredelayMs = predelayMsFromNorm(t); break;
    case 6: pDiffusion  = diffusionFromNorm(t);  break;
    case 7: pMix        = mixFromNorm(t);        break;
    // -- Page B -- (gains live-only; the 4 mod params persist to NVS)
    case 8:  pInGain    = inGainFromNorm(t);     break;
    case 9:  pOutGain   = outGainFromNorm(t);    break;
    case 10: pModRate1  = rate1FromNorm(t);  s_pbDirty=true; break;
    case 11: pModDepth1 = depth1FromNorm(t); s_pbDirty=true; break;
    case 12: pModRate2  = rate2FromNorm(t);  s_pbDirty=true; break;
    case 13: pModDepth2 = depth2FromNorm(t); s_pbDirty=true; break;
    default: return;
  }
  s_midiOwned[pid]=true;
  if(s_pbDirty && s_pbDirtySince==0) s_pbDirtySince=millis();
}

static void midiHandleCC(uint8_t cc, uint8_t val){
  if(cc==MIDI_CC_BYPASS){                 // discrete bypass toggle (>=64 = on)
    bool nb=(val>=64);
    if(nb!=pBypass){ pBypass=nb; Serial.printf("Bypass: %s (MIDI)\n", nb?"ON":"OFF"); }
    return;
  }
  if(cc==MIDI_CC_PAGE){                    // page toggle mirror (>=64 -> Page B)
    // Note: the slider still physically drives the page every uiTask pass, so a
    // MIDI page change only "sticks" if the slider agrees. Kept for parity with
    // the Eltro CC map; harmless when the slider is the real authority.
    bool nb=(val>=64);
    if(nb!=g_pageB){ Serial.printf("Page (MIDI CC65): %s (slider overrides)\n", nb?"B":"A"); }
    return;
  }
  if(cc<MIDI_CC_BASE || cc>=MIDI_CC_BASE+MIDI_NUM_PARAMS) return;  // unmapped
  uint8_t pid=cc-MIDI_CC_BASE;             // flat param index 0..13
  midiSetParamFromNorm(pid, val/127.0f);   // same law as the knob
}

// -- Running-status-aware byte parser (real-time bytes skipped; CC dispatched
//    on our channel only; other channel-voice msgs parsed for framing, ignored).
static uint8_t s_midiStatus  = 0;   // last channel-voice status (0 = none)
static uint8_t s_midiData[2];
static uint8_t s_midiDataIdx = 0;
static uint8_t s_midiExpect  = 0;
static inline uint8_t midiDataLen(uint8_t status){
  switch(status&0xF0){ case 0xC0: case 0xD0: return 1; default: return 2; }
}
static void midiFeedByte(uint8_t b){
  if(b>=0xF8) return;                       // real-time: ignore, don't disturb state
  if(b>=0x80){                              // status byte
    if(b<0xF0){ s_midiStatus=b; s_midiExpect=midiDataLen(b); s_midiDataIdx=0; }
    else      { s_midiStatus=0; s_midiDataIdx=0; }   // system-common clears running status
    return;
  }
  if(s_midiStatus==0) return;               // data byte with no status: drop
  s_midiData[s_midiDataIdx++]=b;
  if(s_midiDataIdx>=s_midiExpect){
    s_midiDataIdx=0;                        // ready for next (running-status) msg
    uint8_t hi=s_midiStatus&0xF0;
    uint8_t ch=(s_midiStatus&0x0F)+1;       // 1..16
    if(hi==0xB0 && ch==pMidiChannel) midiHandleCC(s_midiData[0], s_midiData[1]);
  }
}

// -- TX: CC number for each flat param, and per-param last-sent 7-bit value ---
// Value sent = raw knob POSITION (raw>>5), NOT the mapped value: RX re-applies
// the mapping law identically on the way back in, so a DAW round-trips a knob to
// the exact same sound, and we avoid the lossy flat-spots some laws have.
static uint8_t s_lastTx[MIDI_NUM_PARAMS];
static bool    s_txPrimed[MIDI_NUM_PARAMS] = { false };  // per-param baseline seeded?
static inline void midiSendCC(uint8_t cc, uint8_t val){
  Serial1.write((uint8_t)(0xB0 | ((pMidiChannel-1)&0x0F)));
  Serial1.write((uint8_t)(cc & 0x7F));
  Serial1.write((uint8_t)(val & 0x7F));
}
// Emit `pid`'s CC for a physical move. Skips MIDI-owned params (anti-echo) and
// dead-bands on the 7-bit value so a resting/wiggling pot doesn't spew CCs. The
// first call for a given pid just seeds its baseline (sends nothing) so a knob
// resting at power-on doesn't fire a CC before it's actually touched. Called
// from uiTask AFTER ownership was cleared for this pid on the move.
static void midiTxPageParam(uint8_t pid, uint16_t raw){
  if(pid>=MIDI_NUM_PARAMS) return;
  if(s_midiOwned[pid]) return;              // never echo a MIDI-driven param
  uint8_t v=(uint8_t)(raw>>5);              // 0..4095 -> 0..127 (knob position)
  if(!s_txPrimed[pid]){ s_lastTx[pid]=v; s_txPrimed[pid]=true; return; }  // seed only
  if(v==s_lastTx[pid]) return;              // deadband: only on real 7-bit change
  s_lastTx[pid]=v;
  midiSendCC(MIDI_CC_BASE+pid, v);
}

// ---- derived units (so the console reports physics, not coefficients) ------
// RT60: measured offline against this exact tank, fits RT60 ~= 1.34 / -ln(decay)
// to within a few % over decay 0.5..0.94. Estimate only -- damping shortens it.
static inline float rt60_est(float decay){
  if(decay<=0.f)   return 0.f;
  if(decay>=0.999f)return 999.f;
  return 1.34f/(-logf(decay));
}
// One-pole lowpass with pole `a` has -3 dB at fc = -ln(a)*SR/(2*pi).
static inline float onepole_hz(float pole){
  if(pole<=0.f)    return 99000.f;      // no filtering
  if(pole>=0.9999f)return 0.f;
  return -logf(pole)*(float)AMY_SAMPLE_RATE/6.2831853f;
}
static void fmtHz(char* out,int n,float hz){
  if(hz>=20000.f)     snprintf(out,n,"open");
  else if(hz>=1000.f) snprintf(out,n,"%.1fkHz",hz/1000.f);
  else                snprintf(out,n,"%.0fHz",hz);
}

// Full parameter dump in musical units. Printed on settle (after you stop
// turning), so one clean line per adjustment instead of spam per LSB.
void printParams(){
  char dampHz[12], lpfS[12], hpfS[12], rtS[12];
  fmtHz(dampHz,sizeof(dampHz),onepole_hz(pDamp));
  fmtHz(lpfS,sizeof(lpfS),pLPFHz);   // already in Hz, no onepole_hz needed
  fmtHz(hpfS,sizeof(hpfS),pHPFHz);
  float rt=rt60_est(pDecay);
  if(rt>20.f)  snprintf(rtS,sizeof(rtS),">20s");   // fit measured only to ~0.94;
  else         snprintf(rtS,sizeof(rtS),"%.1fs",rt); // beyond that it's a freeze

  Serial.printf(
    "PARAMS  PAGE %s | DECAY %.3f (~RT60 %s) | DAMP %.3f (~%s) | MIX %.2f | "
    "IN %.2f | OUT %.2f | PREDLY %.0fms | LPF %s | HPF %s | CRUNCH %.2f | "
    "DIFF %.2f | MOD1 %.1fsmp@%.2fHz | MOD2 %.1fsmp@%.2fHz | MIDIch %d%s\n",
    g_pageB?"B":"A", pDecay, rtS, pDamp, dampHz, pMix, pInGain, pOutGain,
    pPredelayMs, lpfS, hpfS, pCrunch, pDiffusion,
    pModDepth1, pModRate1, pModDepth2, pModRate2, (int)pMidiChannel,
    pBypass ? "  [BYPASSED]" : "");
  // The paste-back "defaults:" line is NOT printed here -- it's a source-editing
  // aid, not something you read while tuning. See printDefaults(), key [d].
}

// Paste-back form: drop over the defaults in the source to pin a setting you like.
// On demand only ([d]); deliberately not printed on every knob settle.
void printDefaults(){
  Serial.printf(
    "  defaults: pDecay=%.3ff; pDamp=%.4ff; pMix=%.3ff; pInGain=%.3ff; "
    "pOutGain=%.3ff; pPredelayMs=%.1ff; pLPFHz=%.1ff; pHPFHz=%.1ff; "
    "pCrunch=%.3ff; pDiffusion=%.3ff; "
    "pModRate1=%.3ff; pModRate2=%.3ff; pModDepth1=%.2ff; pModDepth2=%.2ff;\n",
    pDecay, pDamp, pMix, pInGain, pOutGain, pPredelayMs, pLPFHz, pHPFHz,
    pCrunch, pDiffusion,
    pModRate1, pModRate2, pModDepth1, pModDepth2);
}

// ---- serial keypress control for tank modulation ---------------------------
// Single-keypress nudges, non-blocking (reads whatever's queued, no wait).
// Kept off the 8-knob table since all 8 slots are already spoken for.
static const float MOD_RATE_STEP  = 0.05f;  // Hz per keypress
static const float MOD_DEPTH_STEP = 1.0f;   // samples per keypress

static void printModHelp(){
  Serial.println(
    "MOD keys: [1]/[!] rate1 -/+   [2]/[@] rate2 -/+   "
    "[q]/[Q] depth1 -/+   [w]/[W] depth2 -/+   [m] show mod   [h] this help");
  Serial.println(
    "Info: [c] cpu/throughput + i2c drop counters   [d] paste-back defaults line");
  Serial.println(
    "Page A (main): knob0=decay knob1=damp knob2=LPF knob3=HPF knob4=crunch "
    "knob5=predelay knob6=diffusion knob7=mix");
  Serial.println(
    "Page B (mod, toggle selects): pot0(slide)=unused here; pot1=in-gain "
    "pot2=out-gain pot3=rate1 pot4=depth1 pot5=rate2 pot6=depth2 pot7=MIDI-ch, "
    "mod state + MIDI ch persisted to flash ~1s after last move");
  Serial.println(
    "Bank: s<0-3> save slot   l<0-3> load slot   n<0-3><name> rename slot   "
    "B list all slots");
  Serial.println("Bypass: [b] toggle dry passthrough (independent of page)");
  Serial.println(
    "MIDI: in+out, all params, knob<->CC interchangeable, last event "
    "owns. CC102-109=PageA k0-7, CC110-115=PageB k0-5, CC116=bypass, CC65=page. "
    "RX/TX channel = Page B knob6 (NVS).");
}

// Returns true if a mod param changed (so caller can trigger printParams()).
// Also handles the preset-bank commands (s/l/n/B) and bypass ('b') here,
// since this is the sketch's only Serial.read() consumer -- a second
// independent reader would race it for bytes.
// There is no 'p' page-toggle key -- the slider directly drives page
// select (see uiTask), so a serial toggle would just get overridden on the
// next loop pass. 'b' is bypass now (was bank-list); bank-list moved to 'B'.

// Blocking-but-bounded helper: after 's'/'l'/'n'/'b', gather the digit (and
// for 'n', the name text) that follows. Same pattern as the reference
// sketch's 'g<value>' handler: keep polling while bytes are arriving, only
// give up after a brief GAP with nothing available (not a flat deadline) --
// so a slow paste still gets fully captured, without blocking uiTask when
// nothing more is coming.
static int readFollowingDigit(){
  uint32_t tLastByte=millis();
  while(millis()-tLastByte<=8){
    if(Serial.available()){
      int d=Serial.peek();
      if(d>='0' && d<='9'){ Serial.read(); return d-'0'; }
      break;   // next byte isn't a digit -- leave it queued, don't consume
    }
  }
  return -1;   // no digit arrived within the gap window
}
static void readFollowingName(char* out,int outLen){
  int ni=0; uint32_t tLastByte=millis();
  bool skippedLeadingSpace=false;
  while(ni<outLen-1 && millis()-tLastByte<=50){
    if(Serial.available()){
      int d=Serial.peek();
      if(d=='\n' || d=='\r') break;   // line end -- stop, leave it queued
      if(!skippedLeadingSpace && d==' '){ Serial.read(); skippedLeadingSpace=true; tLastByte=millis(); continue; }
      skippedLeadingSpace=true;
      out[ni++]=(char)Serial.read();
      tLastByte=millis();             // reset gap timer on each byte received
    }
  }
  out[ni]='\0';
}

static bool handleModKeys(){
  if(!Serial.available()) return false;
  int ch=Serial.read();
  if(ch=='s'){   // sN -- save current Page-B state into slot N
    int n=readFollowingDigit();
    if(n>=0) saveBankSlot(n);
    else     Serial.println("s<0-3>: save current mod state to a bank slot");
    return false;
  }
  if(ch=='l'){   // lN -- load slot N into the live mod params
    int n=readFollowingDigit();
    if(n>=0) loadBankSlot(n);
    else     Serial.println("l<0-3>: load a bank slot into current mod state");
    return true;   // triggers printParams() via caller; loadBankSlot already printed details
  }
  if(ch=='n'){   // nN<name> -- rename slot N (name applies on next save)
    int n=readFollowingDigit();
    if(n>=0){
      char nm[16]; readFollowingName(nm,sizeof(nm));
      if(nm[0]){
        strncpy(s_slotName[n],nm,sizeof(s_slotName[n])-1);
        s_slotName[n][sizeof(s_slotName[n])-1]='\0';
        Serial.printf("Slot %d renamed to '%s' (save with s%d to persist)\n",n,s_slotName[n],n);
      } else {
        Serial.printf("n<0-3><name>: rename a slot, e.g. n2 SHIMMER\n");
      }
    }
    return false;
  }
  if(ch=='B'){ printBank(); return false; }   // dump all 4 slots
  // [c] one-shot CPU/throughput dump. Deliberately on-demand: a once-per-second
  // print ran forever and buried every other message.
  if(ch=='c'){
    uint32_t us=g_blockUs;
    Serial.printf("cpu=%lu%%  (%lu us / %d us budget)   blocks=%lu\n",
                  (unsigned long)((us*100UL)/BLOCK_BUDGET_US),
                  (unsigned long)us, BLOCK_BUDGET_US,
                  (unsigned long)g_blockCount);
    // I2C health. Both counts should be 0 on a healthy bus -- anything
    // else means reads are being dropped and pot/switch values you see are stale.
    // Rate is integer ppm, not %f: printf float formatting isn't guaranteed on
    // every ESP32 core build, and this path must never be the thing that breaks.
    uint32_t rd=g_i2cPotReads, dr=g_i2cPotDrops, sw=g_i2cSwDrops;
    uint32_t ppm = rd ? (uint32_t)(((uint64_t)dr*1000000ULL)/rd) : 0;
    Serial.printf("i2c: pot drops=%lu / %lu reads (%lu ppm)   switch drops=%lu%s\n",
                  (unsigned long)dr, (unsigned long)rd, (unsigned long)ppm,
                  (unsigned long)sw,
                  (dr||sw) ? "   <-- BUS FAULT" : "   [clean]");
    return false;
  }
  // [d] paste-back defaults line, on demand -- a source-editing aid, not
  // something you want printed on every knob settle.
  if(ch=='d'){ printDefaults(); return false; }
  if(ch=='b'){   // bypass toggle, decoupled from the page slider. No debounce --
                 // a keypress isn't a noisy analog read. NOTE: with
                 // HAS_BYPASS_SWITCH the physical switch is authoritative, so
                 // this flip is re-asserted away on the next uiTask pass.
    pBypass=!pBypass;
#if HAS_BYPASS_SWITCH
    Serial.printf("Bypass: %s (note: physical switch will override)\n", pBypass?"ON":"OFF");
#else
    Serial.printf("Bypass: %s\n", pBypass?"ON":"OFF");
#endif
    return true;   // triggers printParams() via caller
  }
  float r1=pModRate1, r2=pModRate2, d1=pModDepth1, d2=pModDepth2;
  bool changed=true;
  switch(ch){
    case '1': r1-=MOD_RATE_STEP;  break;
    case '!': r1+=MOD_RATE_STEP;  break;
    case '2': r2-=MOD_RATE_STEP;  break;
    case '@': r2+=MOD_RATE_STEP;  break;
    case 'q': d1-=MOD_DEPTH_STEP; break;
    case 'Q': d1+=MOD_DEPTH_STEP; break;
    case 'w': d2-=MOD_DEPTH_STEP; break;
    case 'W': d2+=MOD_DEPTH_STEP; break;
    case 'm': printModHelp(); changed=false; break;
    case 'h': case '?': printModHelp(); changed=false; break;
    default: changed=false; break;   // ignore newlines / unknown keys quietly
  }
  if(changed){
    if(r1<0.f)r1=0.f; if(r1>5.f)r1=5.f;          // sane rate ceiling
    if(r2<0.f)r2=0.f; if(r2>5.f)r2=5.f;
    if(d1<0.f)d1=0.f; if(d1>EXC_MAX) d1=EXC_MAX;  // hard-clamped again in
    if(d2<0.f)d2=0.f; if(d2>EXC2_MAX)d2=EXC2_MAX; // setModDepths(), belt+braces
    pModRate1=r1; pModRate2=r2; pModDepth1=d1; pModDepth2=d2;
  }
  return changed;
}

void uiTask(void*){
  uint32_t lastDisp=0, lastRate=0, lastCount=0, blkPerSec=0, cpuPct=0;
  uint32_t lastMove=0; bool paramsDirty=false;
  for(;;){
    uint32_t now=millis();

    if(now-lastRate>=1000){
      uint32_t c=g_blockCount;
      uint32_t elapsed=now-lastRate;              // ACTUAL window, not assumed 1000
      blkPerSec=((c-lastCount)*1000UL)/elapsed;   // <- was biased high by OLED flush
      lastCount=c; lastRate=now;
      uint32_t us=g_blockUs;
      cpuPct=(us*100UL)/BLOCK_BUDGET_US;          // TRUE headroom gauge
      // Deliberately NOT printed here: a per-second console line buried every
      // other message. Both values feed the OLED header; [c] dumps them.
    }

    // ── Serial keys → tank modulation rate/depth, bank commands, bypass ────
    if(handleModKeys()){ paramsDirty=true; lastMove=now; }

    // ── MIDI RX: feed every queued byte into the running-status
    //    parser. Bounded per pass (guard) so a CC flood can't starve the rest
    //    of uiTask. CC dispatch marks params MIDI-owned; a later physical knob
    //    move clears that (last-writer-wins). ──
    {
      int guard=0;
      while(Serial1.available() && guard++<64) midiFeedByte((uint8_t)Serial1.read());
    }

    // ── Slider → page select: position IS the page, read live every pass.
    //    Fully decoupled from bypass. ──
    bool sliderB=readToggle();
    if(sliderB!=g_pageB){
      bool wasB=g_pageB;
      g_pageB=sliderB;
      if(g_pageB && !wasB) rearmPageBCatch();     // entering B: pots start "waiting"
      if(!g_pageB && wasB) rearmPageACatch();     // entering A: pickup, don't snap
      if(!g_pageB && wasB && s_pbDirty) savePageBSettings(); // leaving B: persist now
      Serial.printf("Page: %s (slider)\n", g_pageB?"B (mod)":"A (main)");
      paramsDirty=true; lastMove=now;
    }
    // ── Physical bypass switch: register 0x22, authoritative. Its
    //    level IS the bypass state -- serial 'b' / MIDI CC116 still write pBypass,
    //    but the switch re-asserts its position every pass, so a held switch wins
    //    (correct for a hardware bypass). Compiled in only when HAS_BYPASS_SWITCH
    //    is set; on a stock 8Angle build this block doesn't exist and bypass is
    //    serial/MIDI only. readSwitch2() holds the current state on a failed read,
    //    so a bus glitch can't flip bypass. ──
#if HAS_BYPASS_SWITCH
    {
      bool nb=readSwitch2();
      if(nb!=pBypass){
        pBypass=nb;
        Serial.printf("Bypass: %s (switch)\n", nb?"ON":"OFF");
        paramsDirty=true; lastMove=now;
      }
    }
#endif
    bool bypass=pBypass;   // physical switch (if built in) else serial/MIDI

    readAllPots();                 // all 8 knobs now mapped
    if(g_pageB){
      applyPotPageB();             // bulk-applies params 0-6 (pots 1-7) with catch gating
      // Move-detect / reclaim / TX per PARAMETER p (0..6). Physical pot = p+PAGEB_POT0.
      // The slide pot (pot 0) is Page-A decay and is intentionally ignored here.
      for(int p=0;p<PAGEB_POTS;p++){
        int c=p+PAGEB_POT0;        // physical pot feeding this param
        bool moved = abs((int)pot_raw[c]-lastRaw[c])>POT_DEADBAND;
        if(moved){
          paramsDirty=true; lastMove=now; lastRaw[c]=pot_raw[c];
          // last-writer-wins + TX for the six CC-mapped params (0..5). Param 6
          // (MIDI channel) has no CC/ownership, so it's skipped. Only fires once
          // the param is CAUGHT so a pre-catch physical position doesn't emit.
          if(p<6 && s_pbCaught[p]){
            s_midiOwned[PID_PAGEB_BASE+p]=false;
            midiTxPageParam(PID_PAGEB_BASE+p, pot_raw[c]);
          }
        }
        // LED on the physical pot: white flash on move, green when caught/live,
        // dim green while waiting for catch.
        if(moved)              setLed(c,255,255,255,60);
        else if(s_pbCaught[p]) setLed(c,0,180,0,40);
        else                   setLed(c,0,60,0,20);   // waiting for catch
      }
      // Slide pot (pot 0) is Page-A decay: unused on Page B, shown dim. Keep its
      // lastRaw fresh so returning to Page A doesn't register a phantom move.
      setLed(0,10,10,10,10);
      lastRaw[0]=pot_raw[0];
    } else {
      for(int c=0;c<8;c++){
        bool moved = abs((int)pot_raw[c]-lastRaw[c])>POT_DEADBAND;
        if(moved){
          paramsDirty=true; lastMove=now;   // settle timer restarts on each move
          lastRaw[c]=pot_raw[c];
          // Only reclaim from MIDI + TX when the move actually DRIVES the param
          // (past the pickup gate on page re-entry). Between POT_DEADBAND and
          // PAGEA_PICKUP_THRESH a nudge is detected but not yet driving, so it
          // should neither steal from MIDI nor emit a CC. potDrivesPageA()
          // latches "picked" as a side effect once crossed, so calling it here
          // and then applyPot (which calls it again, now true) is consistent.
          if(potDrivesPageA(c)){
            s_midiOwned[PID_PAGEA_BASE+c]=false;   // last-writer-wins: hand reclaims
            applyPotPageA(c,pot_raw[c]);
            midiTxPageParam(PID_PAGEA_BASE+c, pot_raw[c]); // emit CC for the move
          }
        }
        // LED: red while bypassed; else white flash on move, idle blue otherwise;
        // dim (waiting for pickup) if armed and not yet grabbed since re-entry.
        if(bypass)                          setLed(c,200,0,0,60);
        else if(moved)                       setLed(c,255,255,255,60);
        else if(s_pageAArmed && !s_paPicked[c]) setLed(c,40,40,0,25);  // waiting
        else                                  setLed(c,0,40,80,30);
      }
    }

    // print the full set once the knobs have settled for 400 ms
    if(paramsDirty && (now-lastMove)>400){ paramsDirty=false; printParams(); }

    // ── Debounced persist of Page-B settings ─────────────────────────────
    // Save ~1s after the last change, so rapid knob sweeps coalesce into one
    // flash write (NVS has limited write-endurance). Leaving Page B also
    // flushes immediately (above).
    if(s_pbDirty && s_pbDirtySince!=0 && (now-s_pbDirtySince)>1000){
      savePageBSettings();
      s_pbDirtySince=0;
    }

    if(now-lastDisp>=DISPLAY_MS){ lastDisp=now; drawDisplay(blkPerSec,cpuPct); }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ============================================================================
//  setup — non-I2C work + amy_start() FIRST, then Wire + UI task.
// ============================================================================
void uiTask(void* arg);   // fwd decl

void setup(){
#if !ARDUINO_USB_CDC_ON_BOOT
  USB.begin();
#endif
  Serial.begin(115200);
  delay(3000);                      // let CDC enumerate before first print
  Serial.println("Ready [BUILD: rev-24]");
  // Boot output is deliberately sanity-only (build, pins, bypass, OLED ACK,
  // "Setup done"). Press [h] or [?] for the key/CC reference.

  // ---- MIDI: TRS MIDI IN on AMY's dedicated pin + differential OUT.
  //      UART only (no I2C), so safe before amy_start(), same as the Eltro
  //      reference. RX feeds the CC parser in uiTask; TX emits knob moves.
  //        TX (normal)   on DATT_MIDI_TX_PIN     -- data leg
  //        TX (inverted) on DATT_MIDI_RETURN_PIN -- active return (push-pull)
  //      The inverted mirror is routed via the ESP32-S3 GPIO matrix so both
  //      pads emit the same UART1 TX signal, one inverted, so a DIN-4/5 opto
  //      input sees a clean full swing (bench-confirmed on the Eltro build).
  Serial1.begin(31250, SERIAL_8N1, AMYBOARD_MIDI_IN, DATT_MIDI_TX_PIN);
  Serial1.setPins(AMYBOARD_MIDI_IN, DATT_MIDI_TX_PIN);   // force TX pad mux
  {
    uint32_t tx_sig = uart_periph_signal[1].pins[SOC_UART_TX_PIN_IDX].signal;
    gpio_set_direction((gpio_num_t)DATT_MIDI_RETURN_PIN, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal((gpio_num_t)DATT_MIDI_RETURN_PIN,
                                    tx_sig, true /*invert*/, false);
  }
  Serial.printf("MIDI pins: RX=%d TX=%d RET=%d(inv)\n",
                AMYBOARD_MIDI_IN, DATT_MIDI_TX_PIN, DATT_MIDI_RETURN_PIN);

  loadPageBSettings();               // mod rate/depth + MIDI channel: NVS if saved, else defaults
  loadBankNames();                   // preset-bank slot names, so the OLED/console show
                                      // real names before any slot is touched this session
  gPlate.init((float)AMY_SAMPLE_RATE);

  // ---- AMY config. amy_start() BEFORE any Wire/I2C (cold-boot latch) ----
  amy_config_t cfg = amy_default_config();
  cfg.features.default_synths = 0;
  cfg.features.reverb         = 0;   // we ARE the reverb
  cfg.features.echo           = 0;
  cfg.features.chorus         = 0;
  cfg.features.startup_bleep  = 0;
  cfg.features.partials       = 0;
  cfg.max_oscs                = 200; // must exceed audio-in osc index (~170) or
                                     // the input osc is never allocated and no
                                     // audio reaches the effect. (From reference.)
  cfg.midi                    = AMY_MIDI_IS_NONE;
  cfg.features.audio_in       = 1;
  amy_start(cfg);

  // ---- I2C AFTER amy_start() ----
  Wire.begin(I2C_SDA_PIN,I2C_SCL_PIN);
  Wire.setClock(100000);

  readAllPots();
  memcpy(pot_prev,pot_raw,sizeof(pot_raw));
  g_pageB=readToggle();          // page follows slider position from boot, not
                                  // just from the first uiTask pass

  // On the emulator build, adopt the physical bypass switch position at boot so
  // the unit comes up matching the toggle. On a stock build this is compiled out
  // and bypass boots false, driven thereafter by serial/MIDI.
#if HAS_BYPASS_SWITCH
  pBypass=readSwitch2();
  Serial.printf("[bypass] switch build: boot %s\n", pBypass?"ON":"OFF");
#endif

  for(int i=0;i<8;i++){ lastRaw[i]=pot_raw[i]; applyPot(i,pot_raw[i]); }
  rearmPageBCatch();   // Page-B pots start "waiting" so loaded NVS values
                        // aren't overwritten by wherever knobs 0-4 physically
                        // sit; matches Page A's boot-time live behavior only
                        // once you actually visit Page B and move a knob.

  bool ok=oledInit();
  Serial.printf("[oled] init: %s\n", ok?"ACKed":"FAILED");
  oledClear(); oledStr(30,28,"DATTORRO"); oledFlush();

  // ---- stereo external-input oscillators (osc0=L, osc1=R) ----
  amy_event e = amy_default_event(); e.reset_osc=RESET_AMY; amy_add_event(&e);

  e = amy_default_event();
  e.synth=18; e.num_voices=1; e.oscs_per_voice=2; amy_add_event(&e);

  e = amy_default_event();
  e.synth=18; e.osc=0; e.wave=AUDIO_EXT0;
  e.pan_coefs[COEF_CONST]=0.0f; e.amp_coefs[COEF_CONST]=1.0f; e.velocity=1.0f;
  amy_add_event(&e);

  e = amy_default_event();
  e.synth=18; e.osc=1; e.wave=AUDIO_EXT1;
  e.pan_coefs[COEF_CONST]=1.0f; e.amp_coefs[COEF_CONST]=1.0f; e.velocity=1.0f;
  amy_add_event(&e);

  e = amy_default_event();
  e.synth=18; e.osc=0; e.wave=AUDIO_EXT0; e.midi_note=60; e.velocity=1.0f;
  amy_add_event(&e);
  e = amy_default_event();
  e.synth=18; e.osc=1; e.wave=AUDIO_EXT1; e.midi_note=60; e.velocity=1.0f;
  amy_add_event(&e);

  // ---- UI task owns I2C (core 0); loop() is audio-only (core 1) ----
  xTaskCreatePinnedToCore(uiTask,"ui",8192,NULL,1,NULL,0);
  Serial.println("Setup done.");
  printParams();                    // known state on the console at boot
}
