/*
 * OV7670 (module without FIFO, sensor reports PID/VER 76/73) on ESP32-C3 SuperMini -
 * still image, 4 bit (D7..D4), grayscale (Y from YUV422)
 *
 * Wiring (OV7670 module -> SuperMini):
 *   3V3   -> 3V3            GND  -> GND
 *   RESET -> 3V3            PWDN -> GND
 *   SIOC  -> GPIO9  (SCL)   SIOD -> GPIO8  (SDA)   4k7 pull-up to 3V3 each, if not on the module
 *   XCLK  -> GPIO0  (LEDC, 10 MHz)
 *   PCLK  -> GPIO1
 *   VSYNC -> GPIO3
 *   HREF  -> GPIO10
 *   D4    -> GPIO4   D5 -> GPIO5   D6 -> GPIO6   D7 -> GPIO7
 *   Leave D0..D3 unconnected.
 *
 * Why this pinout: D4..D7 on GPIO4..7 so the nibble comes out of GPIO_IN with a single shift.
 * GPIO2 (strapping) and GPIO20/21 (UART0) stay free. GPIO8/9 are strapping pins too, but an
 * I2C pull-up (high) is harmless there. Do not press the boot button (GPIO9) while I2C is active.
 *
 * The C3 has no camera/parallel interface. Pixels are read by polling GPIO_IN_REG
 * (Dedicated GPIO). Measured limit: PCLK 7.5 MHz works, 10 MHz does not (bytes/line missing).
 *
 * Image modes (command M), frame period measured with T:
 *   0 VGA   640x479  107 ms  PLL x6, CLKRC 0x03
 *   1 QVGA  320x239   80 ms  PLL x4, CLKRC 0x01, DCW 2x, PCLK/2 = 5 MHz (at 7.5 MHz / 53 ms
 *                              ~1 % of lines lost a byte, at 5 MHz never; measured)
 *   2 QQVGA 160x119   27 ms  PLL x6, CLKRC 0x00, DCW 4x, PCLK/4
 * VGA and QQVGA with PCLK = 7.5 MHz.
 * Sensor windowing (VSTRT/VSTOP) does NOT shorten the frame period (measured: 160 ms at 12 lines);
 * the only way to go faster is a higher internal clock, and PCLK only copes with that via the DCW divider.
 *
 * Arduino IDE: board "ESP32C3 Dev Module", "USB CDC On Boot: Enabled".
 *
 * Serial protocol (text lines, hex without 0x):
 *   C x y w h s p bh bv
 *                   -> "IMG ow oh sum16 bpp\n" + image data
 *                      x,y,w,h: ROI in pixels of the current mode, s: step (decimation),
 *                      p: byte phase 0/1 (which byte of UYVY is Y),
 *                      bh,bv: binning horizontal/vertical (1..8), digital averaging over bh x bv.
 *                      bpp 4: no binning, 2 pixels/byte, left pixel in the high nibble.
 *                      bpp 8: with binning, 1 pixel/byte, mean * 17 (0..255) - averaging
 *                      several 4-bit values yields real intermediate levels. With binning s = 1.
 *                      Note: the OV7670 cannot do true binning (charge summing in the sensor);
 *                      this improves noise and bit depth, not sensitivity.
 *   E [n]           -> set exposure in line periods (0 = auto), "EXP lines us auto"
 *   G [n]           -> set gain 0..1023 (-1 = auto), "GAIN raw auto"
 *                      E/G without argument only query (in auto: current control value).
 *                      Mode change (M) resets the sensor -> back to auto.
 *   W rr vv         -> write register, "OK"
 *   R rr            -> "REG rr vv"
 *   T               -> "TIM period_us vsync_us lines": frame period, VSYNC high time, HREF lines per frame
 *   D ms            -> delay before sending after a capture (default 0), "OK"
 *   S x y w h th [pol [prof]]
 *                   -> slit measurement, runs until the next received character. One line per frame
 *                      "EDGE pos dir ms": pos = position of the first edge along the slit in
 *                      pixels of the mode (sub-pixel interpolated, -1 = none), dir +1 dark->bright,
 *                      -1 bright->dark, ms = millis(). The narrow side of the ROI (max. 8 pixels)
 *                      is averaged (binning), the long side is the slit. th 0..255 (value * 17,
 *                      as in the image), pol 0 both, 1 only dark->bright, 2 only bright->dark.
 *                      prof 1: additionally "PROF v0 v1 ..." (hex) per frame, for tuning th.
 *                      Rate = 1 measurement per frame (VGA ~9/s, QVGA ~12/s, QQVGA ~37/s): each
 *                      sensor line comes only once per frame, windowing does not shorten the frame.
 *   U x y w h [k10 [dmin [mass [learn [prof [lo [comp]]]]]]]
 *                   -> change detection instead of a fixed threshold (beam interruption):
 *                      learns mean mu and std. deviation sigma per position over `learn` frames
 *                      (default 32) ("LEARNED ..."), then per frame "BRK first last count ms a":
 *                      first/last interrupted position (-1 = none), their count and the
 *                      estimated brightness a in % (comp 1, default: illumination compensation,
 *                      v is compared with a*mu; a = median of v/mu; comp 0: a = 100).
 *                      Threshold per position thr = k*sigma + dmin, k = k10/10 (default 5.0),
 *                      dmin default 17 (= 1 level of the 4-bit data). Cluster mass: every
 *                      position with |v - mu| > lo% * thr (default 50) is a candidate and contributes
 *                      min(1, |v - mu| / thr); a contiguous run counts as interrupted if its sum
 *                      >= mass (default 2). Wide weak changes thus count like a few strong ones.
 *                      lo 100 = old rule (run length).
 *                      prof 1: additionally lines PROF/MEAN/THR (hex).
 *                      During measurement: "L" relearns (keep the slit clear!), any other
 *                      character stops it. Interrupted positions are never updated.
 *   M n             -> set image mode 0/1/2, "MODE n w h"; "M" alone only queries
 *   I               -> "ID pid ver" (OV7670: 76 73)
 */
#include <Arduino.h>
#include <Wire.h>
#include <algorithm>
#include "driver/dedic_gpio.h"
#include "hal/dedic_gpio_cpu_ll.h"

constexpr int PIN_XCLK  = 0;
constexpr int PIN_PCLK  = 1;
constexpr int PIN_VSYNC = 3;
constexpr int PIN_D4    = 4;   // D4..D7 = GPIO4..7, contiguous
constexpr int PIN_SDA   = 8;
constexpr int PIN_SCL   = 9;
constexpr int PIN_HREF  = 10;

// Read via Dedicated GPIO (CSR 0x804, 1 CPU cycle) instead of GPIO_IN_REG (APB bus).
// With GPIO_IN_REG, HREF detection came too late: at PCLK 5 MHz ~20 % of lines lost the
// first byte, the line was shifted by one byte (U/V instead of Y). Channel order = DEDIC_PINS:
static const int DEDIC_PINS[] = {PIN_D4, PIN_D4 + 1, PIN_D4 + 2, PIN_D4 + 3, PIN_PCLK, PIN_HREF, PIN_VSYNC};
constexpr uint32_t M_PCLK  = 1u << 4;
constexpr uint32_t M_HREF  = 1u << 5;
constexpr uint32_t M_VSYNC = 1u << 6;

constexpr uint8_t  OV_ADDR  = 0x21;          // SCCB 0x42/0x43
constexpr uint32_t XCLK_HZ  = 10000000;      // OV7670: XCLK 10..48 MHz
constexpr int SENSOR_W = 640;      // buffer sizes = VGA
// Line 0 is lost (see capture()), hence one usable line less per mode.
constexpr int SENSOR_H = 479;

struct RegVal { uint8_t r, v; };
// Window values for QVGA/QQVGA from the Linux driver ov7670.c (hstart 164, hstop 20, vstart 14,
// vstop 494). With the reset defaults 10 resp. 8 bytes per line were missing (630/640, 312/320).
#define OV_WIN_DCW {0x17, 0x14}, {0x18, 0x02}, {0x32, 0xA4}, {0x19, 0x03}, {0x1A, 0x7B}, {0x03, 0x0A}
static const RegVal MODE_VGA[]   = {{0x6B, 0x8A}, {0x11, 0x03}};
static const RegVal MODE_QVGA[]  = {{0x6B, 0x4A}, {0x11, 0x01}, {0x0C, 0x04}, {0x3E, 0x19}, {0x72, 0x11}, {0x73, 0xF1}, OV_WIN_DCW};
static const RegVal MODE_QQVGA[] = {{0x6B, 0x8A}, {0x11, 0x00}, {0x0C, 0x04}, {0x3E, 0x1A}, {0x72, 0x22}, {0x73, 0xF2}, OV_WIN_DCW};
// period_us measured with T. One frame period spans 510 line periods (VGA timing, also with DCW).
struct Mode { const RegVal* regs; size_t n; int w, h; uint32_t period_us; };
static const Mode MODES[] = {
  {MODE_VGA,   sizeof(MODE_VGA) / sizeof(RegVal),   640, 479, 106600},
  {MODE_QVGA,  sizeof(MODE_QVGA) / sizeof(RegVal),  320, 239, 80000},
  {MODE_QQVGA, sizeof(MODE_QQVGA) / sizeof(RegVal), 160, 119, 26700},
};
static int gMode = 1;
// Slit: the narrow side of the ROI (max. 8 pixels) is averaged, the long side is the measuring direction.
// Defined up here because the Arduino IDE places function prototypes before the rest of the file.
struct Slit { int x, y, w, h, bh, bv, ow, oh, n, off; bool horiz; };

// Limit for busy-wait loops inside the critical section. Well below the
// interrupt watchdog (300 ms) and above one line time at slow PCLK.
constexpr uint32_t SPIN_TO = 1500000;

#define GIN() dedic_gpio_cpu_ll_read_in()

alignas(4) static uint8_t lineBuf[SENSOR_W * 2 + 4];
// Binning: vertical sums per column, two columns per 32-bit word (16-bit lanes, max 8*15)
static uint32_t colAcc[SENSOR_W / 2 + 1];
static uint8_t binLut[8 * 8 * 15 + 1];   // sum -> mean * 17
// Static: the heap has no contiguous block for full resolution (malloc 153 KB failed).
static uint8_t frameBuf[(SENSOR_W / 2) * SENSOR_H];
// Delay before sending. Measured over 90 frames each: 20 ms 3 retries, 0 ms 4 -> no benefit;
// the viewer detects lost packets via checksum and requests again.
static uint32_t sendDelayMs = 0;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ---------- SCCB ----------
static bool wr(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(OV_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static int rd(uint8_t reg) {
  // SCCB does not tolerate a repeated start -> STOP first, then read
  Wire.beginTransmission(OV_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(true) != 0) return -1;
  if (Wire.requestFrom((int)OV_ADDR, 1) != 1) return -1;
  return Wire.read();
}

static void startXclk() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_XCLK, XCLK_HZ, 1);
  ledcWrite(PIN_XCLK, 1);            // 1-bit resolution -> 50 %
#else
  ledcSetup(0, XCLK_HZ, 1);
  ledcAttachPin(PIN_XCLK, 0);
  ledcWrite(0, 1);
#endif
}

static bool sensorInit(int mode) {
  if (!wr(0x12, 0x80)) return false;  // COM7: reset
  delay(100);
  wr(0x12, 0x00);                     // COM7: YUV422
  wr(0x15, 0x00);                     // COM10: VSYNC/HREF positive, PCLK free-running
  const Mode& m = MODES[mode];
  for (size_t i = 0; i < m.n; ++i) wr(m.regs[i].r, m.regs[i].v);
  gMode = mode;
  delay(300);                         // let PLL and AEC/AGC settle
  return true;
}

// ---------- Capture ----------
static bool IRAM_ATTR waitLevelMs(uint32_t mask, bool level, uint32_t ms) {
  uint32_t t0 = millis();
  while (((GIN() & mask) != 0) != level)
    if (millis() - t0 > ms) return false;
  return true;
}

// Returns nullptr on success, otherwise an error message.
static int gBmin, gBmax, gMissed, gShort;
static uint32_t gProcMax;   // cycles for processing after a line (diagnostics)   // bytes per captured line, expected 2*(x+w)

// passes > 1 (binning only): block k is captured in pass k % passes, one new frame per
// pass. Each block end is then followed by an unneeded block, during which the
// conversion completes. Required in VGA: the blanking there is ~6000 cycles, the conversion at
// block end takes up to ~12000 (measured); with passes = 1 every line after a block end was missing.
// IRAM_ATTR: avoids flash cache misses in the time-critical loop. Open issue: in QVGA
// with vertical binning some runs had ~1 % of lines one byte short ("short"
// in INFO), others none; cause not found, IRAM alone did not reliably fix it.
static inline bool IRAM_ATTR wantedLine(int li, int pass, int y, int yEnd, bool bin, int bv,
                                        int s, int passes) {
  if (li < y || li >= yEnd) return false;
  return bin ? ((li - y) / bv) % passes == pass : ((li - y) % s) == 0;
}

static const char* IRAM_ATTR capture(int x, int y, int w, int h, int s, int bh, int bv, int phase,
                           uint8_t* out, int ow, int oh, int rowBytes, int passes) {
  const int nBytes = 2 * (x + w);
  const bool bin = bh * bv > 1;
  // Scaling sum -> mean * 17 via lookup table (saves the multiplication during blanking)
  if (bin) {
    const uint32_t n = bh * bv;
    for (uint32_t a = 0; a <= 15 * n; ++a) binLut[a] = (uint8_t)((a * 17 + n / 2) / n);
  }
  const int yEnd = bin ? y + oh * bv : y + h;
  gBmin = 99999; gBmax = 0; gMissed = 0; gShort = 0; gProcMax = 0;

  for (int pass = 0; pass < passes; ++pass) {
    // Synchronize to frame start (interrupts on, the VSYNC phases are long)
    if (!waitLevelMs(M_VSYNC, true, 15000))  return "no VSYNC high";
    if (!waitLevelMs(M_VSYNC, false, 15000)) return "no VSYNC low";
    // The vertical blanking interval passes before the first line. At slow PCLK this can take
    // longer than the interrupt watchdog allows, so we wait outside the critical section.
    // This first line (sensor line 0) is sacrificed for that.
    if (!waitLevelMs(M_HREF, true, 3000))   return "no HREF";

    int outRow = 0;   // only without binning; with binning the row follows from the block
    for (int li = 0; li < yEnd; ++li) {   // li = sensor line li+1
      const bool want = wantedLine(li, pass, y, yEnd, bin, bv, s, passes);
      uint32_t v, t, c0 = 0;
      int b = 0;

      portENTER_CRITICAL(&mux);
      t = SPIN_TO; while (GIN() & M_HREF)    if (!--t) goto timeout;  // end of previous line
      t = SPIN_TO; while (!(GIN() & M_HREF)) if (!--t) goto timeout;  // start of this line
      if (want) {
        // The byte loop is cycle-optimized to the limit: every change (mask, extra compare,
        // taking the first byte from the HREF poll) caused missing bytes at 7.5 MHz (measured).
        while (b < nBytes) {
          // Data is valid on the rising PCLK edge (sensor changes it on the falling edge)
          do { v = GIN(); if (!(v & M_HREF)) goto eol; } while (!(v & M_PCLK));
          // Bits 0..3 = D4..D7, 4..6 = PCLK/HREF/VSYNC, do not mask here (see above)
          lineBuf[b++] = (uint8_t)v;
          while (GIN() & M_PCLK) {}
        }
      eol:
        t = SPIN_TO; while (GIN() & M_HREF) if (!--t) goto timeout;   // rest of the line (ROI < width)
        c0 = esp_cpu_get_cycle_count();
        gBmin = min(gBmin, b); gBmax = max(gBmax, b);
        if (b < nBytes) ++gShort;
        if (bin) {
          // Per line only sum vertically, 2 columns per 32-bit word (bytes U Y V Y -> Y bytes
          // via shift by 8*phase into the 16-bit lanes). Horizontal summing and table scaling
          // happen only in the last line of the block.
          const int r = (li - y) % bv;
          const uint32_t* w32 = (const uint32_t*)(lineBuf + 2 * x);   // x is even (cmdCapture)
          const int nw = (ow * bh + 1) / 2;
          const int sh = 8 * phase;
          if (r == 0) for (int i = 0; i < nw; ++i) colAcc[i] = (w32[i] >> sh) & 0x000F000F;
          else        for (int i = 0; i < nw; ++i) colAcc[i] += (w32[i] >> sh) & 0x000F000F;
          if (r == bv - 1) {
            uint8_t* row = out + ((li - y) / bv) * rowBytes;
            const uint16_t* col = (const uint16_t*)colAcc;   // little-endian: lane 0 = even column
            switch (bh) {   // fixed loop lengths -> compiler unrolls
              case 1: for (int k = 0; k < ow; ++k, col += 1) row[k] = binLut[col[0]]; break;
              case 2: for (int k = 0; k < ow; ++k, col += 2) row[k] = binLut[col[0] + col[1]]; break;
              case 4: for (int k = 0; k < ow; ++k, col += 4)
                        row[k] = binLut[col[0] + col[1] + col[2] + col[3]]; break;
              default:
                for (int k = 0; k < ow; ++k, col += bh) {
                  uint32_t a = 0;
                  for (int j = 0; j < bh; ++j) a += col[j];
                  row[k] = binLut[a];
                }
            }
          }
        } else {
          // Branch-free decimation; the first version (index computation per pixel) was too slow
          // for horizontal blanking, then every second line was missing (image doubled, squashed).
          const uint8_t* src = lineBuf + 2 * x + phase;
          uint8_t* row = out + outRow * rowBytes;
          const int st = 2 * s;
          for (int k = 0; k < ow / 2; ++k, src += 2 * st)
            row[k] = (uint8_t)(src[0] << 4) | (src[st] & 0x0F);
          if (ow & 1) row[ow / 2] = (uint8_t)(src[0] << 4);
          ++outRow;
        }
        const uint32_t dc = esp_cpu_get_cycle_count() - c0;
        if (dc > gProcMax) gProcMax = dc;
      }
      // If HREF is already high again here, the start of the next line was missed: it is counted
      // so line numbers stay correct. It is an error ("missed") only if that line was needed.
      if (want && (GIN() & M_HREF)) { if (wantedLine(li + 1, pass, y, yEnd, bin, bv, s, passes)) ++gMissed; ++li; }
      portEXIT_CRITICAL(&mux);   // pending ISRs (tick, USB) run briefly here
      continue;
    timeout:
      portEXIT_CRITICAL(&mux);
      return "HREF timeout";
    }
  }
  return nullptr;
}

static void cmdCapture(int x, int y, int w, int h, int s, int p, int bh, int bv) {
  const int W = MODES[gMode].w, H = MODES[gMode].h;
  x = constrain(x, 0, W - 1);
  y = constrain(y, 0, H - 1);
  w = constrain(w, 1, W - x);
  h = constrain(h, 1, H - y);
  s = constrain(s, 1, 64);
  p &= 1;
  bh = constrain(bh, 1, 8);
  bv = constrain(bv, 1, 8);
  const bool bin = bh * bv > 1;
  if (bin) { s = 1; x &= ~1; }   // even x: 32-bit access to lineBuf
  // Binning: incomplete blocks at the edge are discarded
  const int ow = bin ? w / bh : (w + s - 1) / s;
  const int oh = bin ? h / bv : (h + s - 1) / s;
  const int rowBytes = bin ? ow : (ow + 1) / 2;
  if (ow < 1 || oh < 1) { Serial.println("ERR roi smaller than bin"); return; }
  const size_t n = (size_t)rowBytes * oh;

  uint8_t* buf = frameBuf;
  memset(buf, 0, n);
  const int passes = (bin && gMode == 0 && oh > 1) ? 2 : 1;   // see capture()
  const char* err = capture(x, y, w, h, s, bh, bv, p, buf, ow, oh, rowBytes, passes);
  // Optional delay (command D). After long interrupt-disabled periods a 64-byte packet is
  // occasionally lost; cause in HWCDC not resolved (SOF detection is commented out there), a delay
  // did not measurably help. Covered by checksum + retry in the viewer.
  delay(sendDelayMs);
  if (err) {
    Serial.printf("ERR %s\n", err);
  } else {
    // Leading \n and trailing END: the last USB packet of an image occasionally got stuck in the
    // TX buffer and only arrived before the next response (viewer log: binary remainder right before
    // "INFO"). The END pushes it out; the \n ensures the header line still starts
    // on its own line.
    Serial.printf("\nINFO bytes/line %d..%d expected %d missed %d short %d/%d proc %u cyc\n", gBmin, gBmax, 2 * (x + w), gMissed, gShort, oh, (unsigned)gProcMax);
    uint32_t sum = 0;
    for (size_t i = 0; i < n; ++i) sum += buf[i];
    Serial.printf("IMG %d %d %u %d\n", ow, oh, (unsigned)(sum & 0xFFFF), bin ? 8 : 4);
    for (size_t o = 0; o < n; o += 1024) Serial.write(buf + o, min((size_t)1024, n - o));
    Serial.write((const uint8_t*)"\nEND\n", 5);
    Serial.flush();
  }
}

// ---------- Exposure / gain ----------
// AEC[15:0] = AECHH(0x07)[5:0] : AECH(0x10)[7:0] : COM1(0x04)[1:0], unit line period.
// More than ~510 lines (one frame period) does not automatically lengthen the frame; that would
// require dummy lines (ADVFL/ADVFH 0x2D/0x2E).
static uint32_t getExposure() {
  return ((uint32_t)(rd(0x07) & 0x3F) << 10) | ((uint32_t)(rd(0x10) & 0xFF) << 2) | (rd(0x04) & 0x03);
}
static void setExposure(uint32_t n) {
  wr(0x13, rd(0x13) & ~0x01);                        // COM8: AEC off
  wr(0x04, (rd(0x04) & ~0x03) | (n & 0x03));
  wr(0x10, (n >> 2) & 0xFF);
  wr(0x07, (rd(0x07) & ~0x3F) | ((n >> 10) & 0x3F));
}
// GAIN[9:0] = VREF(0x03)[7:6] : GAIN(0x00)[7:0]. VREF[3:0] are window bits -> read-modify-write.
static uint32_t getGain() { return ((uint32_t)(rd(0x03) & 0xC0) << 2) | (rd(0x00) & 0xFF); }
static void setGain(uint32_t g) {
  wr(0x13, rd(0x13) & ~0x04);                        // COM8: AGC off
  wr(0x00, g & 0xFF);
  wr(0x03, (rd(0x03) & ~0xC0) | ((g >> 2) & 0xC0));
}

// ---------- Slit measurement ----------

static Slit slitSetup(int x, int y, int w, int h, const char* tag) {
  Slit k;
  const int W = MODES[gMode].w, H = MODES[gMode].h;
  k.x = constrain(x, 0, W - 1) & ~1;   // even, as required for binning
  k.y = constrain(y, 0, H - 1);
  k.w = constrain(w, 1, W - k.x);
  k.h = constrain(h, 1, H - k.y);
  k.horiz = k.w >= k.h;                // slit along the row: h averaged, otherwise w
  k.bh = k.horiz ? 1 : min(k.w, 8);
  k.bv = k.horiz ? min(k.h, 8) : 1;
  if (k.bh * k.bv == 1) { if (k.horiz) k.bv = 2; else k.bh = 2; }   // capture() averages only if bh*bv > 1
  if (k.horiz) k.h = k.bv; else k.w = k.bh;
  k.ow = k.w / k.bh;
  k.oh = k.h / k.bv;
  k.n = k.horiz ? k.ow : k.oh;
  k.off = k.horiz ? k.x : k.y;
  Serial.printf("%s %s %d..%d bin %dx%d\n", tag, k.horiz ? "row" : "col", k.off, k.off + k.n - 1, k.bh, k.bv);
  return k;
}

// Profile along the slit into frameBuf (ow x 1 or 1 x oh, contiguous in both cases)
static const char* slitCapture(const Slit& k) {
  return capture(k.x, k.y, k.w, k.h, 1, k.bh, k.bv, 1, frameBuf, k.ow, k.oh, k.ow, 1);
}

static void cmdScan(int x, int y, int w, int h, int th, int pol, int prof) {
  const Slit k = slitSetup(x, y, w, h, "SCAN");
  const int n = k.n, off = k.off;
  while (!Serial.available()) {
    const char* err = slitCapture(k);
    if (err) { Serial.printf("ERR %s\n", err); delay(100); continue; }
    const uint8_t* p = frameBuf;
    float pos = -1;
    int dir = 0;
    for (int i = 1; i < n; ++i) {
      const bool a = p[i - 1] >= th, b = p[i] >= th;
      if (a == b) continue;
      const int d = b ? 1 : -1;
      if ((pol == 1 && d < 0) || (pol == 2 && d > 0)) continue;
      pos = off + i - 1 + (float)(th - p[i - 1]) / (float)(p[i] - p[i - 1]);
      dir = d;
      break;
    }
    // Output in one piece: with a vertical slit only the short vertical blanking (QQVGA ~0.5 ms)
    // remains after the last line until the next VSYNC. Separate printf calls per value
    // took longer, then every second frame was missed (half rate, measured).
    static char out[24 + 5 + 3 * SENSOR_H + 2];
    int len = snprintf(out, sizeof(out), "EDGE %.1f %d %u\n", pos, dir, (unsigned)millis());
    if (prof) {
      static const char hx[] = "0123456789abcdef";
      memcpy(out + len, "PROF", 4); len += 4;
      for (int i = 0; i < n; ++i) { out[len++] = ' '; out[len++] = hx[p[i] >> 4]; out[len++] = hx[p[i] & 15]; }
      out[len++] = '\n';
    }
    Serial.write((const uint8_t*)out, len);
  }
  while (Serial.available()) Serial.read();   // discard the stop character
  Serial.println("OK");
}

// ---------- Change detection (beam interruption without a fixed threshold) ----------
// Background subtraction with a per-position noise model: first learn mean and std. deviation
// over `learn` frames, from that the per-position threshold thr = k*sigma + dmin (dmin because sigma
// can become 0 with 4-bit data). The decision uses the cluster mass of contiguous positions
// instead of individual pixels: an object in the slit always changes several neighbors, noise is
// spatially independent. This also detects low-contrast but wide objects without lowering the
// per-pixel threshold (more false alarms) - and single outliers still do not count. Non-interrupted
// positions are slowly updated (EMA 1/64) so brightness drift does not count as an interruption;
// interrupted ones are not, so a stationary object does not immediately merge into the background.
// Fixed point instead of float: the ESP32-C3 has no FPU.
static int32_t bgMu[SENSOR_W];    // mean, Q8
static int32_t bgVar[SENSOR_W];   // variance, Q8
static uint16_t bgThr[SENSOR_W];  // threshold |v - mu| in value units (0..255)
static uint8_t bgHit[SENSOR_W];

static uint32_t isqrt32(uint32_t v) {
  uint32_t r = 0, b = 1u << 30;
  while (b > v) b >>= 2;
  while (b) { if (v >= r + b) { v -= r + b; r = (r >> 1) + b; } else r >>= 1; b >>= 2; }
  return r;
}

static void bgThresholds(int n, int k10, int dmin) {
  for (int i = 0; i < n; ++i)
    bgThr[i] = (uint16_t)min<uint32_t>(255, (uint32_t)k10 * isqrt32((uint32_t)bgVar[i] >> 8) / 10 + dmin);
}

static void cmdChange(int x, int y, int w, int h, int k10, int dmin, int mass, int learn, int prof, int lo,
                      int comp) {
  const Slit k = slitSetup(x, y, w, h, "CHG");
  const int n = k.n;
  learn = constrain(learn, 4, 1000);
  const int32_t massQ8 = max(mass, 1) * 256;
  const int32_t loQ8 = constrain(lo, 1, 100) * 256 / 100;
  static uint16_t ex[SENSOR_W];   // contribution min(1, |v - mu| / thr), Q8; 0 = no candidate
  // Learning phase: sum and sum of squares per position (max. 1000 * 255^2 < 2^32)
  static uint32_t sum[SENSOR_W], sq[SENSOR_W];
relearn:
  memset(sum, 0, sizeof(uint32_t) * n);
  memset(sq, 0, sizeof(uint32_t) * n);
  for (int f = 0; f < learn; ) {
    if (Serial.available()) goto stop;
    if (slitCapture(k)) { delay(10); continue; }
    for (int i = 0; i < n; ++i) { sum[i] += frameBuf[i]; sq[i] += (uint32_t)frameBuf[i] * frameBuf[i]; }
    ++f;
  }
  for (int i = 0; i < n; ++i) {
    bgMu[i] = (int32_t)(((uint64_t)sum[i] << 8) / learn);
    const int64_t var = ((int64_t)sq[i] << 8) / learn - (((int64_t)bgMu[i] * bgMu[i]) >> 8);
    bgVar[i] = (int32_t)max<int64_t>(0, var);
  }
  bgThresholds(n, k10, dmin);
  {
    uint32_t sMin = 9999, sMax = 0, sSum = 0;
    for (int i = 0; i < n; ++i) {
      const uint32_t sd = isqrt32((uint32_t)bgVar[i] >> 8);
      sMin = min(sMin, sd); sMax = max(sMax, sd); sSum += sd;
    }
    Serial.printf("LEARNED %d frames sigma min %u mean %u max %u\n", learn, (unsigned)sMin,
                  (unsigned)(sSum / n), (unsigned)sMax);
  }

  for (;;) {
    // "L" relearns the background, any other character stops the measurement. Interrupted
    // positions are deliberately never updated (a stationary tube stays detected); a
    // permanent background change (light, camera moved) is only cleared by relearning.
    if (Serial.available()) {
      const int c = Serial.read();
      if (c != 'L' && c != 'l') goto stop;
      delay(2);
      while (Serial.available()) Serial.read();
      goto relearn;
    }
    if (const char* err = slitCapture(k)) { Serial.printf("ERR %s\n", err); delay(100); continue; }
    const uint8_t* p = frameBuf;
    // Illumination compensation: a change in illumination scales all positions roughly equally
    // (v ~ a * mu), an object only some of them. a = median of v/mu over the usable positions
    // (mu not dark, v and mu not saturated) - robust as long as the object covers less than
    // half of them. v is then compared with a * mu, and the sigma part of the threshold is
    // scaled by a as well (noise grows with signal).
    int32_t a = 256;                                   // Q8, 256 = 1.0
    if (comp) {
      static int32_t rat[SENSOR_W];
      int m = 0;
      for (int i = 0; i < n; ++i)
        if (bgMu[i] >= (34 << 8) && bgMu[i] < (238 << 8) && p[i] < 255)
          rat[m++] = ((int32_t)p[i] << 16) / bgMu[i];
      if (m >= max(8, n / 8)) {
        std::nth_element(rat, rat + m / 2, rat + m);
        a = constrain(rat[m / 2], 64, 1024);           // 0.25 .. 4
      }
    }
    static int32_t ref[SENSOR_W];                      // expected value a*mu, Q8
    static uint16_t thrA[SENSOR_W];                    // threshold at brightness a
    for (int i = 0; i < n; ++i) {
      ref[i] = (int32_t)(((int64_t)bgMu[i] * a) >> 8);
      thrA[i] = (uint16_t)min<int32_t>(255, ((max<int32_t>(0, bgThr[i] - dmin) * a) >> 8) + dmin);
      const int32_t r = abs(((int32_t)p[i] << 8) - ref[i]) / max<int32_t>(1, thrA[i]);  // Q8
      ex[i] = r > loQ8 ? (uint16_t)min<int32_t>(256, r) : 0;
    }
    // Runs of candidates: interrupted if the sum of contributions >= mass
    int first = -1, last = -1, count = 0;
    for (int i = 0; i < n; ) {
      if (!ex[i]) { bgHit[i++] = 0; continue; }
      int j = i;
      int32_t m = 0;
      while (j < n && ex[j]) m += ex[j++];
      const uint8_t hit = m >= massQ8;
      if (hit) {
        if (first < 0) first = i;
        last = j - 1;
        count += j - i;
      }
      for (int q = i; q < j; ++q) bgHit[q] = hit;
      i = j;
    }
    // Update only where this position is not interrupted. mu follows the raw value, so a
    // lasting illumination change migrates into mu and a returns toward 1; the variance uses the
    // deviation from the compensated value, otherwise the brightness step would inflate sigma.
    for (int i = 0; i < n; ++i) {
      if (bgHit[i]) continue;
      bgMu[i] += (((int32_t)p[i] << 8) - bgMu[i]) >> 6;
      const int32_t d0 = (((int32_t)p[i] << 8) - ref[i]) >> 4;  // Q4, so d0*d0 does not overflow
      bgVar[i] += ((d0 * d0) - bgVar[i]) >> 6;         // d0^2 is Q8
    }
    bgThresholds(n, k10, dmin);

    static char out[24 + 3 * (5 + 3 * SENSOR_W + 2)];
    int len = first < 0
      ? snprintf(out, sizeof(out), "BRK -1 -1 0 %u %d\n", (unsigned)millis(), (int)(a * 100 / 256))
      : snprintf(out, sizeof(out), "BRK %d %d %d %u %d\n", k.off + first, k.off + last, count,
                 (unsigned)millis(), (int)(a * 100 / 256));
    if (prof) {
      // PROF current profile, MEAN expected value a*mu, THR threshold |v - a*mu| (hex 0..ff)
      static const char hx[] = "0123456789abcdef";
      const char* tags[] = {"PROF", "MEAN", "THR "};
      for (int t = 0; t < 3; ++t) {
        memcpy(out + len, tags[t], 4); len += 4;
        for (int i = 0; i < n; ++i) {
          const int v = t == 0 ? p[i] : t == 1 ? min<int32_t>(255, (ref[i] + 128) >> 8) : thrA[i];
          out[len++] = ' '; out[len++] = hx[(v >> 4) & 15]; out[len++] = hx[v & 15];
        }
        out[len++] = '\n';
      }
    }
    Serial.write((const uint8_t*)out, len);
  }
stop:
  while (Serial.available()) Serial.read();
  Serial.println("OK");
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.setTxBufferSize(4096);
  Serial.begin(921600);   // USB CDC: baud rate irrelevant
  Serial.setTxTimeoutMs(2000);  // default 100 ms discards data if the host briefly does not read

  const uint8_t ins[] = {PIN_PCLK, PIN_VSYNC, PIN_HREF, PIN_D4, PIN_D4 + 1, PIN_D4 + 2, PIN_D4 + 3};
  for (uint8_t p : ins) pinMode(p, INPUT);
  static dedic_gpio_bundle_handle_t bundle;
  dedic_gpio_bundle_config_t bcfg = {};
  bcfg.gpio_array = DEDIC_PINS;
  bcfg.array_size = sizeof(DEDIC_PINS) / sizeof(DEDIC_PINS[0]);
  bcfg.flags.in_en = 1;
  if (dedic_gpio_new_bundle(&bcfg, &bundle) != ESP_OK) Serial.println("ERR dedic_gpio");

  startXclk();
  delay(10);
  Wire.begin(PIN_SDA, PIN_SCL, 100000);
  delay(1000);            // time for the host to connect

  if (!sensorInit(gMode)) Serial.println("ERR sensor not responding (SCCB)");
  else               Serial.printf("READY pid=%02X ver=%02X\n", rd(0x0A), rd(0x0B));
}

void loop() {
  if (!Serial.available()) { delay(1); return; }
  String l = Serial.readStringUntil('\n');
  l.trim();
  if (l.isEmpty()) return;

  const char c = l[0];
  const char* a = l.c_str() + 1;
  if (c == 'C') {
    int x = 0, y = 0, w = 9999, h = 9999, s = 1, p = 1, bh = 1, bv = 1;  // p=1: UYVY (measured)
    sscanf(a, "%d %d %d %d %d %d %d %d", &x, &y, &w, &h, &s, &p, &bh, &bv);
    cmdCapture(x, y, w, h, s, p, bh, bv);
  } else if (c == 'S') {
    int x = 0, y = 0, w = 9999, h = 8, th = 128, pol = 0, prof = 0;
    sscanf(a, "%d %d %d %d %d %d %d", &x, &y, &w, &h, &th, &pol, &prof);
    cmdScan(x, y, w, h, th, pol, prof);
  } else if (c == 'U') {
    int x = 0, y = 0, w = 9999, h = 8, k10 = 50, dmin = 17, mass = 2, learn = 32, prof = 0, lo = 50;
    int comp = 1;
    sscanf(a, "%d %d %d %d %d %d %d %d %d %d %d", &x, &y, &w, &h, &k10, &dmin, &mass, &learn, &prof,
           &lo, &comp);
    cmdChange(x, y, w, h, k10, dmin, mass, learn, prof, lo, comp);
  } else if (c == 'W') {
    unsigned r, v;
    if (sscanf(a, "%x %x", &r, &v) == 2 && wr(r, v)) Serial.println("OK");
    else Serial.println("ERR write");
  } else if (c == 'R') {
    unsigned r;
    if (sscanf(a, "%x", &r) == 1) Serial.printf("REG %02X %02X\n", r, rd(r) & 0xFF);
    else Serial.println("ERR read");
  } else if (c == 'P') {
    // Diagnostics: count edges per camera pin in ~100 ms -> reveals wiring faults
    const uint8_t pins[] = {4, 6, 5, 0, 1, 2, 3};   // Dedicated GPIO channels of PCLK, VSYNC, HREF, D4..D7
    const char* names[] = {"PCLK", "VSYNC", "HREF", "D4", "D5", "D6", "D7"};
    uint32_t cnt[7] = {0}, prev = GIN(), lvl = prev;
    const uint32_t t0 = millis();
    while (millis() - t0 < 100) {
      const uint32_t v = GIN(), d = v ^ prev;
      prev = v;
      if (d) for (int i = 0; i < 7; ++i) if (d & (1u << pins[i])) ++cnt[i];
    }
    Serial.print("PINS");
    for (int i = 0; i < 7; ++i)
      Serial.printf(" %s=%u/%d", names[i], (unsigned)cnt[i], (lvl >> pins[i]) & 1);
    Serial.println();
  } else if (c == 'T') {
    // Measure frame period (VSYNC rising -> rising) and count HREF lines in between.
    // Without critical section: an ISR can swallow a line, count is accurate to +-1.
    if (!waitLevelMs(M_VSYNC, false, 15000) || !waitLevelMs(M_VSYNC, true, 15000)) {
      Serial.println("ERR no VSYNC"); return;
    }
    const uint32_t t0 = micros();
    uint32_t tVsLow = 0, lines = 0, prev = GIN();
    for (;;) {
      const uint32_t v = GIN();
      if ((v & M_HREF) && !(prev & M_HREF)) ++lines;
      if (!(v & M_VSYNC) && (prev & M_VSYNC) && !tVsLow) tVsLow = micros();
      if ((v & M_VSYNC) && !(prev & M_VSYNC)) break;
      if (micros() - t0 > 15000000) { Serial.println("ERR timeout"); return; }
      prev = v;
    }
    Serial.printf("TIM %u %u %u\n", (unsigned)(micros() - t0), (unsigned)(tVsLow - t0), (unsigned)lines);
  } else if (c == 'D') {
    sendDelayMs = atoi(a);
    Serial.println("OK");
  } else if (c == 'M') {
    int m;
    if (sscanf(a, "%d", &m) == 1) {
      if (m < 0 || m > 2 || !sensorInit(m)) { Serial.println("ERR mode"); return; }
    }
    Serial.printf("MODE %d %d %d\n", gMode, MODES[gMode].w, MODES[gMode].h);
  } else if (c == 'E') {
    int n;
    if (sscanf(a, "%d", &n) == 1) {
      if (n <= 0) wr(0x13, rd(0x13) | 0x01);        // AEC on
      else setExposure((uint32_t)min(n, 0xFFFF));
    }
    const uint32_t e = getExposure();
    Serial.printf("EXP %u %u %d\n", (unsigned)e, (unsigned)(e * MODES[gMode].period_us / 510),
                  rd(0x13) & 0x01);
  } else if (c == 'G') {
    int n;
    if (sscanf(a, "%d", &n) == 1) {
      if (n < 0) wr(0x13, rd(0x13) | 0x04);         // AGC on
      else setGain((uint32_t)min(n, 1023));
    }
    Serial.printf("GAIN %u %d\n", (unsigned)getGain(), (rd(0x13) >> 2) & 0x01);
  } else if (c == 'I') {
    Serial.printf("ID %02X %02X\n", rd(0x0A), rd(0x0B));
  } else {
    Serial.println("ERR unknown");
  }
}
