/*
 * OV7670 (Modul ohne FIFO, Sensor meldet PID/VER 76/73) an ESP32-C3 SuperMini –
 * Standbild, 4 Bit (D7..D4), Graustufe (Y aus YUV422)
 *
 * Anschluss (OV7670-Modul -> SuperMini):
 *   3V3   -> 3V3            GND  -> GND
 *   RESET -> 3V3            PWDN -> GND
 *   SIOC  -> GPIO9  (SCL)   SIOD -> GPIO8  (SDA)   je 4k7 Pull-up nach 3V3, falls nicht auf dem Modul
 *   XCLK  -> GPIO0  (LEDC, 10 MHz)
 *   PCLK  -> GPIO1
 *   VSYNC -> GPIO3
 *   HREF  -> GPIO10
 *   D4    -> GPIO4   D5 -> GPIO5   D6 -> GPIO6   D7 -> GPIO7
 *   D0..D3 offen lassen.
 *
 * Warum so belegt: D4..D7 auf GPIO4..7, damit das Nibble mit einem einzigen Shift aus GPIO_IN
 * kommt. GPIO2 (Strapping) und GPIO20/21 (UART0) bleiben frei. GPIO8/9 sind auch Strapping-Pins,
 * aber ein I2C-Pull-up (high) ist dort unkritisch. Boot-Taste (GPIO9) nicht drücken, während I2C läuft.
 *
 * Der C3 hat kein Kamera-/Parallel-Interface. Die Pixel werden per Polling auf GPIO_IN_REG
 * gelesen (Dedicated GPIO). Grenze gemessen: PCLK 7,5 MHz geht, 10 MHz nicht (Bytes/Zeile fehlen).
 *
 * Bildmodi (Befehl M), Bildperiode gemessen mit T:
 *   0 VGA   640x479  107 ms  PLL x6, CLKRC 0x03
 *   1 QVGA  320x239   80 ms  PLL x4, CLKRC 0x01, DCW 2x, PCLK/2 = 5 MHz (bei 7,5 MHz / 53 ms
 *                              fehlte in ~1 % der Zeilen ein Byte, bei 5 MHz nie; gemessen)
 *   2 QQVGA 160x119   27 ms  PLL x6, CLKRC 0x00, DCW 4x, PCLK/4
 * VGA und QQVGA mit PCLK = 7,5 MHz.
 * Sensor-Windowing (VSTRT/VSTOP) verkürzt die Bildperiode NICHT (gemessen: 160 ms bei 12 Zeilen);
 * schneller geht es nur über einen höheren internen Takt, und den hält PCLK nur mit DCW-Teiler aus.
 *
 * Arduino-IDE: Board "ESP32C3 Dev Module", "USB CDC On Boot: Enabled".
 *
 * Serielles Protokoll (Textzeilen, Hex ohne 0x):
 *   C x y w h s p bh bv
 *                   -> "IMG ow oh sum16 bpp\n" + Bilddaten
 *                      x,y,w,h: ROI in Pixeln des aktuellen Modus, s: Schrittweite (Dezimierung),
 *                      p: Byte-Phase 0/1 (welches Byte von UYVY das Y ist),
 *                      bh,bv: Binning horizontal/vertikal (1..8), digitale Mittelung über bh x bv.
 *                      bpp 4: ohne Binning, 2 Pixel/Byte, linkes Pixel im High-Nibble.
 *                      bpp 8: mit Binning, 1 Pixel/Byte, Mittelwert * 17 (0..255) – die Mittelung
 *                      über mehrere 4-Bit-Werte liefert echte Zwischenstufen. Bei Binning gilt s = 1.
 *                      Hinweis: echtes Binning (Ladung im Sensor zusammenfassen) kann der OV7670
 *                      nicht; das hier verbessert Rauschen und Bittiefe, nicht die Empfindlichkeit.
 *   E [n]           -> Belichtung in Zeilenperioden setzen (0 = Automatik), "EXP lines us auto"
 *   G [n]           -> Verstärkung 0..1023 setzen (-1 = Automatik), "GAIN raw auto"
 *                      E/G ohne Argument fragen nur ab (bei Automatik: aktueller Regelwert).
 *                      Moduswechsel (M) setzt den Sensor zurück -> wieder Automatik.
 *   W rr vv         -> Register schreiben, "OK"
 *   R rr            -> "REG rr vv"
 *   T               -> "TIM period_us vsync_us lines": Bildperiode, VSYNC-High-Dauer, HREF-Zeilen je Bild
 *   D ms            -> Pause vor dem Senden nach einer Erfassung (Default 0), "OK"
 *   S x y w h th [pol [prof]]
 *                   -> Spaltmessung, läuft bis zum nächsten empfangenen Zeichen. Je Bild eine Zeile
 *                      "EDGE pos dir ms": pos = Position der ersten Kante entlang des Spalts in
 *                      Pixeln des Modus (Bruchteil interpoliert, -1 = keine), dir +1 dunkel->hell,
 *                      -1 hell->dunkel, ms = millis(). Die schmale Seite des ROI (max. 8 Pixel)
 *                      wird gemittelt (Binning), die lange ist der Spalt. th 0..255 (Wert * 17,
 *                      wie im Bild), pol 0 beide, 1 nur dunkel->hell, 2 nur hell->dunkel.
 *                      prof 1: zusätzlich "PROF v0 v1 ..." (hex) je Bild, zum Einstellen von th.
 *                      Rate = 1 Messung je Bild (VGA ~9/s, QVGA ~12/s, QQVGA ~37/s): jede
 *                      Sensorzeile kommt nur einmal pro Bild, Windowing verkürzt das Bild nicht.
 *   M n             -> Bildmodus 0/1/2 setzen, "MODE n w h"; "M" allein fragt nur ab
 *   I               -> "ID pid ver" (OV7670: 76 73)
 */
#include <Arduino.h>
#include <Wire.h>
#include "driver/dedic_gpio.h"
#include "hal/dedic_gpio_cpu_ll.h"

constexpr int PIN_XCLK  = 0;
constexpr int PIN_PCLK  = 1;
constexpr int PIN_VSYNC = 3;
constexpr int PIN_D4    = 4;   // D4..D7 = GPIO4..7, zusammenhängend
constexpr int PIN_SDA   = 8;
constexpr int PIN_SCL   = 9;
constexpr int PIN_HREF  = 10;

// Gelesen wird über Dedicated GPIO (CSR 0x804, 1 CPU-Takt) statt GPIO_IN_REG (APB-Bus).
// Mit GPIO_IN_REG kam die HREF-Erkennung zu spät: bei PCLK 5 MHz fehlte in ~20 % der Zeilen das
// erste Byte, die Zeile war um ein Byte verschoben (U/V statt Y). Kanalreihenfolge = DEDIC_PINS:
static const int DEDIC_PINS[] = {PIN_D4, PIN_D4 + 1, PIN_D4 + 2, PIN_D4 + 3, PIN_PCLK, PIN_HREF, PIN_VSYNC};
constexpr uint32_t M_PCLK  = 1u << 4;
constexpr uint32_t M_HREF  = 1u << 5;
constexpr uint32_t M_VSYNC = 1u << 6;

constexpr uint8_t  OV_ADDR  = 0x21;          // SCCB 0x42/0x43
constexpr uint32_t XCLK_HZ  = 10000000;      // OV7670: XCLK 10..48 MHz
constexpr int SENSOR_W = 640;      // Puffergrößen = VGA
// Zeile 0 geht verloren (siehe capture()), daher eine nutzbare Zeile weniger je Modus.
constexpr int SENSOR_H = 479;

struct RegVal { uint8_t r, v; };
// Fensterwerte für QVGA/QQVGA aus dem Linux-Treiber ov7670.c (hstart 164, hstop 20, vstart 14,
// vstop 494). Mit den Reset-Defaults fehlten 10 bzw. 8 Byte je Zeile (630/640, 312/320).
#define OV_WIN_DCW {0x17, 0x14}, {0x18, 0x02}, {0x32, 0xA4}, {0x19, 0x03}, {0x1A, 0x7B}, {0x03, 0x0A}
static const RegVal MODE_VGA[]   = {{0x6B, 0x8A}, {0x11, 0x03}};
static const RegVal MODE_QVGA[]  = {{0x6B, 0x4A}, {0x11, 0x01}, {0x0C, 0x04}, {0x3E, 0x19}, {0x72, 0x11}, {0x73, 0xF1}, OV_WIN_DCW};
static const RegVal MODE_QQVGA[] = {{0x6B, 0x8A}, {0x11, 0x00}, {0x0C, 0x04}, {0x3E, 0x1A}, {0x72, 0x22}, {0x73, 0xF2}, OV_WIN_DCW};
// period_us gemessen mit T. Eine Bildperiode umfasst 510 Zeilenperioden (VGA-Timing, auch bei DCW).
struct Mode { const RegVal* regs; size_t n; int w, h; uint32_t period_us; };
static const Mode MODES[] = {
  {MODE_VGA,   sizeof(MODE_VGA) / sizeof(RegVal),   640, 479, 106600},
  {MODE_QVGA,  sizeof(MODE_QVGA) / sizeof(RegVal),  320, 239, 80000},
  {MODE_QQVGA, sizeof(MODE_QQVGA) / sizeof(RegVal), 160, 119, 26700},
};
static int gMode = 1;

// Grenze für Warteschleifen innerhalb des kritischen Abschnitts. Liegt deutlich unter dem
// Interrupt-Watchdog (300 ms) und über einer Zeilendauer bei langsamem PCLK.
constexpr uint32_t SPIN_TO = 1500000;

#define GIN() dedic_gpio_cpu_ll_read_in()

alignas(4) static uint8_t lineBuf[SENSOR_W * 2 + 4];
// Binning: vertikale Summen je Spalte, zwei Spalten je 32-Bit-Wort (16-Bit-Lanes, max 8*15)
static uint32_t colAcc[SENSOR_W / 2 + 1];
static uint8_t binLut[8 * 8 * 15 + 1];   // Summe -> Mittelwert * 17
// Statisch: der Heap hat keinen zusammenhängenden Block für volle Auflösung (malloc 153 KB schlug fehl).
static uint8_t frameBuf[(SENSOR_W / 2) * SENSOR_H];
// Pause vor dem Senden. Gemessen über je 90 Bilder: 20 ms 3 Wiederholungen, 0 ms 4 -> kein Nutzen;
// der Viewer erkennt verlorene Pakete per Prüfsumme und fordert neu an.
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
  // SCCB verträgt keinen Repeated Start -> erst STOP, dann lesen
  Wire.beginTransmission(OV_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(true) != 0) return -1;
  if (Wire.requestFrom((int)OV_ADDR, 1) != 1) return -1;
  return Wire.read();
}

static void startXclk() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_XCLK, XCLK_HZ, 1);
  ledcWrite(PIN_XCLK, 1);            // 1 Bit Auflösung -> 50 %
#else
  ledcSetup(0, XCLK_HZ, 1);
  ledcAttachPin(PIN_XCLK, 0);
  ledcWrite(0, 1);
#endif
}

static bool sensorInit(int mode) {
  if (!wr(0x12, 0x80)) return false;  // COM7: Reset
  delay(100);
  wr(0x12, 0x00);                     // COM7: YUV422
  wr(0x15, 0x00);                     // COM10: VSYNC/HREF positiv, PCLK frei laufend
  const Mode& m = MODES[mode];
  for (size_t i = 0; i < m.n; ++i) wr(m.regs[i].r, m.regs[i].v);
  gMode = mode;
  delay(300);                         // PLL und AEC/AGC einschwingen lassen
  return true;
}

// ---------- Erfassung ----------
static bool IRAM_ATTR waitLevelMs(uint32_t mask, bool level, uint32_t ms) {
  uint32_t t0 = millis();
  while (((GIN() & mask) != 0) != level)
    if (millis() - t0 > ms) return false;
  return true;
}

// Liefert nullptr bei Erfolg, sonst eine Fehlermeldung.
static int gBmin, gBmax, gMissed, gShort;
static uint32_t gProcMax;   // Takte für die Verarbeitung nach einer Zeile (Diagnose)   // Byte je erfasster Zeile, Soll 2*(x+w)

// passes > 1 (nur Binning): Block k wird im Durchgang k % passes erfasst, je Durchgang ein
// neues Bild. Nach jedem Blockende folgt dann ein nicht benötigter Block, in dessen Zeit die
// Umrechnung fertig wird. Nötig in VGA: die Austastung dort ist ~6000 Takte, die Umrechnung am
// Blockende braucht bis ~12000 (gemessen); mit passes = 1 fehlte jede Blockend-Folgezeile.
// IRAM_ATTR: gegen Flash-Cache-Fehlzugriffe in der zeitkritischen Schleife. Offener Punkt: in QVGA
// mit vertikalem Binning gab es in manchen Läufen in ~1 % der Zeilen ein Byte zu wenig ("short"
// in INFO), in anderen keine; Ursache nicht gefunden, IRAM allein hat es nicht sicher behoben.
static inline bool IRAM_ATTR wantedLine(int li, int pass, int y, int yEnd, bool bin, int bv,
                                        int s, int passes) {
  if (li < y || li >= yEnd) return false;
  return bin ? ((li - y) / bv) % passes == pass : ((li - y) % s) == 0;
}

static const char* IRAM_ATTR capture(int x, int y, int w, int h, int s, int bh, int bv, int phase,
                           uint8_t* out, int ow, int oh, int rowBytes, int passes) {
  const int nBytes = 2 * (x + w);
  const bool bin = bh * bv > 1;
  // Skalierung Summe -> Mittelwert * 17 per Tabelle (spart die Multiplikation in der Austastung)
  if (bin) {
    const uint32_t n = bh * bv;
    for (uint32_t a = 0; a <= 15 * n; ++a) binLut[a] = (uint8_t)((a * 17 + n / 2) / n);
  }
  const int yEnd = bin ? y + oh * bv : y + h;
  gBmin = 99999; gBmax = 0; gMissed = 0; gShort = 0; gProcMax = 0;

  for (int pass = 0; pass < passes; ++pass) {
    // Bildanfang synchronisieren (Interrupts an, die VSYNC-Phasen sind lang)
    if (!waitLevelMs(M_VSYNC, true, 15000))  return "no VSYNC high";
    if (!waitLevelMs(M_VSYNC, false, 15000)) return "no VSYNC low";
    // Bis zur ersten Zeile vergeht die vertikale Austastlücke. Bei langsamem PCLK kann das länger
    // dauern, als der Interrupt-Watchdog erlaubt. Deshalb wird außerhalb des kritischen Abschnitts
    // gewartet. Diese erste Zeile (Sensorzeile 0) wird dafür geopfert.
    if (!waitLevelMs(M_HREF, true, 3000))   return "no HREF";

    int outRow = 0;   // nur ohne Binning; mit Binning ergibt sich die Zeile aus dem Block
    for (int li = 0; li < yEnd; ++li) {   // li = Sensorzeile li+1
      const bool want = wantedLine(li, pass, y, yEnd, bin, bv, s, passes);
      uint32_t v, t, c0 = 0;
      int b = 0;

      portENTER_CRITICAL(&mux);
      t = SPIN_TO; while (GIN() & M_HREF)    if (!--t) goto timeout;  // Ende der Vorzeile
      t = SPIN_TO; while (!(GIN() & M_HREF)) if (!--t) goto timeout;  // Start dieser Zeile
      if (want) {
        // Die Byte-Schleife ist auf Takte ausgereizt: jede Änderung (Maske, zusätzlicher Vergleich,
        // erstes Byte schon aus der HREF-Abfrage nehmen) ließ bei 7,5 MHz Bytes fehlen (gemessen).
        while (b < nBytes) {
          // Daten sind bei steigender PCLK-Flanke gültig (Sensor wechselt sie bei fallender)
          do { v = GIN(); if (!(v & M_HREF)) goto eol; } while (!(v & M_PCLK));
          // Bits 0..3 = D4..D7, 4..6 = PCLK/HREF/VSYNC, nicht hier maskieren (s. o.)
          lineBuf[b++] = (uint8_t)v;
          while (GIN() & M_PCLK) {}
        }
      eol:
        t = SPIN_TO; while (GIN() & M_HREF) if (!--t) goto timeout;   // Rest der Zeile (ROI < Breite)
        c0 = esp_cpu_get_cycle_count();
        gBmin = min(gBmin, b); gBmax = max(gBmax, b);
        if (b < nBytes) ++gShort;
        if (bin) {
          // Je Zeile nur vertikal aufsummieren, 2 Spalten pro 32-Bit-Wort (Bytes U Y V Y -> Y-Bytes
          // über Shift um 8*phase in die 16-Bit-Lanes). Horizontal summiert und per Tabelle skaliert
          // wird erst in der letzten Zeile des Blocks.
          const int r = (li - y) % bv;
          const uint32_t* w32 = (const uint32_t*)(lineBuf + 2 * x);   // x ist gerade (cmdCapture)
          const int nw = (ow * bh + 1) / 2;
          const int sh = 8 * phase;
          if (r == 0) for (int i = 0; i < nw; ++i) colAcc[i] = (w32[i] >> sh) & 0x000F000F;
          else        for (int i = 0; i < nw; ++i) colAcc[i] += (w32[i] >> sh) & 0x000F000F;
          if (r == bv - 1) {
            uint8_t* row = out + ((li - y) / bv) * rowBytes;
            const uint16_t* col = (const uint16_t*)colAcc;   // little-endian: Lane 0 = gerade Spalte
            switch (bh) {   // feste Schleifenlängen -> Compiler rollt aus
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
          // Dezimierung ohne Verzweigung; die erste Fassung (Index-Rechnung je Pixel) war für die
          // Zeilenaustastung zu langsam, dann fehlte jede zweite Zeile (Bild doppelt, gestaucht).
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
      // Ist HREF hier schon wieder high, ist der Start der Folgezeile verpasst: sie wird mitgezählt,
      // damit die Zeilennummern stimmen. Ein Fehler ("missed") ist das nur, wenn sie gebraucht wurde.
      if (want && (GIN() & M_HREF)) { if (wantedLine(li + 1, pass, y, yEnd, bin, bv, s, passes)) ++gMissed; ++li; }
      portEXIT_CRITICAL(&mux);   // anstehende ISRs (Tick, USB) laufen hier kurz
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
  if (bin) { s = 1; x &= ~1; }   // gerades x: 32-Bit-Zugriff auf lineBuf
  // Binning: angefangene Blöcke am Rand werden verworfen
  const int ow = bin ? w / bh : (w + s - 1) / s;
  const int oh = bin ? h / bv : (h + s - 1) / s;
  const int rowBytes = bin ? ow : (ow + 1) / 2;
  if (ow < 1 || oh < 1) { Serial.println("ERR roi smaller than bin"); return; }
  const size_t n = (size_t)rowBytes * oh;

  uint8_t* buf = frameBuf;
  memset(buf, 0, n);
  const int passes = (bin && gMode == 0 && oh > 1) ? 2 : 1;   // siehe capture()
  const char* err = capture(x, y, w, h, s, bh, bv, p, buf, ow, oh, rowBytes, passes);
  // Optionale Pause (Befehl D). Nach langen Interrupt-Sperren geht gelegentlich ein 64-Byte-Paket
  // verloren; Ursache in HWCDC nicht geklärt (SOF-Erkennung ist dort auskommentiert), eine Pause
  // half messbar nicht. Abgesichert wird über Prüfsumme + Wiederholung im Viewer.
  delay(sendDelayMs);
  if (err) {
    Serial.printf("ERR %s\n", err);
  } else {
    // Führendes \n und abschließendes END: das letzte USB-Paket eines Bildes blieb gelegentlich im
    // Sendepuffer hängen und kam erst vor der nächsten Antwort an (Viewer-Log: Binärrest direkt vor
    // "INFO"). Das END schiebt es hinterher; das \n sorgt dafür, dass die Kopfzeile trotzdem auf
    // einer eigenen Zeile beginnt.
    Serial.printf("\nINFO bytes/line %d..%d soll %d missed %d short %d/%d proc %u cyc\n", gBmin, gBmax, 2 * (x + w), gMissed, gShort, oh, (unsigned)gProcMax);
    uint32_t sum = 0;
    for (size_t i = 0; i < n; ++i) sum += buf[i];
    Serial.printf("IMG %d %d %u %d\n", ow, oh, (unsigned)(sum & 0xFFFF), bin ? 8 : 4);
    for (size_t o = 0; o < n; o += 1024) Serial.write(buf + o, min((size_t)1024, n - o));
    Serial.write((const uint8_t*)"\nEND\n", 5);
    Serial.flush();
  }
}

// ---------- Belichtung / Verstärkung ----------
// AEC[15:0] = AECHH(0x07)[5:0] : AECH(0x10)[7:0] : COM1(0x04)[1:0], Einheit Zeilenperiode.
// Mehr als ~510 Zeilen (eine Bildperiode) verlängert nicht automatisch das Bild; dafür wären
// Dummy-Zeilen (ADVFL/ADVFH 0x2D/0x2E) nötig.
static uint32_t getExposure() {
  return ((uint32_t)(rd(0x07) & 0x3F) << 10) | ((uint32_t)(rd(0x10) & 0xFF) << 2) | (rd(0x04) & 0x03);
}
static void setExposure(uint32_t n) {
  wr(0x13, rd(0x13) & ~0x01);                        // COM8: AEC aus
  wr(0x04, (rd(0x04) & ~0x03) | (n & 0x03));
  wr(0x10, (n >> 2) & 0xFF);
  wr(0x07, (rd(0x07) & ~0x3F) | ((n >> 10) & 0x3F));
}
// GAIN[9:0] = VREF(0x03)[7:6] : GAIN(0x00)[7:0]. VREF[3:0] sind Fensterbits -> Read-Modify-Write.
static uint32_t getGain() { return ((uint32_t)(rd(0x03) & 0xC0) << 2) | (rd(0x00) & 0xFF); }
static void setGain(uint32_t g) {
  wr(0x13, rd(0x13) & ~0x04);                        // COM8: AGC aus
  wr(0x00, g & 0xFF);
  wr(0x03, (rd(0x03) & ~0xC0) | ((g >> 2) & 0xC0));
}

// ---------- Spaltmessung ----------
static void cmdScan(int x, int y, int w, int h, int th, int pol, int prof) {
  const int W = MODES[gMode].w, H = MODES[gMode].h;
  x = constrain(x, 0, W - 1) & ~1;   // gerade, wie beim Binning nötig
  y = constrain(y, 0, H - 1);
  w = constrain(w, 1, W - x);
  h = constrain(h, 1, H - y);
  const bool horiz = w >= h;           // Spalt entlang der Zeile: h gemittelt, sonst w
  int bh = horiz ? 1 : min(w, 8), bv = horiz ? min(h, 8) : 1;
  if (bh * bv == 1) { if (horiz) bv = 2; else bh = 2; }   // capture() mittelt nur bei bh*bv > 1
  if (horiz) h = bv; else w = bh;
  const int ow = w / bh, oh = h / bv, n = horiz ? ow : oh;
  const int off = horiz ? x : y;
  Serial.printf("SCAN %s %d..%d bin %dx%d\n", horiz ? "row" : "col", off, off + n - 1, bh, bv);
  while (!Serial.available()) {
    const char* err = capture(x, y, w, h, 1, bh, bv, 1, frameBuf, ow, oh, ow, 1);
    if (err) { Serial.printf("ERR %s\n", err); delay(100); continue; }
    const uint8_t* p = frameBuf;   // ow x 1 oder 1 x oh, in beiden Fällen zusammenhängend
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
    // In einem Stück ausgeben: beim senkrechten Spalt bleibt nach der letzten Zeile nur die kurze
    // vertikale Austastung (QQVGA ~0,5 ms) bis zum nächsten VSYNC. Einzelne printf je Wert
    // dauerten länger, dann wurde jedes zweite Bild verpasst (halbe Rate, gemessen).
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
  while (Serial.available()) Serial.read();   // Abbruchzeichen verwerfen
  Serial.println("OK");
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.setTxBufferSize(4096);
  Serial.begin(921600);   // USB-CDC: Baudrate egal
  Serial.setTxTimeoutMs(2000);  // Default 100 ms verwirft Daten, wenn der Host kurz nicht abholt

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
  delay(1000);            // Zeit, damit sich der Host verbindet

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
    int x = 0, y = 0, w = 9999, h = 9999, s = 1, p = 1, bh = 1, bv = 1;  // p=1: UYVY (gemessen)
    sscanf(a, "%d %d %d %d %d %d %d %d", &x, &y, &w, &h, &s, &p, &bh, &bv);
    cmdCapture(x, y, w, h, s, p, bh, bv);
  } else if (c == 'S') {
    int x = 0, y = 0, w = 9999, h = 8, th = 128, pol = 0, prof = 0;
    sscanf(a, "%d %d %d %d %d %d %d", &x, &y, &w, &h, &th, &pol, &prof);
    cmdScan(x, y, w, h, th, pol, prof);
  } else if (c == 'W') {
    unsigned r, v;
    if (sscanf(a, "%x %x", &r, &v) == 2 && wr(r, v)) Serial.println("OK");
    else Serial.println("ERR write");
  } else if (c == 'R') {
    unsigned r;
    if (sscanf(a, "%x", &r) == 1) Serial.printf("REG %02X %02X\n", r, rd(r) & 0xFF);
    else Serial.println("ERR read");
  } else if (c == 'P') {
    // Diagnose: Flanken je Kamera-Pin in ~100 ms zählen -> zeigt Verdrahtungsfehler
    const uint8_t pins[] = {4, 6, 5, 0, 1, 2, 3};   // Dedicated-GPIO-Kanäle von PCLK, VSYNC, HREF, D4..D7
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
    // Bildtakt messen (VSYNC steigend -> steigend) und HREF-Zeilen dazwischen zählen.
    // Ohne kritischen Abschnitt: eine ISR kann eine Zeile verschlucken, Zählung ist +-1 genau.
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
      if (n <= 0) wr(0x13, rd(0x13) | 0x01);        // AEC an
      else setExposure((uint32_t)min(n, 0xFFFF));
    }
    const uint32_t e = getExposure();
    Serial.printf("EXP %u %u %d\n", (unsigned)e, (unsigned)(e * MODES[gMode].period_us / 510),
                  rd(0x13) & 0x01);
  } else if (c == 'G') {
    int n;
    if (sscanf(a, "%d", &n) == 1) {
      if (n < 0) wr(0x13, rd(0x13) | 0x04);         // AGC an
      else setGain((uint32_t)min(n, 1023));
    }
    Serial.printf("GAIN %u %d\n", (unsigned)getGain(), (rd(0x13) >> 2) & 0x01);
  } else if (c == 'I') {
    Serial.printf("ID %02X %02X\n", rd(0x0A), rd(0x0B));
  } else {
    Serial.println("ERR unknown");
  }
}
