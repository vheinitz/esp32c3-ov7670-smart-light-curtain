# esp32c3-ov7670-smart-light-curtain

**Low-cost smart light barrier:** an OV7670 camera module (without FIFO) on an ESP32-C3
SuperMini used as a **4-bit line-scan camera**. Instead of a plain "beam interrupted yes/no" it
reports, along a narrow slit, **where** the beam is interrupted, e.g. the height of a passing
tube. Technically this is a camera-based **measuring light curtain**.

- Up to **37 measurements/s**, 160 positions along the slit (QQVGA)
- The narrow side of the slit (up to 8 pixels) is averaged. This lowers noise and yields finer
  steps than 4 bits.
- Two evaluation methods, both running entirely on the ESP32-C3; only one text line per
  measurement goes over USB:
  - **Threshold:** position of the first dark/bright edge, interpolated to a fraction of a pixel
  - **Change detection:** background subtraction with a per-position noise model, no fixed
    threshold needed (see below)
- Fixed exposure and gain
- 160×119 live image as an adjustment aid for focus, exposure, gain and threshold
- Hardware for a few euros: camera module + ESP32-C3 SuperMini, no extra parts except possibly
  two I2C pull-ups

## Adjustment: live image

![Adjustment view with slit (blue), detected edge (red) and focus value](docs/img/viewer_qqvga_slit.png)

`viewer.py` shows the 160×119 image with the slit drawn in. The red mark is the edge the
threshold measurement would find with the current threshold. The focus value (variance of the
Laplacian around the slit) peaks when the lens is in focus.

## Measuring with a threshold

![Profile along the slit with threshold (green) and first edge (red)](docs/img/scan_profile.png)

```
$ python3 scan.py /dev/ttyACM0 --roi 0 56 160 8 --th 100 --plot
```

The firmware sends one line per frame, `EDGE pos dir ms`: `pos` is the edge position in pixels
along the slit (-1 = no edge), `dir` +1 for dark→bright, −1 for bright→dark, `ms` the firmware
timestamp. Consecutive timestamps are 27 ms apart (~37/s).

## Measuring by change detection

```
$ python3 scan.py /dev/ttyACM0 --roi 0 56 160 8 --change --exp 300 --gain 40 --plot
```

With `--plot` the window shows the current profile (black), the expected background a·μ
(blue) and the bands ±thr and ±lo·thr (gray); candidates are marked orange, positions above
thr red. No threshold to set. The method:

1. **Learning:** for `--learn` frames (default 32, ~1 s) the firmware learns mean μ and standard
   deviation σ per position. **Keep the slit clear while learning.** Per position the threshold
   is thr = k·σ + dmin (`--k` default 5, `--dmin` default 17 = one step of the 4-bit data,
   needed because σ can be 0).
2. **Cluster mass:** a position deviating by more than `--lo` % of thr (default 50) is a
   candidate and contributes min(1, |v − μ| / thr). A run of adjacent candidates counts as
   interrupted if its sum reaches `--mass` (default 2). An object always changes several
   neighbours, noise is spatially independent; so wide low-contrast objects are detected as well
   as narrow high-contrast ones, while single outliers are ignored. `--lo 100` gives the plain
   run-length rule.
3. **Illumination compensation:** if the brightness changes roughly proportionally everywhere,
   the firmware estimates the factor a as the median of v/μ over the usable positions (not dark,
   not saturated) and compares v with a·μ. This holds as long as an object covers less than half
   of the usable positions. `--nocomp` turns it off.
4. **Tracking:** positions that are not interrupted slowly follow the scene (EMA 1/64), so slow
   drift does not raise an alarm. Interrupted positions are never updated, so a stopped object
   stays detected. A permanent change of the background (camera moved, scene changed) is cleared
   only by re-learning: key `l` in the plot window or the character `L` sent to the firmware.

Output per frame: `BRK first last count ms a`, first/last interrupted position (-1 = none),
their count and the estimated brightness a in percent of the learned background.

**Setup:** the method needs a **bright background**, e.g. backlighting. Where the free slit is
black (value 0) only "gets brighter" can be detected, and dark positions carry no information
for the illumination compensation. Set exposure and gain so that the free slit lies roughly in
the middle of the value range.

## Hardware

| OV7670 module | ESP32-C3 SuperMini | Note |
|---|---|---|
| 3V3 / GND | 3V3 / GND | |
| RESET | 3V3 | |
| PWDN | GND | |
| SIOC | GPIO9 | 4k7 to 3V3 if not on the module |
| SIOD | GPIO8 | 4k7 to 3V3 if not on the module |
| XCLK | GPIO0 | 10 MHz from the ESP (LEDC) |
| PCLK | GPIO1 | |
| VSYNC | GPIO3 | |
| HREF | GPIO10 | |
| D4, D5, D6, D7 | GPIO4, 5, 6, 7 | only the upper nibble is read |
| D0–D3 | – | leave open |

**Mounting:** preferably rotate the camera so that the measured direction (e.g. tube height)
runs along a sensor row, and place the slit near the top of the image. The rate stays the same,
but the result is ready earlier in the frame.

## Installation

Firmware (Arduino core esp32 ≥ 3.x, board "ESP32C3 Dev Module", **USB CDC On Boot: Enabled**):

```
arduino-cli compile -u -p /dev/ttyACM0 -b esp32:esp32:esp32c3:CDCOnBoot=cdc ov7670_c3
```

Host scripts (Python 3):

```
pip install -r requirements.txt
```

## Usage

### viewer.py: adjustment aid

```
python3 viewer.py /dev/ttyACM0 --slit 0 56 160 8 --th 100
```

| Key | Function |
|---|---|
| e / r | shorter / longer exposure (turns off both automatic controls) |
| g / f | lower / higher gain (turns off both automatic controls) |
| x | exposure and gain back to automatic |
| t | tint pixels ≥ threshold green |
| + / - | change threshold (step 5) |
| a | continuous mode on/off |
| space | single frame |
| s | save image as PNG |
| q / Esc | quit; prints the matching `scan.py` parameters |

Main options: `--slit X Y W H` (like `scan.py --roi`), `--th`, `--exp`, `--gain`,
`--fps` (default 3), `--shot FILE` (save window image and quit).
All options: `python3 viewer.py -h`.

### scan.py: measurement

```
python3 scan.py /dev/ttyACM0 --roi X Y W H --th TH [--pol 0|1|2] [--exp N] [--gain N] [--plot]
python3 scan.py /dev/ttyACM0 --roi X Y W H --change [--k K] [--dmin D] [--mass M] [--lo P]
                [--learn N] [--nocomp] [--exp N] [--gain N] [--plot]
```

- `--roi`: slit in pixels (QQVGA: 160×119). The narrow side (≤ 8) is averaged, the long side is
  the measuring direction. Horizontal and vertical slits are equally fast.
- `--th`: threshold 0..255 (gray value × 17, as in the live image)
- `--pol`: 0 both edges, 1 only dark→bright, 2 only bright→dark
- `--change` and its parameters: see "Measuring by change detection"
- `--plot`: show the profile live (for adjusting); `q`/`Esc` quits, `l` re-learns (`--change`)

The measurement also runs without the PC script: any terminal sending `S …` or `U …` (see
below) gets the `EDGE`/`BRK` lines; any character stops it (with `U`: except `L`, which
re-learns).

## Serial protocol

USB CDC, text lines, decimal values (registers in hex).

| Command | Response | Meaning |
|---|---|---|
| `S x y w h th [pol [prof]]` | `SCAN …`, then per frame `EDGE pos dir ms` | threshold measurement until a character arrives; `prof 1` adds `PROF` (profile, hex) |
| `U x y w h [k10 [dmin [mass [learn [prof [lo [comp]]]]]]]` | `CHG …`, `LEARNED …`, then per frame `BRK first last count ms a` | change detection, k = k10/10; `L` re-learns, any other character stops; `prof 1` adds `PROF`/`MEAN`/`THR ` |
| `E [n]` | `EXP lines µs auto` | exposure in line periods, 0 = automatic |
| `G [n]` | `GAIN raw auto` | gain 0..1023, −1 = automatic |
| `M [n]` | `MODE n w h` | image mode: 2 = QQVGA 160×119 (default for measuring), 1 = QVGA, 0 = VGA |
| `C x y w h s p bh bv` | `IMG ow oh sum16 bpp` + data + `END` | image/ROI, for the viewer |
| `I` | `ID 76 73` | sensor ID |
| `R rr` / `W rr vv` | `REG rr vv` / `OK` | read/write sensor register |
| `T` | `TIM period_us vsync_us lines` | measure frame timing |
| `P` | `PINS …` | count edges per camera pin (wiring test) |

A mode change (`M`) resets the sensor; afterwards the automatic controls are active again.
Detailed description in the header of [`ov7670_c3/ov7670_c3.ino`](ov7670_c3/ov7670_c3.ino).

## Technical notes and limits

- **No camera interface:** the ESP32-C3 has none. Pixels are polled via *Dedicated GPIO*
  (CPU register, 1 cycle). This handles a pixel clock up to 7.5 MHz; 10 MHz fails. Via
  `GPIO_IN_REG` the first byte was lost in ~20 % of the lines already at 5 MHz.
- **Rate = frame rate:** each sensor row arrives only once per frame, hence one measurement per
  frame no matter how small the slit. Sensor windowing (VSTRT/VSTOP) does **not** shorten the
  OV7670 frame (measured). Only a higher internal clock with DCW subsampling makes it faster
  (QQVGA: PLL ×6, PCLK/4 → 27 ms per frame).
- **4 bits:** only D7..D4 are connected. Finer steps come from averaging across the slit width.
- **USB quirk:** the last USB packet of a transfer sometimes gets stuck in the ESP32-C3 until
  more data follows. Firmware and viewer push it out with an `END` or a short query; images are
  additionally protected by a checksum. At high rates single lines can arrive garbled; the
  scripts discard them.
- **Sensor:** tested with a module identifying as OV7670 (PID/VER 0x76/0x73).

## Status

Working prototype, tested on one setup. Change detection has so far been verified only against
false alarms (0 in several hundred frames on a static scene); detection of real objects and the
illumination compensation have not yet been tested on the setup.

## License

MIT, see [`LICENSE`](LICENSE).

---

*README created 2026-11-18 with Claude Code at the author's request: "Bitte bei github ein Projekt
anlegen und puschen mit readme und wenns geht mit hilfe und screenshots. Idee ist: günstige
intelligente Lichtschranke". Translated to English and extended with change detection on
2026-11-18 ("bitte bei gelegenheit pushen aber davor doku auf englisch übersetzen").
Measurements from tests on the setup on 2026-11-17/18.*
