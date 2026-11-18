#!/usr/bin/env python3
"""Viewer for ov7670_c3: fetches grayscale images over USB CDC and displays them with OpenCV.

Without binning 4 bits per pixel; with binning (digital averaging over bh x bv in the firmware) 8 bits.
Display and saved files are always 8 bit (nibble * 17 or mean * 17, respectively).

Keys in the window:
  a          toggle continuous mode (initially on)
  Space      single frame (in still mode)
  h / v      cycle horizontal / vertical binning (1, 2, 4, 8)
  e / r      shorter / longer exposure (factor 1.25; disables AEC)
  g / f      lower / higher gain (disables AGC)
             Each of these keys disables both automatic controls and freezes the other value.
             (r/f instead of Shift+E/G: OpenCV with the Qt backend under Wayland does not report
             Shift; E/G also work where Shift gets through)
  x          reset exposure and gain to automatic
  t          toggle threshold overlay (pixels >= th green)
  + / -      raise / lower threshold th (step 5)
  p          toggle byte phase (if the image is flat gray: U/V instead of Y)
  s          save as PNG
  q / Esc    quit; prints the matching scan.py parameters

Setup aid for scan.py: --slit draws the slit and marks the edge that scan.py would find with the
current th (same logic as the firmware, but on the 4-bit image).
"focus" = variance of the Laplacian (around the slit or over the whole image); larger = sharper,
only comparable relatively while turning the lens.

Modes (--mode), frame period:  0 VGA 640x479 107 ms | 1 QVGA 320x239 80 ms | 2 QQVGA 160x119 27 ms

Examples:
  python3 viewer.py /dev/ttyACM0                                # QQVGA video for setup
  python3 viewer.py /dev/ttyACM0 --slit 0 56 160 8 --th 100     # with slit and threshold
  python3 viewer.py /dev/ttyACM0 --mode 0 --step 2              # VGA, decimated
  python3 viewer.py /dev/ttyACM0 --mode 0 --roi 0 200 640 10    # VGA strip
  python3 viewer.py /dev/ttyACM0 --mode 2                       # fastest mode
  python3 viewer.py /dev/ttyACM0 --mode 0 --bin 4 1             # VGA, 4x horizontal binning
  python3 viewer.py /dev/ttyACM0 --exp 200 --gain 16            # fixed exposure/gain
"""
import argparse
import time

import cv2
import numpy as np
import serial


def wait_line(ser, prefixes, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode(errors="replace").strip()
        # Search for the keyword anywhere in the line: a reply may be preceded by binary leftovers
        # of the previous image (late USB packet), so the line does not start with the keyword.
        for p in prefixes + ("ERR",):
            i = line.find(p)
            if i >= 0 and (len(line) == i + len(p) or line[i + len(p)] == " "):
                if p == "ERR" and "ERR" not in prefixes:
                    raise RuntimeError(line[i:])
                return line[i:]
        if line and "INFO" not in line and line != "END":
            print("<", line[:80] + ("..." if len(line) > 80 else ""))
    raise TimeoutError(f"no reply {prefixes}")


def drain(ser, quiet=0.2):
    """Flush input until the line has been silent for `quiet` seconds (leftovers of a broken image)."""
    old, ser.timeout = ser.timeout, quiet
    while ser.read(4096):
        pass
    ser.timeout = old


def capture_once(ser, roi, step, phase, binning=(1, 1)):
    """Return an 8-bit image (0..255)."""
    ser.reset_input_buffer()
    x, y, w, h = roi
    bh, bv = binning
    ser.write(f"C {x} {y} {w} {h} {step} {phase} {bh} {bv}\n".encode())
    # One image takes at most ~0.5 s (VGA with binning, two passes). Previously 40 s: if an image
    # header was lost, the display stalled that long (looked like a freeze).
    f = wait_line(ser, ("IMG",), 3).split()
    ow, oh, chk = int(f[1]), int(f[2]), int(f[3])
    bpp = int(f[4]) if len(f) > 4 else 4
    rb = ow if bpp == 8 else (ow + 1) // 2
    # Data arrive over USB within a few ms (QQVGA ~10 ms); short timeout so that a stuck
    # packet is pushed quickly instead of waiting a full second.
    old, ser.timeout = ser.timeout, 0.2 + rb * oh / 200000
    data = ser.read(rb * oh)
    ser.timeout = old
    if len(data) < rb * oh:
        # The last USB packet occasionally gets stuck in the ESP32-C3 until more data follow.
        # Any request pushes it out; the reply to it is skipped by the next wait_line() call.
        ser.write(b"I\n")
        data += ser.read(rb * oh - len(data))
    if len(data) != rb * oh:
        raise RuntimeError(f"incomplete: {len(data)}/{rb * oh} bytes")
    if sum(data) & 0xFFFF != chk:
        raise RuntimeError("checksum mismatch")
    a = np.frombuffer(data, np.uint8).reshape(oh, rb)
    if bpp == 8:
        return a.copy()
    img = np.empty((oh, rb * 2), np.uint8)
    img[:, 0::2] = a >> 4
    img[:, 1::2] = a & 0x0F
    return img[:, :ow] * 17


def capture(ser, roi, step, phase, binning=(1, 1), retries=3):
    # USB CDC occasionally loses a 64-byte packet after long interrupt lockouts -> retry
    for i in range(retries):
        try:
            return capture_once(ser, roi, step, phase, binning)
        except (RuntimeError, TimeoutError, ValueError) as e:
            if str(e).startswith("ERR") or i == retries - 1:
                raise
            print("Retry:", e)
            drain(ser)


def query(ser, cmd, prefix):
    ser.write((cmd + "\n").encode())
    return wait_line(ser, (prefix,), 5).split()[1:]


def to_display(img, min_h=360, min_w=480, th=None):
    """Upscale by an integer factor (nearest). Height and width separately, so that a
    10-row strip becomes visible (it is stretched vertically).
    Returns (image, fx, fy). th: color pixels >= th green."""
    fy = max(1, -(-min_h // img.shape[0]))
    fx = max(1, -(-min_w // img.shape[1]))
    disp = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if th is not None:
        m = img >= th
        disp[m] = (disp[m] * 0.5 + np.array([0, 127, 0])).astype(np.uint8)
    if fx > 1 or fy > 1:
        disp = cv2.resize(disp, None, fx=fx, fy=fy, interpolation=cv2.INTER_NEAREST)
    return disp, fx, fy


def first_edge(prof, th):
    """Like the firmware (command S): first threshold crossing, linearly interpolated."""
    for i in range(1, len(prof)):
        a, b = prof[i - 1] >= th, prof[i] >= th
        if a != b:
            return i - 1 + (th - prof[i - 1]) / (prof[i] - prof[i - 1]), (1 if b else -1)
    return -1.0, 0


def focus(img):
    return float(cv2.Laplacian(img, cv2.CV_64F).var()) if img.size and min(img.shape) >= 3 else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--mode", type=int, default=2, choices=[0, 1, 2],
                    help="0 VGA, 1 QVGA, 2 QQVGA (default)")
    ap.add_argument("--slit", type=int, nargs=4, default=None, metavar=("X", "Y", "W", "H"),
                    help="draw slit as with scan.py --roi and show the edge")
    ap.add_argument("--th", type=int, default=128, help="threshold 0..255 for overlay/slit")
    ap.add_argument("--roi", type=int, nargs=4, default=None, metavar=("X", "Y", "W", "H"),
                    help="region in pixels of the mode (default: whole image)")
    ap.add_argument("--step", type=int, default=1, help="decimation (1 = full resolution)")
    ap.add_argument("--bin", type=int, nargs=2, default=[1, 1], metavar=("BH", "BV"),
                    help="horizontal/vertical binning 1..8 (digital averaging, 8-bit output)")
    ap.add_argument("--exp", type=int, default=None,
                    help="exposure in line periods (0 = automatic)")
    ap.add_argument("--gain", type=int, default=None, help="gain 0..1023 (-1 = automatic)")
    ap.add_argument("--phase", type=int, default=1, choices=[0, 1])
    ap.add_argument("--delay", type=int, default=None,
                    help="delay before sending [ms] (firmware default 0)")
    ap.add_argument("--reg", action="append", default=[], metavar="RR=VV",
                    help="write additional sensor registers beforehand (hex)")
    ap.add_argument("--still", action="store_true", help="start in still mode")
    ap.add_argument("--shot", default=None, metavar="FILE",
                    help="after 3 s save the window image as PNG and quit")
    ap.add_argument("--fps", type=float, default=3.0,
                    help="frame rate in continuous mode (default 3; 0 = as fast as possible)")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 921600, timeout=1)
    time.sleep(0.2)
    # A previously aborted run may have left the firmware in the middle of a transfer
    for i in range(3):
        drain(ser)
        ser.write(b"I\n")
        try:
            print(wait_line(ser, ("ID",), 3))
            break
        except TimeoutError:
            if i == 2:
                raise
    if args.delay is not None:
        ser.write(f"D {args.delay}\n".encode())
        wait_line(ser, ("OK",), 3)
    ser.write(f"M {args.mode}\n".encode())
    _, _, mw, mh = wait_line(ser, ("MODE",), 5).split()
    if args.roi is None:
        args.roi = [0, 0, int(mw), int(mh)]
    for r in args.reg:
        rr, vv = r.split("=")
        ser.write(f"W {rr} {vv}\n".encode())
        wait_line(ser, ("OK",), 3)
    if args.exp is not None:
        query(ser, f"E {args.exp}", "EXP")
    if args.gain is not None:
        query(ser, f"G {args.gain}", "GAIN")
    bins = [1, 2, 4, 8]
    binning = list(args.bin)

    win = "OV7670"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    phase, auto, img = args.phase, not args.still, None
    last, fps, n, want_one = None, 0.0, 0, True
    expgain, t_eg = "", 0.0
    th, show_th, eg_vals = args.th, False, ("?", "?")

    t_start = time.time()
    while True:
        due = auto and (args.fps <= 0 or last is None or time.time() - last >= 1 / args.fps)
        if due or want_one:
            want_one = False
            t0 = time.time()
            status = ""
            try:
                img = capture(ser, args.roi, args.step, phase, binning)
                n += 1
            except Exception as e:  # display should keep running on transfer errors
                status = f"Error: {e}"
                print(status)
                drain(ser)
            now = time.time()
            if auto and last is not None:
                f = 1 / (now - last)
                fps = 0.8 * fps + 0.2 * f if fps else f
            last = now
            if now - t_eg > 0.5 or not expgain:   # control values change continuously in automatic mode
                try:
                    e, us, ae = query(ser, "E", "EXP")
                    g, ag = query(ser, "G", "GAIN")
                    eg_vals = (e if ae == "0" else "0", g if ag == "0" else "-1")
                    expgain = (f"exp {e} ({int(us) / 1000:.1f}ms){' auto' if ae == '1' else ''}"
                               f"  gain {g}{' auto' if ag == '1' else ''}")
                except Exception as ex:
                    expgain = f"exp/gain: {ex}"
                t_eg = now
            if img is not None:
                disp, fx, fy = to_display(img, th=th if show_th else None)
                txt = status or (f"#{n} {img.shape[1]}x{img.shape[0]} bin {binning[0]}x{binning[1]}"
                                 f" {now - t0:.2f}s" + (f" {fps:.1f}fps" if auto else " still"))
                lines = [txt, expgain]
                # Slit in image coordinates (remove ROI offset, decimation and binning)
                sx = args.step * binning[0]
                sy = args.step * binning[1]
                if args.slit:
                    X, Y, W, H = args.slit
                    x0, y0 = (X - args.roi[0]) // sx, (Y - args.roi[1]) // sy
                    x1, y1 = x0 + max(1, W // sx), y0 + max(1, H // sy)
                    cv2.rectangle(disp, (x0 * fx, y0 * fy), (x1 * fx - 1, y1 * fy - 1), (255, 128, 0), 1)
                    reg = img[max(0, y0):y1, max(0, x0):x1]
                    if reg.size:
                        horiz = W >= H
                        prof = reg.astype(float).mean(axis=0 if horiz else 1)
                        pos, d = first_edge(prof, th)
                        if pos >= 0:
                            if horiz:
                                xp = int((x0 + pos + 0.5) * fx)
                                cv2.line(disp, (xp, y0 * fy - 8), (xp, y1 * fy + 8), (0, 0, 255), 2)
                            else:
                                yp = int((y0 + pos + 0.5) * fy)
                                cv2.line(disp, (x0 * fx - 8, yp), (x1 * fx + 8, yp), (0, 0, 255), 2)
                        # Focus in the neighborhood of the slit (the slit itself is too narrow for the Laplacian)
                        pad = 8
                        env = img[max(0, y0 - pad):y1 + pad, max(0, x0 - pad):x1 + pad]
                        lines.append(f"th {th}  edge {pos:.1f} dir {d:+d}  focus {focus(env):.0f}")
                else:
                    lines.append(f"th {th}  focus {focus(img):.0f}")
                for i, t in enumerate(lines):
                    cv2.putText(disp, t, (5, 18 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 0, 255), 1)
                cv2.imshow(win, disp)
                # only after 3 s: the mode change at startup resets the sensor, and the automatic
                # control keeps adjusting afterwards (with VGA binning over 2 frames this caused stripes)
                if args.shot and n >= 2 and now - t_start > 3:
                    cv2.imwrite(args.shot, disp)
                    print("Window image saved:", args.shot)
                    break

        key = cv2.waitKey(20) & 0xFF   # keeps the window responsive between frames
        if key in (ord("q"), 27) or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
            break
        if key == ord("a"):
            auto, last, fps, want_one = not auto, None, 0.0, True
        elif key == ord(" "):
            want_one = True
        elif key in (ord("h"), ord("v")):
            i = 0 if key == ord("h") else 1
            binning[i] = bins[(bins.index(binning[i]) + 1) % len(bins)] if binning[i] in bins else 1
            want_one = True
        elif key in (ord("e"), ord("E"), ord("r"), ord("g"), ord("G"), ord("f"), ord("x")):
            try:
                if key == ord("x"):
                    query(ser, "E 0", "EXP")
                    query(ser, "G -1", "GAIN")
                else:
                    # Disable both automatic controls: otherwise AGC immediately compensates an exposure
                    # change via the gain (and vice versa), and the image jumps back.
                    e, _, ae = query(ser, "E", "EXP")
                    g, ag = query(ser, "G", "GAIN")
                    if ae == "1":
                        query(ser, f"E {max(1, int(e))}", "EXP")   # E 0 would mean automatic again
                    if ag == "1":
                        query(ser, f"G {g}", "GAIN")
                if key in (ord("e"), ord("E"), ord("r")):
                    e = int(query(ser, "E", "EXP")[0])
                    e = max(1, int(e * 1.25) + 1 if key in (ord("E"), ord("r")) else int(e / 1.25))
                    query(ser, f"E {e}", "EXP")
                elif key != ord("x"):
                    g = int(query(ser, "G", "GAIN")[0])
                    up = key in (ord("G"), ord("f"))
                    g = min(1023, g + max(1, g // 8)) if up else max(0, g - max(1, g // 8))
                    query(ser, f"G {g}", "GAIN")
            except Exception as ex:
                print("exp/gain:", ex)
            t_eg, want_one = 0.0, True
        elif key == ord("t"):
            show_th, want_one = not show_th, True
        elif key in (ord("+"), ord("=")):
            th, want_one = min(255, th + 5), True
        elif key == ord("-"):
            th, want_one = max(0, th - 5), True
        elif key == ord("p"):
            phase ^= 1
            want_one = True
        elif key == ord("s") and img is not None:
            fn = time.strftime("ov7670_%Y%m%d_%H%M%S.png")
            cv2.imwrite(fn, img)
            print("saved:", fn)

    cv2.destroyAllWindows()
    roi = " ".join(map(str, args.slit)) if args.slit else "X Y W H"
    print(f"scan.py parameters: --mode {args.mode} --roi {roi} --th {th}"
          f" --exp {eg_vals[0]} --gain {eg_vals[1]}")


if __name__ == "__main__":
    main()
