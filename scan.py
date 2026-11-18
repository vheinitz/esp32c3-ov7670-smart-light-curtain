#!/usr/bin/env python3
"""Slit measurement with ov7670_c3 along a narrow ROI. Two methods:

Threshold (default, firmware command S): first edge at which the profile crosses --th;
output per frame "EDGE pos dir ms".

Change (--change, firmware command U): background subtraction with a noise model. The firmware
first learns, over --learn frames, the mean mu and standard deviation sigma per position (keep the
slit clear during this!); threshold per position thr = k*sigma + dmin. The decision is based on
cluster mass: positions with |v - mu| > lo% * thr are candidates and contribute
min(1, |v - mu|/thr); a contiguous run counts as interrupted if its sum is >= --mass. Many weakly
changed neighbors thus weigh as much as a few strongly changed ones; isolated outliers do not count.
Illumination compensation (disable with --nocomp): if the brightness changes roughly
proportionally everywhere, the firmware estimates the factor a (median of v/mu) and compares v with
a*mu - this holds as long as an object covers less than half of the bright positions.
Output per frame "BRK first last count ms a" (first/last interrupted position, -1 = none;
a = brightness in % of the learned background).
Interrupted positions are never updated, so a stationary object remains detected. If the
background changes permanently (lighting, camera moved), relearn with key l in the plot window or
by sending "L" to the firmware.

The firmware averages the narrow side of the ROI (max. 8 pixels). Rate = frame rate of the mode
(QQVGA ~37/s, QVGA ~12/s, VGA ~9/s).

Examples:
  python3 scan.py /dev/ttyACM0 --mode 2 --roi 0 0 160 8 --th 100 --exp 200 --gain 0
  python3 scan.py /dev/ttyACM0 --mode 2 --roi 76 0 8 119 --th 100 --plot   # vertical slit
  python3 scan.py /dev/ttyACM0 --roi 0 56 160 8 --change --exp 200 --gain 0 --plot
Key q / Esc in the plot window or Ctrl+C quits, l relearns (--change).
"""
import argparse
import time

import cv2
import numpy as np
import serial

from viewer import drain, query, wait_line


def draw_profile(prof, th, pos, off, h=240):
    n = len(prof)
    w = max(320, n * 2)
    img = np.full((h, w, 3), 255, np.uint8)
    xs = (np.arange(n) * (w - 1) / max(1, n - 1)).astype(int)
    ys = (h - 1 - np.asarray(prof) * (h - 1) / 255).astype(int)
    cv2.polylines(img, [np.stack([xs, ys], 1).reshape(-1, 1, 2)], False, (0, 0, 0), 1)
    yt = int(h - 1 - th * (h - 1) / 255)
    cv2.line(img, (0, yt), (w - 1, yt), (0, 160, 0), 1)
    if pos >= 0:
        xp = int((pos - off) * (w - 1) / max(1, n - 1))
        cv2.line(img, (xp, 0), (xp, h - 1), (0, 0, 255), 1)
    return img


def draw_change(prof, mean, thr, lo=0.5, h=240):
    """Profile (black), expected value a*mu (blue), band mu +- thr (light gray) and mu +- lo*thr
    (darker), positions above thr (red) or above lo*thr (orange) - without the firmware's
    cluster mass decision, for illustration only."""
    n = len(prof)
    w = max(320, n * 2)
    img = np.full((h, w, 3), 255, np.uint8)
    xs = (np.arange(n) * (w - 1) / max(1, n - 1)).astype(int)
    p, m, t = (np.asarray(a, float) for a in (prof, mean, thr))
    Y = lambda v: (h - 1 - np.clip(v, 0, 255) * (h - 1) / 255).astype(int)
    band = np.concatenate([np.stack([xs, Y(m + t)], 1), np.stack([xs, Y(m - t)], 1)[::-1]])
    cv2.fillPoly(img, [band.reshape(-1, 1, 2)], (225, 225, 225))
    tl = t * lo
    band = np.concatenate([np.stack([xs, Y(m + tl)], 1), np.stack([xs, Y(m - tl)], 1)[::-1]])
    cv2.fillPoly(img, [band.reshape(-1, 1, 2)], (200, 200, 200))
    cv2.polylines(img, [np.stack([xs, Y(m)], 1).reshape(-1, 1, 2)], False, (200, 120, 0), 1)
    cv2.polylines(img, [np.stack([xs, Y(p)], 1).reshape(-1, 1, 2)], False, (0, 0, 0), 1)
    d = np.abs(p - m)
    for i in np.nonzero(d > tl)[0]:
        cv2.line(img, (int(xs[i]), h - 6), (int(xs[i]), h - 1),
                 (0, 0, 255) if d[i] > t[i] else (0, 160, 255), 2)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--mode", type=int, default=2, choices=[0, 1, 2], help="0 VGA, 1 QVGA, 2 QQVGA (default, fastest)")
    ap.add_argument("--roi", type=int, nargs=4, required=True, metavar=("X", "Y", "W", "H"),
                    help="slit in pixels of the mode; the narrow side (<= 8) is averaged")
    ap.add_argument("--th", type=int, default=128, help="threshold 0..255")
    ap.add_argument("--pol", type=int, default=0, choices=[0, 1, 2],
                    help="0 both edges, 1 dark->bright only, 2 bright->dark only")
    ap.add_argument("--exp", type=int, default=None, help="exposure in line periods (0 = automatic)")
    ap.add_argument("--gain", type=int, default=None, help="gain 0..1023 (-1 = automatic)")
    ap.add_argument("--change", action="store_true",
                    help="change detection instead of threshold (see above)")
    ap.add_argument("--k", type=float, default=5.0, help="--change: threshold in multiples of sigma")
    ap.add_argument("--dmin", type=int, default=17,
                    help="--change: minimum deviation 0..255 (17 = one step of the 4-bit data)")
    ap.add_argument("--mass", type=int, default=2,
                    help="--change: minimum cluster mass of a run (in positions of full deviation)")
    ap.add_argument("--lo", type=int, default=50,
                    help="--change: candidate from lo%% of the threshold (100 = only full deviation counts)")
    ap.add_argument("--nocomp", action="store_true",
                    help="--change: no illumination compensation")
    ap.add_argument("--learn", type=int, default=32, help="--change: frames for the learning phase")
    ap.add_argument("--plot", action="store_true", help="show profile, threshold and edge")
    ap.add_argument("--shot", default=None, metavar="FILE",
                    help="with --plot: after 20 measurements save the profile plot as PNG and quit")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 921600, timeout=1)
    time.sleep(0.2)
    # Stop a measurement (S/U) still running from an aborted run: it stops at the first
    # character and discards the remaining input - otherwise the first command would be lost.
    ser.write(b"x\n")
    drain(ser, 0.3)
    print("MODE", *query(ser, f"M {args.mode}", "MODE"))
    if args.exp is not None:
        print("EXP", *query(ser, f"E {args.exp}", "EXP"))
    if args.gain is not None:
        print("GAIN", *query(ser, f"G {args.gain}", "GAIN"))
    x, y, w, h = args.roi
    if args.change:
        ser.write(f"U {x} {y} {w} {h} {round(args.k * 10)} {args.dmin} {args.mass} {args.learn}"
                  f" {int(args.plot)} {args.lo} {int(not args.nocomp)}\n".encode())
        hdr = wait_line(ser, ("CHG",), 5).split()
        print(" ".join(hdr))
        print("learning background - keep slit clear ...")
        print(wait_line(ser, ("LEARNED",), 10 + args.learn))
    else:
        ser.write(f"S {x} {y} {w} {h} {args.th} {args.pol} {int(args.plot)}\n".encode())
        hdr = wait_line(ser, ("SCAN",), 5).split()
        print(" ".join(hdr))
    off = int(hdr[2].split("..")[0])
    cur = {}

    n, t0, pos = 0, time.time(), -1.0
    try:
        while True:
            line = ser.readline().decode(errors="replace").strip()
            try:
                if line.startswith("EDGE"):
                    _, p, d, ms = line.split()
                    pos = float(p)
                    n += 1
                    print(f"{ms:>8} ms  pos {p:>6}  dir {d:>2}   ({n / (time.time() - t0):.1f}/s)")
                elif line.startswith("BRK"):
                    _, f, l, c, ms, a = line.split()
                    n += 1
                    txt = "clear" if f == "-1" else f"interrupted {f}..{l} ({c} pos.)"
                    print(f"{ms:>8} ms  bright. {a:>3} %  {txt}   ({n / (time.time() - t0):.1f}/s)")
                elif args.plot and line[:4] in ("MEAN", "THR "):
                    cur[line[:4]] = [int(v, 16) for v in line.split()[1:]]
                    if (line.startswith("THR") and "PROF" in cur and "MEAN" in cur
                        and len(cur["PROF"]) == len(cur["MEAN"]) == len(cur["THR "]) > 1):
                        pimg = draw_change(cur["PROF"], cur["MEAN"], cur["THR "], args.lo / 100)
                        cv2.imshow("Profile", pimg)
                        if args.shot and n >= 20:
                            cv2.imwrite(args.shot, pimg)
                            print("Profile saved:", args.shot)
                            break
                        key = cv2.waitKey(1) & 0xFF
                        if key in (ord("q"), 27):
                            break
                        if key == ord("l"):
                            ser.write(b"L\n")
                            print("relearning - keep slit clear ...")
                elif line.startswith("PROF") and args.plot and args.change:
                    cur["PROF"] = [int(v, 16) for v in line.split()[1:]]
                elif line.startswith("PROF") and args.plot:
                    prof = [int(v, 16) for v in line.split()[1:]]
                    pimg = draw_profile(prof, args.th, pos, off)
                    cv2.imshow("Profile", pimg)
                    if args.shot and n >= 20:
                        cv2.imwrite(args.shot, pimg)
                        print("Profile saved:", args.shot)
                        break
                    if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                        break
                elif line.startswith(("ERR", "LEARNED")):
                    print(line)
            except ValueError:
                # Garbled line (missing bytes on USB CDC): discard instead of crashing
                cur.clear()
    except KeyboardInterrupt:
        pass
    ser.write(b"x\n")   # any character stops the measurement
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
