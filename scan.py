#!/usr/bin/env python3
"""Spaltmessung mit ov7670_c3: Position der ersten Kante entlang eines schmalen ROI.

Die Firmware mittelt die schmale Seite des ROI (max. 8 Pixel), vergleicht mit dem Schwellenwert
und schickt je Bild "EDGE pos dir ms". Rate = Bildrate des Modus (QQVGA ~37/s, QVGA ~12/s, VGA ~9/s).

Beispiel:
  python3 scan.py /dev/ttyACM0 --mode 2 --roi 0 0 160 8 --th 100 --exp 200 --gain 0
  python3 scan.py /dev/ttyACM0 --mode 2 --roi 76 0 8 119 --th 100 --plot   # senkrechter Spalt
Taste q / Esc im Plotfenster oder Strg+C beendet.
"""
import argparse
import time

import cv2
import numpy as np
import serial

from viewer import query, wait_line


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--mode", type=int, default=2, choices=[0, 1, 2], help="0 VGA, 1 QVGA, 2 QQVGA (Default, schnellster)")
    ap.add_argument("--roi", type=int, nargs=4, required=True, metavar=("X", "Y", "W", "H"),
                    help="Spalt in Pixeln des Modus; die schmale Seite (<= 8) wird gemittelt")
    ap.add_argument("--th", type=int, default=128, help="Schwellenwert 0..255")
    ap.add_argument("--pol", type=int, default=0, choices=[0, 1, 2],
                    help="0 beide Kanten, 1 nur dunkel->hell, 2 nur hell->dunkel")
    ap.add_argument("--exp", type=int, default=None, help="Belichtung in Zeilenperioden (0 = Automatik)")
    ap.add_argument("--gain", type=int, default=None, help="Verstärkung 0..1023 (-1 = Automatik)")
    ap.add_argument("--plot", action="store_true", help="Profil, Schwelle und Kante anzeigen")
    ap.add_argument("--shot", default=None, metavar="DATEI",
                    help="mit --plot: nach 20 Messungen den Profilgraphen als PNG speichern und beenden")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 921600, timeout=1)
    time.sleep(0.2)
    ser.reset_input_buffer()
    print("MODE", *query(ser, f"M {args.mode}", "MODE"))
    if args.exp is not None:
        print("EXP", *query(ser, f"E {args.exp}", "EXP"))
    if args.gain is not None:
        print("GAIN", *query(ser, f"G {args.gain}", "GAIN"))
    x, y, w, h = args.roi
    ser.write(f"S {x} {y} {w} {h} {args.th} {args.pol} {int(args.plot)}\n".encode())
    hdr = wait_line(ser, ("SCAN",), 5).split()
    off = int(hdr[2].split("..")[0])
    print(" ".join(hdr))

    n, t0, pos = 0, time.time(), -1.0
    try:
        while True:
            line = ser.readline().decode(errors="replace").strip()
            if line.startswith("EDGE"):
                _, p, d, ms = line.split()
                pos = float(p)
                n += 1
                print(f"{ms:>8} ms  pos {p:>6}  dir {d:>2}   ({n / (time.time() - t0):.1f}/s)")
            elif line.startswith("PROF") and args.plot:
                prof = [int(v, 16) for v in line.split()[1:]]
                pimg = draw_profile(prof, args.th, pos, off)
                cv2.imshow("Profil", pimg)
                if args.shot and n >= 20:
                    cv2.imwrite(args.shot, pimg)
                    print("Profil gespeichert:", args.shot)
                    break
                if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                    break
            elif line.startswith("ERR"):
                print(line)
    except KeyboardInterrupt:
        pass
    ser.write(b"x\n")   # beliebiges Zeichen beendet die Messung
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
