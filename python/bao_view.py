"""
bao_view.py: window for baoemu with the RV1 panel, touch and console

Copyright (c) 2026 Armstrong Subero
SPDX-License-Identifier: Apache-2.0

Usage:
  python bao_view.py program.elf --dc PC10 --cs PC6 [options]

The ST7796 sits on SPIM2 and the FT6336 on I2C0 at 0x38 unless told
otherwise. Left mouse button is the first finger, right mouse button holds
a second finger so two touches can be down at once. Keys can be wired to
input pins with --key, for example --key z=PB3. Text typed into the
console box goes to UART2.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from baoemu import Bao  # noqa: E402

W, H = 320, 480


def main():
    import tkinter as tk
    ap = argparse.ArgumentParser(description="baoemu viewer")
    ap.add_argument("program")
    ap.add_argument("--spim", type=int, default=2)
    ap.add_argument("--cs", default=None, help="GPIO chip select of the panel, e.g. PC6")
    ap.add_argument("--dc", required=True, help="data/command pin of the panel, e.g. PC10")
    ap.add_argument("--i2c", type=int, default=0)
    ap.add_argument("--touch-int", default=None, help="touch interrupt pin, e.g. PB4")
    ap.add_argument("--no-touch", action="store_true")
    ap.add_argument("--sd", default=None, help="SD card image on SPIM2")
    ap.add_argument("--sd-cs", default=None, help="GPIO chip select of the SD card, e.g. PC7")
    ap.add_argument("--rotate", type=int, default=0, choices=(0, 90, 180, 270))
    ap.add_argument("--zoom", type=int, default=0, help="whole number zoom, default 1 portrait, 2 landscape")
    ap.add_argument("--key", action="append", default=[], help="KEY=PIN, pressed pulls the pin low")
    a = ap.parse_args()

    chip = Bao()
    opts = "dc=" + a.dc + ("," + "cs=" + a.cs if a.cs else "")
    chip.attach("st7796", a.spim, 0, opts)
    if not a.no_touch:
        chip.attach("ft6336", a.i2c, 0x38, ("int=" + a.touch_int) if a.touch_int else None)
    if a.sd:
        chip.attach("sd", 2, 0, a.sd + ("," + "cs=" + a.sd_cs if a.sd_cs else ""))
    keymap = {}
    for k in a.key:
        name, pin = k.split("=")
        keymap[name.lower()] = pin.upper()
        chip.set_input(pin.upper(), 1)
    chip.load(a.program)

    root = tk.Tk()
    root.title("baoemu  " + os.path.basename(a.program))
    vw, vh = (H, W) if a.rotate in (90, 270) else (W, H)
    z = a.zoom if a.zoom else (2 if a.rotate in (90, 270) else 1)
    canvas = tk.Canvas(root, width=vw * z, height=vh * z, bg="black", highlightthickness=0)
    canvas.grid(row=0, column=0, rowspan=2)
    holder = {"img": None}
    item = canvas.create_image(0, 0, anchor="nw")
    console = tk.Text(root, width=60, height=30, bg="white", fg="black")
    console.grid(row=0, column=1, sticky="nsew")
    entry = tk.Entry(root)
    entry.grid(row=1, column=1, sticky="ew")
    status = tk.Label(root, anchor="w")
    status.grid(row=2, column=0, columnspan=2, sticky="ew")

    def to_panel(x, y):
        x, y = x / z, y / z
        if a.rotate == 90:
            return W - 1 - y, x
        if a.rotate == 180:
            return W - 1 - x, H - 1 - y
        if a.rotate == 270:
            return y, H - 1 - x
        return x, y

    def press(ev, finger, down=True):
        px, py = to_panel(ev.x, ev.y)
        chip.touch(px, py, down, finger)

    canvas.bind("<ButtonPress-1>", lambda e: press(e, 0))
    canvas.bind("<B1-Motion>", lambda e: press(e, 0))
    canvas.bind("<ButtonRelease-1>", lambda e: press(e, 0, False))
    canvas.bind("<ButtonPress-3>", lambda e: press(e, 1))
    canvas.bind("<B3-Motion>", lambda e: press(e, 1))
    canvas.bind("<ButtonRelease-3>", lambda e: press(e, 1, False))

    def key(ev, level):
        if ev.widget is entry:
            return
        pin = keymap.get(ev.keysym.lower())
        if pin:
            chip.set_input(pin, level)

    root.bind("<KeyPress>", lambda e: key(e, 0))
    root.bind("<KeyRelease>", lambda e: key(e, 1))

    def send(ev):
        chip.uart_send(entry.get() + "\r\n")
        entry.delete(0, "end")
    entry.bind("<Return>", send)

    seen = {"text": 0, "frames": -1}
    t0 = time.time()

    def tick():
        reason = chip.run_ms(16)
        frames = chip.display_frames()
        if frames != seen["frames"]:
            seen["frames"] = frames
            ppm = chip.display_ppm(a.rotate)
            if ppm:
                img = tk.PhotoImage(data=ppm, format="PPM")
                if z > 1:
                    img = img.zoom(z)
                holder["img"] = img
                canvas.itemconfigure(item, image=img)
        text = chip.console()
        if len(text) > seen["text"]:
            console.insert("end", text[seen["text"]:].replace("\r", ""))
            console.see("end")
            seen["text"] = len(text)
        emu = chip.cycles / 350e6
        status.config(text="%.2f s emulated, %.0f%% of real time  %s %s" % (
            emu, 100.0 * emu / max(time.time() - t0, 1e-6), reason, chip.stop_message))
        if reason in ("cycle limit", "idle"):
            ahead = emu - (time.time() - t0)
            root.after(max(1, int(ahead * 1000)), tick)

    root.after(10, tick)
    root.mainloop()


if __name__ == "__main__":
    main()
