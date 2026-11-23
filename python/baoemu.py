"""
baoemu.py: Python bindings for the Baochip-1x emulator

Copyright (c) 2026 Armstrong Subero
SPDX-License-Identifier: Apache-2.0

Loads baoemu.dll, libbaoemu.so or libbaoemu.dylib from this folder (or
from BAOEMU_LIB) through ctypes.

    from baoemu import Bao
    chip = Bao()
    chip.load("blink.elf")
    chip.on_gpio(lambda port, out, oe: print(port, hex(out)))
    chip.run_ms(1000)
"""

import ctypes
import struct
import os
import sys

STOP_NAMES = ["running", "cycle limit", "guard", "exception", "breakpoint", "idle", "halt", "stopped"]
ACLK_HZ = 350_000_000

I2C_START, I2C_ADDR, I2C_WRITE, I2C_READ, I2C_STOP = range(5)

_LOG = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p)
_UART = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int, ctypes.c_int)
_GPIO = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32, ctypes.c_uint32)
_SPI = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int)
_SPICS = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int)
_I2C = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_uint8))
_USB = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8), ctypes.c_int)


def _load_library():
    here = os.path.dirname(os.path.abspath(__file__))
    names = {"win32": ["baoemu.dll"], "darwin": ["libbaoemu.dylib"]}.get(sys.platform, ["libbaoemu.so"])
    candidates = []
    if os.environ.get("BAOEMU_LIB"):
        candidates.append(os.environ["BAOEMU_LIB"])
    plat = "windows" if sys.platform == "win32" else "linux"
    for d in (here, os.path.join(here, ".."), os.path.join(here, "..", "build"),
              os.path.join(here, "..", "bin", plat)):
        candidates += [os.path.join(d, n) for n in names]
    for c in candidates:
        if os.path.exists(c):
            return ctypes.CDLL(os.path.abspath(c))
    raise OSError("baoemu library not found, looked for: " + ", ".join(candidates))


_lib = _load_library()

_sig = {
    "bao_new": (ctypes.c_void_p, []),
    "bao_free": (None, [ctypes.c_void_p]),
    "bao_reset": (None, [ctypes.c_void_p]),
    "bao_version": (ctypes.c_char_p, []),
    "bao_load_file": (ctypes.c_int, [ctypes.c_void_p, ctypes.c_char_p]),
    "bao_load_bin": (ctypes.c_int, [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint32]),
    "bao_rram_image_load": (ctypes.c_int, [ctypes.c_void_p, ctypes.c_char_p]),
    "bao_rram_image_save": (ctypes.c_int, [ctypes.c_void_p, ctypes.c_char_p]),
    "bao_run": (ctypes.c_int, [ctypes.c_void_p, ctypes.c_uint64]),
    "bao_stop": (None, [ctypes.c_void_p]),
    "bao_stop_reason": (ctypes.c_int, [ctypes.c_void_p]),
    "bao_stop_message": (ctypes.c_char_p, [ctypes.c_void_p]),
    "bao_cycles": (ctypes.c_uint64, [ctypes.c_void_p]),
    "bao_millis": (ctypes.c_uint64, [ctypes.c_void_p]),
    "bao_get_pc": (ctypes.c_uint32, [ctypes.c_void_p]),
    "bao_set_pc": (None, [ctypes.c_void_p, ctypes.c_uint32]),
    "bao_get_reg": (ctypes.c_uint32, [ctypes.c_void_p, ctypes.c_int]),
    "bao_set_reg": (None, [ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32]),
    "bao_mem_read": (ctypes.c_int, [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_char_p, ctypes.c_uint32]),
    "bao_peek32": (ctypes.c_int, [ctypes.c_void_p, ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint32)]),
    "bao_mem_write": (ctypes.c_int, [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_char_p, ctypes.c_uint32]),
    "bao_symbol": (ctypes.c_uint32, [ctypes.c_void_p, ctypes.c_char_p]),
    "bao_where": (ctypes.c_char_p, [ctypes.c_void_p, ctypes.c_uint32]),
    "bao_set_guard": (None, [ctypes.c_void_p, ctypes.c_int]),
    "bao_set_stop_on_exception": (None, [ctypes.c_void_p, ctypes.c_int]),
    "bao_set_uart_instant": (None, [ctypes.c_void_p, ctypes.c_int]),
    "bao_set_fast_forward": (None, [ctypes.c_void_p, ctypes.c_int]),
    "bao_skipped_cycles": (ctypes.c_uint64, [ctypes.c_void_p]),
    "bao_rram_commits": (ctypes.c_uint32, [ctypes.c_void_p]),
    "bao_on_log": (None, [ctypes.c_void_p, _LOG, ctypes.c_void_p]),
    "bao_on_uart": (None, [ctypes.c_void_p, _UART, ctypes.c_void_p]),
    "bao_on_gpio": (None, [ctypes.c_void_p, _GPIO, ctypes.c_void_p]),
    "bao_on_spi": (None, [ctypes.c_void_p, _SPI, _SPICS, ctypes.c_void_p]),
    "bao_on_i2c": (None, [ctypes.c_void_p, _I2C, ctypes.c_void_p]),
    "bao_attach": (ctypes.c_int, [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_char_p]),
    "bao_mpu6050_set": (ctypes.c_int, [ctypes.c_void_p] + [ctypes.c_int16] * 6),
    "bao_touch": (ctypes.c_int, [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int]),
    "bao_display_rgb": (ctypes.c_int, [ctypes.c_void_p, ctypes.c_char_p, ctypes.POINTER(ctypes.c_uint32)]),
    "bao_display_rgb_rot": (ctypes.c_int, [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]),
    "bao_pwm_get": (ctypes.c_int, [ctypes.c_void_p, ctypes.c_int, ctypes.c_int,
                                   ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32)]),
    "bao_uart_rx": (None, [ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]),
    "bao_gpio_set_input": (None, [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]),
    "bao_gpio_get_out": (ctypes.c_uint32, [ctypes.c_void_p, ctypes.c_int]),
    "bao_gpio_get_oe": (ctypes.c_uint32, [ctypes.c_void_p, ctypes.c_int]),
    "bao_gpio_get_af": (ctypes.c_uint32, [ctypes.c_void_p, ctypes.c_int]),
    "bao_adc_set_mv": (None, [ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32]),
    "bao_usb_attach": (None, [ctypes.c_void_p, ctypes.c_int]),
    "bao_on_usb_serial": (None, [ctypes.c_void_p, _USB, ctypes.c_void_p]),
    "bao_usb_serial_rx": (None, [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]),
    "bao_usb_serial_open": (ctypes.c_int, [ctypes.c_void_p]),
    "bao_usb_disk_blocks": (ctypes.c_uint32, [ctypes.c_void_p]),
    "bao_usb_disk_block_size": (ctypes.c_uint32, [ctypes.c_void_p]),
    "bao_usb_disk_read": (ctypes.c_int, [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32]),
    "bao_usb_disk_write": (ctypes.c_int, [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_char_p]),
    "bao_usb_disk_busy": (ctypes.c_int, [ctypes.c_void_p]),
    "bao_usb_disk_data": (ctypes.c_uint32, [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint32]),
}
for _name, (_res, _args) in _sig.items():
    _f = getattr(_lib, _name)
    _f.restype = _res
    _f.argtypes = _args

PORTS = "ABCDEF"


def _pin(name):
    """'PB3' -> (1, 3)"""
    name = name.upper()
    if len(name) < 3 or name[0] != "P" or name[1] not in PORTS:
        raise ValueError("pin names look like PB3")
    return PORTS.index(name[1]), int(name[2:])


class Bao:
    def __init__(self, guard=True, fast_forward=True):
        self._h = _lib.bao_new()
        if not self._h:
            raise MemoryError("bao_new failed")
        self._keep = {}
        self.uart_text = {0: bytearray(), 1: bytearray(), 2: bytearray(), 3: bytearray()}
        self.log = []
        self._log_cb = _LOG(lambda u, lvl, msg: self.log.append((lvl, msg.decode(errors="replace"))))
        _lib.bao_on_log(self._h, self._log_cb, None)
        self._uart_user = None
        self._uart_cb = _UART(self._on_uart)
        _lib.bao_on_uart(self._h, self._uart_cb, None)
        self.usb_text = bytearray()
        self._usb_cb = _USB(lambda u, data, n: self.usb_text.extend(data[:n]))
        _lib.bao_on_usb_serial(self._h, self._usb_cb, None)
        _lib.bao_set_guard(self._h, 1 if guard else 0)
        _lib.bao_set_fast_forward(self._h, 1 if fast_forward else 0)

    def close(self):
        if self._h:
            _lib.bao_free(self._h)
            self._h = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    @staticmethod
    def version():
        return _lib.bao_version().decode()

    def load(self, path):
        if _lib.bao_load_file(self._h, os.fsencode(path)):
            raise RuntimeError("could not load %s: %s" % (path, self.last_log()))

    def reset(self):
        _lib.bao_reset(self._h)

    def run(self, cycles):
        return STOP_NAMES[_lib.bao_run(self._h, int(cycles))]

    def run_ms(self, ms):
        return self.run(int(ms * ACLK_HZ / 1000))

    def stop(self):
        _lib.bao_stop(self._h)

    @property
    def stop_reason(self):
        return STOP_NAMES[_lib.bao_stop_reason(self._h)]

    @property
    def stop_message(self):
        return _lib.bao_stop_message(self._h).decode(errors="replace")

    @property
    def cycles(self):
        return _lib.bao_cycles(self._h)

    @property
    def millis(self):
        return _lib.bao_millis(self._h)

    @property
    def pc(self):
        return _lib.bao_get_pc(self._h)

    def reg(self, n):
        return _lib.bao_get_reg(self._h, n)

    def read(self, addr, n):
        buf = ctypes.create_string_buffer(n)
        if _lib.bao_mem_read(self._h, addr, buf, n):
            raise ValueError("0x%08X is not RAM or RRAM" % addr)
        return buf.raw

    def peek32(self, addr):
        """Read a peripheral register as the CPU would, side effects included."""
        v = ctypes.c_uint32()
        if _lib.bao_peek32(self._h, addr, ctypes.byref(v)):
            raise ValueError("0x%08X is not a peripheral" % addr)
        return v.value

    def write(self, addr, data):
        if _lib.bao_mem_write(self._h, addr, bytes(data), len(data)):
            raise ValueError("0x%08X is not RAM or RRAM" % addr)

    def symbol(self, name):
        return _lib.bao_symbol(self._h, name.encode())

    def where(self, pc=None):
        return _lib.bao_where(self._h, self.pc if pc is None else pc).decode()

    def last_log(self):
        return self.log[-1][1] if self.log else ""

    def set_uart_instant(self, on=True):
        _lib.bao_set_uart_instant(self._h, 1 if on else 0)

    def set_stop_on_exception(self, on=True):
        _lib.bao_set_stop_on_exception(self._h, 1 if on else 0)

    def rram_load(self, path):
        return _lib.bao_rram_image_load(self._h, os.fsencode(path)) == 0

    def rram_save(self, path):
        return _lib.bao_rram_image_save(self._h, os.fsencode(path)) == 0

    @property
    def rram_commits(self):
        return _lib.bao_rram_commits(self._h)

    def _on_uart(self, user, inst, byte):
        self.uart_text[inst].append(byte)
        if self._uart_user:
            self._uart_user(inst, byte)

    def on_uart(self, fn):
        """fn(instance, byte) for every byte the chip transmits."""
        self._uart_user = fn

    def console(self, inst=2):
        return self.uart_text[inst].decode(errors="replace")

    def usb_attach(self, on=True):
        """Plug the PC into the USB port (the default) or leave it unplugged."""
        _lib.bao_usb_attach(self._h, 1 if on else 0)

    @property
    def usb_serial_open(self):
        return bool(_lib.bao_usb_serial_open(self._h))

    def usb_serial_send(self, data):
        """Type into the USB serial port. Held until the PC side has opened it."""
        if isinstance(data, str):
            data = data.encode()
        _lib.bao_usb_serial_rx(self._h, data, len(data))

    def usb_serial(self):
        """What the chip has sent to the PC on its USB serial port."""
        return self.usb_text.decode(errors="replace")

    @property
    def usb_disk_blocks(self):
        """Blocks on the USB disk, 0 until the PC side has found it."""
        return _lib.bao_usb_disk_blocks(self._h)

    @property
    def usb_disk_block_size(self):
        return _lib.bao_usb_disk_block_size(self._h)

    def _usb_disk_wait(self, what, timeout_ms):
        for _ in range(timeout_ms * 4):
            state = _lib.bao_usb_disk_busy(self._h)
            if state == 0:
                return
            if state < 0:
                raise IOError("USB disk %s failed" % what)
            _lib.bao_run(self._h, 87500)
        raise TimeoutError("USB disk %s did not finish" % what)

    def usb_disk_read(self, lba, count=1, timeout_ms=3000):
        """Read blocks the way the PC does. Runs the chip until the data is in."""
        bs = self.usb_disk_block_size
        out = bytearray()
        while count:
            n = min(count, 65536 // bs) if bs else 0
            if not n or _lib.bao_usb_disk_read(self._h, lba, n):
                raise IOError("USB disk is not ready or the range is bad")
            self._usb_disk_wait("read", timeout_ms)
            buf = ctypes.create_string_buffer(n * bs)
            got = _lib.bao_usb_disk_data(self._h, buf, n * bs)
            if got != n * bs:
                raise IOError("USB disk read came back short")
            out += buf.raw
            lba += n
            count -= n
        return bytes(out)

    def usb_disk_write(self, lba, data, timeout_ms=3000):
        """Write whole blocks the way the PC does. Runs the chip until they are taken."""
        bs = self.usb_disk_block_size
        if not bs or len(data) % bs:
            raise ValueError("data must be whole blocks and the disk must be ready")
        data = bytes(data)
        while data:
            n = min(len(data) // bs, 65536 // bs)
            if _lib.bao_usb_disk_write(self._h, lba, n, data[:n * bs]):
                raise IOError("USB disk is not ready or the range is bad")
            self._usb_disk_wait("write", timeout_ms)
            data = data[n * bs:]
            lba += n

    # USB disk files: FAT12 and FAT16, root directory, 8.3 names.

    def _fat(self):
        bs = self.usb_disk_block_size
        if bs != 512:
            raise IOError("USB disk is not ready")
        first = self.usb_disk_read(0)
        start = 0
        if first[510:512] == b"\x55\xaa" and first[11:13] != b"\x00\x02":
            start = struct.unpack_from("<I", first, 446 + 8)[0]
            first = self.usb_disk_read(start)
        bps, spc, rsv, nfat, nroot, tot16, _, spf = struct.unpack_from("<HBHBHHBH", first, 11)
        total = tot16 or struct.unpack_from("<I", first, 32)[0]
        if bps != 512 or not spc or not nroot:
            raise IOError("no FAT12 or FAT16 volume on the USB disk")
        fat0 = start + rsv
        root = fat0 + nfat * spf
        data0 = root + (nroot * 32 + 511) // 512
        clusters = (start + total - data0) // spc
        return dict(spc=spc, nfat=nfat, spf=spf, fat0=fat0, root=root,
                    rootblocks=data0 - root, data0=data0, clusters=clusters, fat16=clusters >= 4085)

    @staticmethod
    def _fat_get(v, fat, n):
        if v["fat16"]:
            return struct.unpack_from("<H", fat, n * 2)[0]
        w = struct.unpack_from("<H", fat, n + n // 2)[0]
        return (w >> 4) if n & 1 else (w & 0xFFF)

    @staticmethod
    def _fat_set(v, fat, n, val):
        if v["fat16"]:
            struct.pack_into("<H", fat, n * 2, val & 0xFFFF)
            return
        o = n + n // 2
        w = struct.unpack_from("<H", fat, o)[0]
        w = (w & 0x000F) | ((val & 0xFFF) << 4) if n & 1 else (w & 0xF000) | (val & 0xFFF)
        struct.pack_into("<H", fat, o, w)

    @staticmethod
    def _name83(name):
        base, _, ext = name.upper().partition(".")
        if not base or len(base) > 8 or len(ext) > 3 or "." in ext:
            raise ValueError("file names here are 8.3, such as code.py")
        return base.ljust(8).encode("ascii") + ext.ljust(3).encode("ascii")

    def _chain(self, v, fat, cl):
        end = 0xFFF8 if v["fat16"] else 0xFF8
        out = []
        while 2 <= cl < end and len(out) <= v["clusters"]:
            out.append(cl)
            cl = self._fat_get(v, fat, cl)
        return out

    def usb_disk_files(self):
        """{name: size} for the files in the root of the USB disk."""
        v = self._fat()
        root = self.usb_disk_read(v["root"], v["rootblocks"])
        out = {}
        for i in range(0, len(root), 32):
            e = root[i:i + 32]
            if e[0] == 0:
                break
            if e[0] == 0xE5 or e[11] & 0x08 or e[11] == 0x0F:
                continue
            name = e[:8].decode("ascii", "replace").rstrip()
            ext = e[8:11].decode("ascii", "replace").rstrip()
            if e[11] & 0x10:
                name += "/"
            elif ext:
                name += "." + ext
            out[name.lower()] = struct.unpack_from("<I", e, 28)[0]
        return out

    def usb_disk_get(self, name):
        """Contents of a file in the root of the USB disk."""
        v = self._fat()
        want = self._name83(name)
        root = self.usb_disk_read(v["root"], v["rootblocks"])
        fat = self.usb_disk_read(v["fat0"], v["spf"])
        for i in range(0, len(root), 32):
            e = root[i:i + 32]
            if e[0] == 0:
                break
            if e[:11] == want and not e[11] & 0x18:
                size = struct.unpack_from("<I", e, 28)[0]
                data = b""
                for cl in self._chain(v, fat, struct.unpack_from("<H", e, 26)[0]):
                    data += self.usb_disk_read(v["data0"] + (cl - 2) * v["spc"], v["spc"])
                return data[:size]
        raise FileNotFoundError(name)

    def usb_disk_put(self, name, data):
        """Write a file into the root of the USB disk, replacing one of the same name."""
        if isinstance(data, str):
            data = data.encode()
        v = self._fat()
        want = self._name83(name)
        root = bytearray(self.usb_disk_read(v["root"], v["rootblocks"]))
        fat = bytearray(self.usb_disk_read(v["fat0"], v["spf"]))
        slot = None
        for i in range(0, len(root), 32):
            e = root[i:i + 32]
            if e[:11] == want and e[0] != 0xE5 and not e[11] & 0x18:
                for cl in self._chain(v, fat, struct.unpack_from("<H", e, 26)[0]):
                    self._fat_set(v, fat, cl, 0)
                slot = i
                break
            if slot is None and e[0] in (0, 0xE5):
                slot = i
            if e[0] == 0:
                break
        if slot is None:
            raise IOError("the root directory of the USB disk is full")
        csize = v["spc"] * 512
        need = (len(data) + csize - 1) // csize
        free = [n for n in range(2, v["clusters"] + 2) if self._fat_get(v, fat, n) == 0][:need]
        if len(free) < need:
            raise IOError("not enough room on the USB disk")
        for k, cl in enumerate(free):
            self._fat_set(v, fat, cl, free[k + 1] if k + 1 < need else 0xFFFF)
            chunk = data[k * csize:(k + 1) * csize]
            self.usb_disk_write(v["data0"] + (cl - 2) * v["spc"], chunk.ljust(csize, b"\0"))
        entry = bytearray(32)
        entry[:11] = want
        entry[11] = 0x20
        struct.pack_into("<H", entry, 26, free[0] if free else 0)
        struct.pack_into("<I", entry, 28, len(data))
        was_end = root[slot] == 0
        root[slot:slot + 32] = entry
        if was_end and slot + 32 < len(root):
            root[slot + 32] = 0
        for n in range(v["nfat"]):
            self.usb_disk_write(v["fat0"] + n * v["spf"], fat)
        blk = slot // 512
        self.usb_disk_write(v["root"] + blk, root[blk * 512:(blk + 1) * 512])

    def uart_send(self, data, inst=2):
        if isinstance(data, str):
            data = data.encode()
        _lib.bao_uart_rx(self._h, inst, data, len(data))

    def on_gpio(self, fn):
        """fn(port, out, oe) when any output or direction changes."""
        cb = _GPIO(lambda u, p, out, oe: fn(p, out, oe))
        self._keep["gpio"] = cb
        _lib.bao_on_gpio(self._h, cb, None)

    def set_input(self, pin, level):
        port, n = _pin(pin)
        _lib.bao_gpio_set_input(self._h, port, n, 1 if level else 0)

    def pin(self, pin):
        port, n = _pin(pin)
        return (_lib.bao_gpio_get_out(self._h, port) >> n) & 1

    def pin_is_output(self, pin):
        port, n = _pin(pin)
        return bool((_lib.bao_gpio_get_oe(self._h, port) >> n) & 1)

    def on_spi(self, xfer, cs=None):
        """xfer(instance, cs, mosi) returns miso (or None for 0xFF);
        cs(instance, cs, active) is called on chip select changes."""
        def _x(u, inst, c, tx):
            r = xfer(inst, c, tx)
            return -1 if r is None else int(r) & 0xFF
        x = _SPI(_x)
        c = _SPICS((lambda u, i, s, a: cs(i, s, bool(a))) if cs else (lambda u, i, s, a: None))
        self._keep["spi"] = (x, c)
        _lib.bao_on_spi(self._h, x, c, None)

    def on_i2c(self, fn):
        """fn(instance, event, byte) for START, ADDR, WRITE, READ and STOP.
        Return True to ACK an ADDR or WRITE. For READ return the byte."""
        def _f(u, inst, ev, data):
            r = fn(inst, ev, data[0])
            if ev == I2C_READ:
                data[0] = 0xFF if r is None else int(r) & 0xFF
                return 1
            return 1 if r else 0
        cb = _I2C(_f)
        self._keep["i2c"] = cb
        _lib.bao_on_i2c(self._h, cb, None)

    def attach(self, kind, bus, sel=0, path=None):
        """kind: 'spi-loopback', 'w25q', 'sd', 'mpu6050'."""
        if _lib.bao_attach(self._h, kind.encode(), bus, sel, os.fsencode(path) if path else None):
            raise RuntimeError("could not attach %s: %s" % (kind, self.last_log()))

    def mpu6050_set(self, ax=0, ay=0, az=16384, gx=0, gy=0, gz=0):
        _lib.bao_mpu6050_set(self._h, ax, ay, az, gx, gy, gz)

    def touch(self, x, y, down=True, finger=0):
        """Press or release a finger on the first FT6336 (panel coordinates)."""
        return _lib.bao_touch(self._h, finger, 1 if down else 0, int(x), int(y)) == 0

    def display_rgb(self, rotate=0):
        """RGB888 bytes of the first ST7796 (320x480, or 480x320 when rotated
        by 90 or 270 degrees), or None if no panel is attached."""
        buf = ctypes.create_string_buffer(320 * 480 * 3)
        if _lib.bao_display_rgb_rot(self._h, buf, int(rotate)):
            return None
        return buf.raw

    def display_ppm(self, rotate=0):
        rgb = self.display_rgb(rotate)
        if rgb is None:
            return None
        w, h = (480, 320) if rotate in (90, 270) else (320, 480)
        return b"P6\n%d %d\n255\n" % (w, h) + rgb

    def display_frames(self):
        n = ctypes.c_uint32(0)
        if _lib.bao_display_rgb(self._h, None, ctypes.byref(n)):
            return 0
        return n.value

    def pwm(self, slice_, channel):
        """(frequency_hz, duty) of a running PWM channel, or None."""
        per, hi = ctypes.c_uint32(0), ctypes.c_uint32(0)
        if _lib.bao_pwm_get(self._h, slice_, channel, ctypes.byref(per), ctypes.byref(hi)) or not per.value:
            return None
        return ACLK_HZ / per.value, hi.value / per.value

    def adc_set_mv(self, channel, mv):
        _lib.bao_adc_set_mv(self._h, channel, int(mv))


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Run a Dabao SDK program on baoemu")
    ap.add_argument("program")
    ap.add_argument("--ms", type=float, default=1000)
    a = ap.parse_args()
    chip = Bao()
    chip.load(a.program)
    chip.on_uart(lambda inst, b: sys.stdout.write(chr(b)) if inst == 2 else None)
    reason = chip.run_ms(a.ms)
    print("\n[stop] %s %s at %.3f ms" % (reason, chip.stop_message, chip.cycles / 350000.0))
