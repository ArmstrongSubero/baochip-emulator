"""
bao_emu.py: build a Dabao SDK program and run it on baoemu

Copyright (c) 2026 Armstrong Subero
SPDX-License-Identifier: Apache-2.0

Usage:
  bao_emu [--sdk PATH] list
  bao_emu [--sdk PATH] run  <program> [baoemu options]
  bao_emu [--sdk PATH] view <program> [viewer options]
  bao_emu [--sdk PATH] test

A program is an example in the SDK's examples folder or one of baoemu's
own test programs in tests/programs. The SDK is found from --sdk, then
DABAO_SDK, then a dabao-sdk folder beside baoemu.
"""

import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROGRAMS = os.path.join(HERE, "tests", "programs")
WIN = sys.platform == "win32"

PANEL = ["--attach", "st7796:2:0:dc=PC10,cs=PC6", "--attach", "ft6336:0:0x38"]
VIEW = {"lcd": ["--dc", "PC10", "--cs", "PC6"]}

PARTS = {
    "spi_async": ["--spi-loopback"],
    "ch07_701": ["--spi-loopback"],
    "i2c_mpu6050": ["--mpu6050"],
    "i2c_async": ["--mpu6050"],
    "ch08_801": ["--mpu6050"],
    "ch08_802": ["--mpu6050"],
    "w25q_flash": ["--w25q", "@flash.bin"],
    "fatfs": ["--sd", "@sd.img"],
    "ch20_2001": ["--sd", "@sd.img"],
    "ch20_2002": ["--sd", "@sd.img"],
    "ch20_2003": ["--sd", "@sd.img"],
    "adc_read": ["--adc", "0=604"],
    "ch10_1001": ["--adc", "0=604"],
    "ch10_1002": ["--adc", "0=604"],
    "ch10_1003": ["--adc", "0=604"],
    "ch10_1004": ["--adc", "0=604"],
    "rram": ["--rram", "@rram.bin"],
    "lcd": PANEL + ["--screenshot", "@screen.ppm"],
}


def die(msg):
    print(msg)
    sys.exit(1)


def is_sdk(p):
    return bool(p) and os.path.isfile(os.path.join(p, "bao_flash.sh")) and os.path.isdir(os.path.join(p, "examples"))


def find_sdk(given):
    if given:
        if is_sdk(given):
            return os.path.abspath(given)
        die("%s is not a Dabao SDK folder (no bao_flash.sh or examples)." % given)
    env = os.environ.get("DABAO_SDK")
    if env:
        if is_sdk(env):
            return os.path.abspath(env)
        die("DABAO_SDK is set to %s, which is not a Dabao SDK folder." % env)
    up = os.path.dirname(HERE)
    for n in sorted(os.listdir(up)):
        p = os.path.join(up, n)
        if n.lower().startswith("dabao") and is_sdk(p):
            return p
    if is_sdk(up):
        return up
    die("Dabao SDK not found. Put it beside baoemu, set DABAO_SDK, or pass --sdk PATH.")


def sdk_examples(sdk):
    d = os.path.join(sdk, "examples")
    return sorted(n for n in os.listdir(d) if os.path.isdir(os.path.join(d, n)))


def own_programs():
    return sorted(n for n in os.listdir(PROGRAMS) if os.path.isdir(os.path.join(PROGRAMS, n)))


def emulator():
    exe = "baoemu.exe" if WIN else "baoemu"
    plat = "windows" if WIN else "linux"
    for p in (os.path.join(HERE, "build", exe), os.path.join(HERE, "bin", plat, exe)):
        if os.path.isfile(p):
            return p
    die("baoemu not found. Build it first: make (build.bat on Windows).")


def library():
    lib = "baoemu.dll" if WIN else "libbaoemu.so"
    plat = "windows" if WIN else "linux"
    for p in (os.path.join(HERE, "build", lib), os.path.join(HERE, "bin", plat, lib)):
        if os.path.isfile(p):
            return p
    return None


def build(sdk, name, quiet=False):
    staged = None
    if name not in sdk_examples(sdk):
        if name not in own_programs():
            die("No program named %s. Run: bao_emu list" % name)
        staged = os.path.join(sdk, "examples", name)
        shutil.copytree(os.path.join(PROGRAMS, name), staged)
    try:
        if WIN:
            cmd = ["cmd", "/c", os.path.join(sdk, "bao_flash.bat"), "build", name]
        else:
            cmd = ["bash", os.path.join(sdk, "bao_flash.sh"), "build", name]
        out = subprocess.run(cmd, cwd=sdk, capture_output=True, text=True)
    finally:
        if staged:
            shutil.rmtree(staged, ignore_errors=True)
    elf = os.path.join(sdk, "build", name + ".elf")
    if out.returncode != 0 or "BUILD SUCCESS" not in out.stdout or not os.path.isfile(elf):
        if not quiet:
            print(out.stdout + out.stderr)
        return None
    return elf


def build_or_die(sdk, name):
    elf = build(sdk, name)
    if not elf:
        die("Build failed: %s" % name)
    return elf


def sd_image(path):
    if os.path.isfile(path):
        return
    try:
        from pyfatfs.PyFat import PyFat
        from pyfatfs.PyFatFS import PyFatFS
    except ImportError:
        die("This program needs an SD card image. pip install pyfatfs, or pass --sd your.img")
    with open(path, "wb") as f:
        f.truncate(64 * 1024 * 1024)
    pf = PyFat()
    pf.mkfs(path, fat_type=32, label="DABAO")
    pf.close()
    fs = PyFatFS(path)
    fs.writetext("/HOST.TXT", "hello from the host\n")
    fs.close()


def parts(sdk, name, user):
    args = []
    for a in PARTS.get(name, []):
        if a.startswith("@"):
            a = os.path.join(sdk, "build", name + "_" + a[1:])
            if a.endswith(".img") and "--sd" not in user:
                sd_image(a)
        args.append(a)
    if "--sd" in user and "--sd" in args:
        i = args.index("--sd")
        del args[i:i + 2]
    return args


def run(sdk, name, user):
    elf = build_or_die(sdk, name)
    args = [emulator()] + parts(sdk, name, user)
    if "--ms" not in user and "--cycles" not in user:
        args.append("--interactive")
    args += user + [elf]
    print("Running %s on baoemu. Ctrl+C to stop.\n" % name)
    try:
        sys.exit(subprocess.call(args))
    except KeyboardInterrupt:
        sys.exit(0)


def view(sdk, name, user):
    if name not in VIEW and "--dc" not in user:
        die("%s has no display. Use: bao_emu run %s\n"
            "For your own panel program pass --dc and --cs, for example --dc PC10 --cs PC6" % (name, name))
    elf = build_or_die(sdk, name)
    args = [sys.executable, os.path.join(HERE, "python", "bao_view.py"), elf]
    if "--dc" not in user:
        args += VIEW[name]
    env = dict(os.environ)
    if library() and not env.get("BAOEMU_LIB"):
        env["BAOEMU_LIB"] = library()
    sys.exit(subprocess.call(args + user, env=env))


def test(sdk):
    names = sorted(set(n for n in sdk_examples(sdk) if not n.startswith("zz_")) | set(own_programs()))
    print("Building %d programs ..." % len(names))
    bad = [n for n in names if not build(sdk, n, quiet=True)]
    if bad:
        print("Build failed: " + " ".join(bad))
    print("Running them on baoemu ...\n")
    env = dict(os.environ)
    if library() and not env.get("BAOEMU_LIB"):
        env["BAOEMU_LIB"] = library()
    good = [n for n in names if n not in bad]
    rc = subprocess.call([sys.executable, os.path.join(HERE, "tests", "run_examples.py"), sdk] + good, env=env)
    sys.exit(1 if rc or bad else 0)


def main():
    a = sys.argv[1:]
    given = None
    if len(a) >= 2 and a[0] == "--sdk":
        given, a = a[1], a[2:]
    if not a or a[0] not in ("list", "run", "view", "test"):
        print(__doc__.split("Usage:")[1].rstrip())
        sys.exit(0 if not a else 1)
    sdk = find_sdk(given)
    if a[0] == "list":
        print("SDK: " + sdk)
        for n in sdk_examples(sdk):
            print("  " + n + ("  (display)" if n in VIEW else ""))
        own = [n for n in own_programs() if n not in sdk_examples(sdk)]
        if own:
            print("baoemu test programs:")
            for n in own:
                print("  " + n + ("  (display)" if n in VIEW else ""))
    elif a[0] == "test":
        test(sdk)
    elif len(a) < 2:
        die("Which program? Run: bao_emu list")
    elif a[0] == "run":
        run(sdk, a[1], a[2:])
    else:
        view(sdk, a[1], a[2:])


if __name__ == "__main__":
    main()
