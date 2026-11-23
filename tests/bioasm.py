"""
bioasm.py: assemble a BIO core program into a C array of 32 bit words

Copyright (c) 2026 Armstrong Subero
SPDX-License-Identifier: Apache-2.0

Usage:
  python tests/bioasm.py NAME < program.s

Uses riscv64-unknown-elf-as (or riscv-none-elf-as) with rv32imc. Fields
that the BIO core would decode as x20, a FIFO or x30 are flagged.
"""

import os
import shutil
import subprocess
import sys
import tempfile


def tool(name):
    for prefix in ("riscv64-unknown-elf-", "riscv-none-elf-", "riscv32-unknown-elf-"):
        if shutil.which(prefix + name):
            return prefix + name
    sys.exit("no RISC-V %s found" % name)


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    src = sys.stdin.read()
    tmp = tempfile.mkdtemp(prefix="bioasm_")
    s = os.path.join(tmp, "p.s")
    o = os.path.join(tmp, "p.o")
    with open(s, "w") as f:
        f.write(".option norelax\n.text\n" + src + "\n")
    subprocess.check_call([tool("as"), "-march=rv32imc", "-mabi=ilp32", "-o", o, s])
    dump = subprocess.check_output([tool("objdump"), "-d", "-M", "no-aliases,numeric", o]).decode()
    insns = []
    for line in dump.splitlines():
        parts = line.split("\t")
        if len(parts) >= 3 and parts[0].strip().endswith(":"):
            try:
                addr = int(parts[0].strip()[:-1], 16)
                word = int(parts[1].strip(), 16)
            except ValueError:
                continue
            size = 4 if len(parts[1].strip()) == 8 else 2
            text = " ".join(p.strip() for p in parts[2:])
            insns.append((addr, size, word, text))
    image = bytearray()
    notes = {}
    for addr, size, word, text in insns:
        image += word.to_bytes(size, "little")
        notes.setdefault(addr // 4, []).append(text)
        if size == 4:
            rs1, rs2 = (word >> 15) & 31, (word >> 20) & 31
            if 20 in (rs1, rs2):
                sys.stderr.write("quantum halt field at 0x%03x: %s\n" % (addr, text))
            for r in (rs1, rs2):
                if 16 <= r <= 19 or r == 30:
                    sys.stderr.write("check field x%d at 0x%03x: %s\n" % (r, addr, text))
    while len(image) % 4:
        image += b"\x00\x00"
    print("static const uint32_t %s[] = {" % sys.argv[1])
    for i in range(len(image) // 4):
        w = int.from_bytes(image[4 * i:4 * i + 4], "little")
        print("    0x%08X,  /* %03X: %s */" % (w, 4 * i, "; ".join(notes.get(i, []))))
    print("};")
    return 0


if __name__ == "__main__":
    sys.exit(main())
