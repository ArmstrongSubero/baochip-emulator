"""
ff_compare.py: run every example check with fast forward on and off and compare

Copyright (c) 2026 Armstrong Subero
SPDX-License-Identifier: Apache-2.0

Usage:
  python tests/ff_compare.py <dabao-sdk folder> [name ...]

Each check in run_examples.py is run twice, once with fast forward and once
without. The result, the console text, the stop reason and the final cycle
count must all match.
"""

import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "python"))
import run_examples  # noqa: E402
from baoemu import Bao  # noqa: E402


def run_one(name, elf, tmp, ff):
    made = []

    class Rec(Bao):
        def __init__(self, guard=True, fast_forward=True):
            Bao.__init__(self, guard=guard, fast_forward=ff)
            made.append(self)

    run_examples.Bao = Rec
    result = run_examples.check(name, elf, tmp)
    chip = made[0]
    return (result, chip.console(), chip.stop_reason, chip.cycles)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    sdk = sys.argv[1]
    names = sys.argv[2:] or run_examples.all_names(sdk)
    tmp = tempfile.mkdtemp(prefix="baoemu_ff_")
    bad = 0
    for name in names:
        elf = os.path.join(sdk, "build", name + ".elf")
        if not os.path.exists(elf):
            continue
        a = run_one(name, elf, tmp, True)
        b = run_one(name, elf, tmp, False)
        same = a == b
        if not same:
            bad += 1
        print("%-18s %s" % (name, "same" if same else "DIFFERENT"))
        if not same:
            for i, what in enumerate(("result", "console", "stop", "cycles")):
                if a[i] != b[i]:
                    print("   %s differs" % what)
                    if what != "console":
                        print("     on:  %r\n     off: %r" % (a[i], b[i]))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
