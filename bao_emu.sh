#!/bin/bash
# bao_emu.sh: build a Dabao SDK program and run it on baoemu
#
# Usage:
#   ./bao_emu.sh [--sdk PATH] list
#   ./bao_emu.sh [--sdk PATH] run  <program> [baoemu options]
#   ./bao_emu.sh [--sdk PATH] view <program> [viewer options]
#   ./bao_emu.sh [--sdk PATH] test

exec python3 "$(cd "$(dirname "$0")" && pwd)/bao_emu.py" "$@"
