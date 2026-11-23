# Makefile: baoemu command line tool and shared library
#
# Copyright (c) 2026 Armstrong Subero
# SPDX-License-Identifier: Apache-2.0

CC      ?= gcc
CFLAGS  ?= -O2 -std=c99 -Wall -Wextra
LIBSRC  = src/core.c src/cpu.c src/bus.c src/udma.c src/sce.c src/devices.c src/bio.c src/pke.c src/pwm.c src/mdma.c src/mbox.c src/usb.c

UNAME := $(shell uname -s 2>/dev/null)
ifeq ($(OS),Windows_NT)
  LIB = build/baoemu.dll
  EXE = build/baoemu.exe
else ifeq ($(UNAME),Darwin)
  LIB = build/libbaoemu.dylib
  EXE = build/baoemu
else
  LIB = build/libbaoemu.so
  EXE = build/baoemu
endif

all: $(EXE) $(LIB)

build:
	mkdir -p build

$(EXE): $(LIBSRC) src/main.c src/*.h | build
	$(CC) $(CFLAGS) -o $@ $(LIBSRC) src/main.c

$(LIB): $(LIBSRC) src/*.h | build
	$(CC) $(CFLAGS) -shared -fPIC -fvisibility=hidden -o $@ $(LIBSRC)

clean:
	rm -rf build

.PHONY: all clean
