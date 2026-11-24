# baoemu

A Baochip-1x emulator by Armstrong Subero, RV Embedded.

baoemu runs Dabao SDK programs (ELF, UF2 or raw BIN) on a model of the Baochip-1x: the VexRiscv CPU with its MMU and pipeline timing, the memory map, the UDMA peripherals, timers and interrupt arrays, GPIO, the SCE with the PKE and big number ALU, PWM, the PL230 MDMA, the mailbox, the four BIO cores, the USB device controller, and common parts that sit on the Dabao's buses. It is plain C99 with no global state and no dependencies, and builds as a command line tool and as a shared library driven from Python through ctypes.

Register behaviour comes from the Baochip-1x RTL ([baochip/baochip-1x](https://github.com/baochip/baochip-1x)), the register map in xous-core (`utralib/bao1x/*.svd`) and the Dabao SDK drivers. No code was copied from those repositories.

## Quick start

baoemu is its own project. Put it beside your copy of the Dabao SDK:

```
work/
  dabao-sdk-main/
  baoemu/
```

From the baoemu folder, `bao_emu` builds a program with the SDK's own `bao_flash` and runs it on the emulator, with the parts its wiring needs already attached (the MPU6050, the SD card, the SPI loopback, the panel and so on).

```
bao_emu list                    SDK examples and baoemu's test programs
bao_emu run hello_uart          build and run, console in the terminal, Ctrl+C to stop
bao_emu run blink --ms 3000 --trace-gpio
bao_emu view lcd                build and open the viewer window
bao_emu test                    build everything and check it all
```

On Linux, macOS or WSL use `./bao_emu.sh`. If the SDK is somewhere else, set `DABAO_SDK` to its folder or pass `--sdk PATH` first. Anything after the program name goes to baoemu (or the viewer), so `--ms`, `--button PB3=0`, `--sd card.img` and the rest all work. Without `--ms` the program runs in real time and what you type goes to UART2, like a board on a serial terminal.

You need what `bao_flash` needs: the xPack `riscv-none-elf-gcc` 15.2.0-1 toolchain, Python 3 and `pip install pure25519`. `pip install pyfatfs` lets the SD card examples make their own card image. The viewer needs tkinter, which ships with Python on Windows (`sudo apt install python3-tk` on Ubuntu).

## Building

Linux, macOS or WSL:

```
make
```

This gives `build/baoemu` and `build/libbaoemu.so` (or `.dylib`).

Windows: run `build.bat`. It uses gcc from PATH (w64devkit or MinGW) and falls back to the clang in `python -m pip install ziglang`. Prebuilt binaries are in `bin/linux` and `bin/windows`.

## Command line

```
baoemu --ms 2000 build/hello_uart.elf
baoemu --ms 3000 --trace-gpio build/blink.elf
baoemu --interactive build/my_console.elf
baoemu --ms 3000 --sd card.img build/fatfs.elf
baoemu --ms 1000 --w25q flash.bin build/w25q_flash.elf
baoemu --ms 500 --mpu6050 build/i2c_mpu6050.elf
baoemu --ms 200 --button PB3=0 build/button.elf
baoemu --ms 2000 --adc 0=604 build/adc_read.elf
baoemu --rram rram.bin build/rram.elf
baoemu --ms 800 --usb-input "hello" build/usb_cdc.elf
baoemu --interactive --usb-console circuitpython.elf
baoemu --ms 1500 --attach st7796:2:0:dc=PC10,cs=PC6 --attach ft6336:0:0x38 \
       --touch 60,400 --screenshot screen.ppm build/lcd.elf
```

UART2 (PB14) goes to stdout. Warnings, traces and the stop line go to stderr with the emulated time. Run `baoemu` with no arguments for the full option list. The exit code is 1 when the run stopped on the guard or an exception.

A PC is attached to the USB port unless `--no-usb` is given. It enumerates the device, opens the CDC serial port if there is one and finds a mass storage disk. USB serial output is printed as `[USB]` lines, and `--usb-console` makes it the console.

`--attach KIND:BUS:SELECT:OPTIONS` puts a built in part on a bus. `cs=` selects it with a GPIO instead of the SPIM chip select, `dc=` names a display's data/command pin and `int=` a touch interrupt pin.

## Python

```python
from baoemu import Bao

chip = Bao()
chip.attach("st7796", 2, 0, "dc=PC10,cs=PC6")
chip.attach("ft6336", 0, 0x38)
chip.load("build/lcd.elf")
chip.run_ms(500)
chip.touch(60, 400)
chip.run_ms(100)
open("screen.ppm", "wb").write(chip.display_ppm())
print(chip.console())
```

The USB side is reachable the same way:

```python
chip = Bao()
chip.load("circuitpython.elf")
while not chip.usb_disk_blocks:
    chip.run_ms(50)
chip.usb_disk_put("code.py", 'print("hello")\n')
chip.run_ms(3000)
print(chip.usb_serial())
```

Parts that are not built in can be written in Python with `on_spi(xfer, cs)`, `on_i2c(fn)` and `on_gpio(fn)`. `pwm(slice, channel)` returns frequency and duty, and `peek32(addr)` reads a peripheral register as the CPU would.

## Viewer

```
python python/bao_view.py ../dabao-sdk-main/build/rv1_game.elf --dc PC10 --cs PC6 --rotate 90 --sd card.img --sd-cs PC7
```

A tkinter window that acts like an RV1 on your desk: the panel, the UART2 console with a box to type into, and a status line with emulated time. It runs at real time. Left mouse is the first finger and right mouse holds a second. `--key z=PB3` wires a key to an input pin. `bao_emu view lcd` does the build and the pins for you.

## What is modelled

| Block | Coverage |
|---|---|
| CPU | RV32IMAC, Zicsr, M, S and U privilege, delegation, the AesZknPlugin scalar AES instructions, custom cache flush `0x500F` |
| CPU timing | VexRiscv pipeline as GenCramSoC.scala builds it: result latency, static branch prediction, redirects, CSR drain, divider, uncached loads, cache flushes, traps. The table is in `src/cpu.c` |
| MMU | Sv32 with 8 line TLBs per port, superpages, SUM, MXR, MPRV, sfence.vma |
| Memory | RRAM 4 MB, SRAM 2 MB (filled with 0xCD so a missing .bss clear shows), IFRAM 256 KB |
| RRAM writes | RRC staging and page commits, image kept in a file with `--rram` |
| Guard | stores into boot0 and boot1, RRC accesses other than RRC_CR, the security region and unreviewed boot chain registers stop the run |
| CGU | sysctrl registers at RTL widths and reset values, software reset through RCURST0 and RCURST1 |
| Interrupts | all 20 IRQ arrays, MIM, MIP, SIM and SIP, TickTimer and Timer0 lines |
| Timers | TickTimer, Timer0, D11CTIME, WDT, RTC, both ATimer instances, power down and wake |
| GPIO | IOX alternate functions, pull ups, eight interrupt channels, BIO handover, PWM on AF3 |
| PWM | four slices of the PULP apb_adv_timer with all channel modes and CH_EN gating |
| UDMA | UART, SPI master, I2C and ADC with clock gating, channel readback and bus timing |
| SCE | SHA-256, SHA-512 and HMAC on the hash engine, AES 128/192/256 in ECB, CBC, CTR, OFB and CFB, TRNG, PKE, big number ALU, SDMA internal channel, RAM clear, register blocks at RTL widths |
| BIO | four PicoRV32 cores stepped per aclk, x16 to x31 extensions, FIFOs, quantum dividers, snap, masked GPIO, events, pioirq lines, DMA through the filter windows |
| MDMA | PL230 with eight channels, every cycle type, and the request block |
| Mailbox | MAILBOX, MBOX_APB and MB_CLIENT with loopback, FIFOs, errors and abort |
| USB | Corigine controller modelled from its register header and the TinyUSB driver, with a host that does CDC and mass storage |
| Parts | SPI loopback, W25Q128 flash, SD card (SPI mode, SDHC), MPU6050, ST7796S 320x480 panel, FT6336U touch |

Not modelled: SDIO, I2S, the camera, QFC and XIP, the DUART, PMP (the core has none), speculative TLB fills, a reprogrammed PLL changing the CPU speed (the frequency sensors follow the clock registers, but timing stays at 350 MHz), and RIPEMD, BLAKE and SHA3 on the hash engine. The PKE, ALU, hash engine and MDMA finish in no time, and both CPU caches always hit. Registers that are not modelled act as storage and are reported once in the log.

## Timing

Time is counted in CPU cycles at ACLK (350 MHz), so a run is deterministic and independent of host speed. The BIO runs at FCLK, which follows the divider `bio_init` writes. Idle loops are fast forwarded to the next timer tick or peripheral event; `--no-fast-forward` turns this off and `tests/ff_compare.py` checks both give the same output and final cycle count.

## Tests

`bao_emu test` builds every SDK example and every program in `tests/programs`, then runs `tests/run_examples.py` on them. 80 are checked against lines of their expected output; the rest, mostly book chapter examples, must run without tripping the guard or an exception. SDK examples whose names start with `zz_` are board only and are skipped.

`tests/programs` holds baoemu's own test programs: register readback, CPU timing, scalar AES, the MMU, the mailbox, MDMA, BIO, the crypto vectors and the panel. `bao_emu` copies one into the SDK's `examples` for the length of its build and removes it again, so the SDK is left as it was. `tests/ff_compare.py` checks every program gives the same output with fast forward on and off, `tests/sign_ref.py` checks the ECDSA and Ed25519 vectors on the host, and `tests/bioasm.py` assembles BIO programs into C arrays.

`tests/fuzz_mmio.c` throws random register accesses, instruction words and host events at the emulator, alone or between the instructions of a loaded program. Build it with the address and undefined behaviour sanitizers; the command is at the top of the file.

## Files

```
src/baoemu.h        public API
src/core.c          instance, run loop, ELF, UF2 and BIN loaders
src/cpu.c           RV32IMAC interpreter, privilege, MMU, timing
src/cpu_loop.h      interpreter loop
src/bus.c           address map, guard, CGU, IRQ arrays, timers, IOX
src/udma.c          UART, SPI, I2C, ADC
src/sce.c           SCE register blocks, hash engine, AES, TRNG
src/pke.c           PKE, big number ALU, SDMA internal channel
src/pwm.c           PWM slices
src/mdma.c          PL230 MDMA and its request block
src/mbox.c          mailbox
src/usb.c           USB device controller and host
src/bio.c           BIO cores
src/devices.c       external parts
src/main.c          command line tool
bao_emu.py          build and run a program (bao_emu.sh, bao_emu.bat)
python/baoemu.py    ctypes bindings
python/bao_view.py  viewer window
tests/              runner, fast forward check, fuzzer, references
tests/programs/     baoemu's test programs
```

## License

Apache-2.0. Copyright (c) 2026 Armstrong Subero.
