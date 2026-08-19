# QRNix Firmware

Standalone DSP noise reduction for HF radio, running on a Teensy 4.0 with a
Teensy Audio Shield Rev D/D2. It sits between a radio's line output and an
amplified speaker or line-level input and offers three modes: **Bypass**
(clean pass-through), **Spectral** (NR1, noise-profile based), and
**Adaptive** (NR2, continuously adapting).

```text
Radio line out -> Audio Shield ADC -> STFT -> NR core -> ISTFT
               -> Audio Shield DAC -> line out -> speaker/amplifier
```

This README is for people who want to build, flash, or modify the firmware.
For what the product does and how the DSP works, see the QRNix website.

## Target hardware and pin map

The firmware targets the QRNix board (`qrnix-hardware`), which carries the
Teensy 4.0 + Audio Shield module, the OLED, the four trim pots, and the mode
switch onboard. Pins are fixed in `src/qrnix.cpp` if you wire your own
hardware:

| Pin | Signal | Note |
|---|---|---|
| A0 | Reduction pot | 10 kΩ linear, outer pins to 3.3V/GND, wiper to A0 |
| A1 | Smoothing pot | same wiring |
| A2 | Whitening pot | same wiring |
| A3 | Aggression pot | same wiring |
| D3 | Mode switch — Adaptive | `LOW` selects; center-off = bypass |
| D4 | Mode switch — Spectral | `LOW` selects; center-off = bypass |
| D2 | Button | active LOW, internal pullup |
| SDA/SCL | SSD1306 OLED | I²C address `0x3C` (optional device) |

The OLED is optional: if it does not respond at boot, audio processing
continues without it.

## Toolchain

PlatformIO with the Teensy platform. Install and build:

```bash
uv tool install platformio
pio run -e teensy40
```

The firmware uploads through the Teensy HalfKay bootloader. `pio device list`
will *not* show the Teensy — the bootloader is a USB HID device, not a serial
port. The reliable CLI upload:

```bash
~/.platformio/packages/tool-teensy/teensy_loader_cli \
  --mcu=imxrt1062 -w -v .pio/build/teensy40/firmware.hex
```

Press the Teensy Program button once when the loader says it is waiting; a
successful upload ends with `Programming...` then `Booting`.

Serial monitor (115200 baud):

```bash
pio device monitor -p /dev/ttyACM0 -b 115200
```

## Repository layout

| Path | What it is |
|---|---|
| `src/qrnix.cpp` | The sketch: UI, modes, controls, wiring of the DSP pipeline. This is *your* code to patch. |
| `src/processors/` | The two DSP processor implementations (spectral + adaptive). |
| `src/shared/` | Vendored libspecbleach DSP core (42 files) — upstream code, see License. |
| `src/interfaces/` | Processor interface definition. |
| `include/` | Public API headers for the DSP core. |
| `platformio.ini` | Build environment (`teensy40`), include paths, library deps. |

## Code map

All user-facing logic lives in `src/qrnix.cpp` (~800 lines, single file):

| Want to change… | Look at |
|---|---|
| Boot sequence, hardware init | `setup()` |
| Main loop: control reads, serial status | `loop()` |
| Default/reference parameter values | `set_default_params()`, `apply_params()` |
| Mode switching (free/recreate of DSP cores) | `activate_mode()`, `read_mode_switch()` |
| Button behavior (tap = feature circle, hold = noise capture) | `handle_button_tap()`, `handle_button_hold()`, `advance_feature_circle()` |
| Noise-profile capture | `start_noise_capture()`, `abort_noise_capture()` |
| Tone-kill / post-filter sync in bypass | `sync_tk_bypass_processor()` |
| OLED screens | `update_boot_splash()`, `update_display()` |
| Version string (shown at boot) | `SOFTWARE_VERSION` (`qrnix.cpp:85`, currently `0.3.5`) |
| Pin assignments, OLED address | `#define`s at the top of `qrnix.cpp` |

## Engineering constraints (read before patching)

- **Heap:** the two DSP processors do not fit in the Teensy 4.0 heap
  simultaneously. A mode switch frees the old processor and creates the
  selected one.
- **Frame size API:** the libspecbleach frame-size API takes milliseconds —
  the code passes `25.0f`, not `0.025f`.
- **FFT:** CMSIS-DSP requires a supported power-of-two FFT; both STFT
  configurations use `NEXT_POWER_OF_TWO`.
- **Allocation failure:** if a processor cannot be allocated, the firmware
  falls back to bypass instead of dereferencing null.
- **Mode debounce:** mode changes are debounced for 50 ms and applied with
  audio interrupts stopped.
- **OLED absence:** processing continues if the display never initializes.
- **Boot serial:** USB serial waits at most three seconds during startup, so
  standalone operation is not held up by an absent monitor.

## Serial and debugging

`setup()` reports its stages (`setup: USB serial ready`, `setup: codec ready`,
`setup: display ready`, `setup: complete`) so you can see how far boot got.

Once per second, `loop()` prints a status line:

```text
m=2 src=L red=12.0 sm=55.0 wh=30.0 ag=1.20 tk=0 pp=0 clip=0 blk_l=0 blk_r=0 in_l=1234 in_r=1200 lvl_l=87.0 lvl_r=12.0 out_l=1230 out_r=0 bad=0
```

Fields: `m` mode (0 bypass / 1 spectral / 2 adaptive), `src` selected input
(L/R), `red/sm/wh/ag` control values, `tk`/`pp` feature flags, `clip`
overload latch, `blk`/`in`/`lvl` input statistics, `out` output levels,
`bad` error counter. With diagnostics enabled, a second line reports
`snr`/`bands`/`gain`/`mix`.

A `CrashReport` printed once after a successful boot describes the previous
crash retained by the Teensy; repeated healthy status lines after
`setup: complete` mean the current run is fine.

## Configuration constants

The knobs are `#define`s and `constexpr`s at the top of `src/qrnix.cpp`:
pins, `OLED_ADDR`, `SOFTWARE_VERSION`, frame/FFT sizes, and the tone-kill /
post-filter compile gates.

## License

LGPL 2.1-or-later (see [LICENSE](LICENSE)). The DSP core in `src/shared/`
and `src/processors/` is derived from libspecbleach (Copyright 2022 Luciano
Dato); modifications to those files must carry prominent notices per LGPL
§2(a) — see [CONTRIBUTING](CONTRIBUTING.md).

## Contributing

Found a bug or want to add a feature? See [CONTRIBUTING](CONTRIBUTING.md).

## Related repositories

- [qrnix-hardware](https://github.com/qrnix-labs/qrnix-hardware) — the board (schematic, layout, gerbers)
- [qrnix-enclosure](https://github.com/qrnix-labs/qrnix-enclosure) — the box
- [brand](https://github.com/qrnix-labs/brand) — logos and style guide
