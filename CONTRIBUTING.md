# Contributing to QRNix Firmware

Thanks for wanting to patch the firmware. This project is small and
hobbyist-run, so the rules are light — but the license has one hard
requirement, and the hardware has a few practical ones.

## Before you start

- Build and flash the stock firmware first (see [README](README.md#toolchain))
  so you know your toolchain works before you change anything.
- Open an issue for anything non-trivial, or say what you are doing in the PR
  description. There is no issue template; a paragraph is enough.

## License rule (non-negotiable)

The repo is **LGPL 2.1-or-later**:

- Your contributions to files you write are licensed LGPL 2.1-or-later by
  submitting them. That is the point of the license — keep it open.
- **`src/shared/`, `src/processors/`, `src/interfaces/`, `include/` are the
  vendored libspecbleach core.** Treat them as upstream: if you must modify
  them, LGPL §2(a) requires a prominent notice stating what you changed and
  when. Prefer patching in `src/qrnix.cpp` instead — it is your layer.
- Do not delete or alter the existing license headers.

## Practical rules

- Target the `teensy40` environment only; do not add other boards unless the
  change genuinely needs them.
- Keep changes buildable: `pio run -e teensy40` must succeed before you push.
- If you change pins, modes, the serial status line, or the OLED layout,
  update the README's pin map / code map / serial section in the same PR.
- No reformatting of code you did not touch — keep diffs reviewable.
- Test what you changed: build, flash, and verify over the serial monitor.
  State in the PR what you tested and what you did not.

## Commit messages

Changelogs are generated from the commit log (see
[docs/release-process.md](docs/release-process.md)), so subjects start with
a type prefix:

| Type | Use for |
|---|---|
| `feat:` | new feature |
| `fix:` | bug fix |
| `docs:` | documentation |
| `refactor:` | restructuring, no behavior change |
| `build:` | build system, tooling, release process |
| `chore:` | maintenance |
| `perf:` / `style:` / `test:` | performance, formatting, tests |

A scope is optional (`feat(ui): ...`); a `!` marks a breaking change
(`feat!: ...`).

The `.githooks/commit-msg` hook enforces this. Install it once:

```bash
git config core.hooksPath .githooks
```

The release driver's `Prepare release vX.Y.Z` commits and git's own
merge/revert commits are exempt.

## What makes a good PR

- One logical change per PR.
- A description that says *what* changed and *why* — the *how* is in the diff.
- The build output (`pio run`) success line, if you can paste it.
- Notes on anything you observed on real hardware (timing, audio behavior,
  display quirks) — this project lives and dies by hardware observations.

## Getting help

Open an issue with `[question]` in the title, or ask in the PR itself.
