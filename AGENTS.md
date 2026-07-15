# AGENTS.md

SoftGNSS v3.0 — MATLAB software-defined GNSS receiver (GPS/Galileo). Academic project for "AV423 Satellite Navigation" at SJTU.

## Quickstart

```matlab
init  % cleans env, adds include/ & geoFunctions/ to path, loads settings, runs processing
```

## Getting code to run

- Change `settings.fileName` in `initSettings.m` to your data file path.
- `settings.fileType`: 1 = 8-bit real, 2 = 8-bit interleaved I/Q (I0,Q0,I1,Q1,…). Default is 2 (GN3S sampler).
- `settings.msToProcess` should be >= 36000 to get enough nav subframes.
- `init.m:79` hardcodes `gnssStart = 1` — processing starts automatically, no user prompt.
- Data files go under `data/` (gitignored). Sample files: `gnss0.bin` (60.5s), `gnsa14.bin` (40s).

## No build, no tests

Pure MATLAB — run directly. No compile step, no automated test suite. To verify changes: run `init` with a sample data file.

## Architecture

Two modes controlled by `settings.VLLen`:
- **0 (scalar, default)**: `init → postProcessing → acquisition → tracking → postNavigation`
- **1 (vector)**: same chain, then `trackingv.m` for coupled Kalman-filter tracking+navigation loop

## Key directories

| Directory | Purpose |
|-----------|---------|
| `include/` | Signal processing: acquisition, tracking, C/N₀ estimation, bit sync, ephemeris |
| `geoFunctions/` | Geodesy: satellite position, coordinate transforms, least-squares position |

## Linting

```bash
python -m miss_hit.mh_style path/to/file.m
```
This runs automatically on `.m` file edits via hooks (configured in `.claude/settings.json` and `.codex/hooks.json`).

## Never edit these files

`.mat` files, `.bin` files, and anything under `data/` — blocked by pre-edit hooks. Intermediate results (`acqresults.mat`, `trackingResults.mat`, `navSolutions.mat`) are auto-saved/loaded and gitignored.

## MATLAB pitfalls specific to this codebase

- Always use `1i` for complex, never `i` (shadowed by loop variables everywhere in tracking loops).
- 4-space indentation, camelCase variables, UPPERCASE constants, `%%` section markers.
- Preallocate arrays before loops — the tracking loop runs for `settings.msToProcess` milliseconds (default 36000) and dynamic growth kills performance.
- `settings.skipAcquisition = 1` loads `acqresults.mat` instead of running acquisition (useful for re-running tracking/nav only).

## Configuration highlights

- **FLL**: Enabled by default (`enableFLL = 1`, `fllHandoffTime = 2000ms`). FLL assists PLL for the first 2000ms then hands off. Use `fllHandoffTime = 0` for continuous FLL assistance.
- **C/N₀**: Three independent estimators (VSM, PRM, MOM), each toggled by `settings.CNo.enable*`. PRM accuracy improves when bit boundary is known via `histBitSync.m` (requires ≥1200ms of tracking data).
- **Positioning**: Weighted least squares (`enableWeightedLS`), robust LS (`enableRobustLS`), elevation-weighted, with configurable Doppler convention.
- Default IF = 38.4 kHz, fs = 8.1838 MHz (GN3S v2 sampler).

## Skills

- `matlab-analyze`: Run MATLAB Code Analyzer on `.m` files for correctness/performance warnings.
- `matlab-reviewer`: Agent-based code review for MATLAB/GNSS-specific issues (vectorization, preallocation, `1i` vs `i`, etc.).
