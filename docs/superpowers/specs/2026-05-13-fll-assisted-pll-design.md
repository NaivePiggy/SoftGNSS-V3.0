# FLL-Assisted PLL Carrier Tracking

**Date**: 2026-05-13
**Scope**: Add Frequency Lock Loop (FLL) assistance to the existing PLL carrier tracking loop in scalar tracking.

## Motivation

The current carrier tracking loop uses pure PLL (Costas loop with atan discriminator). `carrFreqBasis` is set once from acquisition results and never updated. Under high dynamics or long tracking sessions, frequency drift can cause the PLL to lose lock. FLL assistance provides a frequency pull-in mechanism that dynamically updates `carrFreqBasis`, improving robustness.

FLL parameters (`fllNoiseBandwidth = 10 Hz`, `fllDampingRatio = 0.7`) already exist in `initSettings.m` but are unused.

## Architecture

FLL operates as an outer frequency loop, feeding a dynamic `carrFreqBasis` into the inner PLL:

```
acquisition freq ──→ carrFreqBasis ──→ + ──→ carrFreq (to NCO)
                         ↑               ↑
                     FLL filter      PLL filter
                         ↑               ↑
                  cross-product      atan(Q_P/I_P)
                  I_P,Q_P + prev
```

- **FLL** (outer loop): cross-product frequency discriminator → 2nd-order filter → updates `carrFreqBasis`
- **PLL** (inner loop, unchanged): atan phase discriminator → 2nd-order filter → adds fine correction to `carrFreq`

## Changes

### `tracking.m`

**New filter state variables** (per channel):
- `oldFllNco`, `oldFllError` — FLL filter memory
- `prevI_P`, `prevQ_P` — previous-epoch I_P/Q_P for cross-product discriminator

**New loop coefficients**:
- `[tau1fll, tau2fll] = calcLoopCoef(settings.fllNoiseBandwidth, settings.fllDampingRatio, 1.0)`

**FLL processing** (runs every ms between correlation and PLL):

1. Skip first iteration (no previous I/Q yet)
2. Two-quadrant, amplitude-normalized cross/dot discriminator:
   ```
   cross = prevI_P * Q_P - prevQ_P * I_P
   dot = prevI_P * I_P + prevQ_P * Q_P
   fllError = atan(cross / dot) / (2 * pi)
   ```
   `fllError` is phase change over one coherent integration interval in
   cycles, not Hz. Keeping it in cycles matches the existing loop filter
   structure, whose output is the Hz correction.
3. FLL filter (same structure as DLL/PLL):
   ```
   fllNco = oldFllNco + (tau2fll/tau1fll) * (fllError - oldFllError) + fllError * (PDIcarr/tau1fll)
   ```
4. Update `carrFreqBasis = channel(channelNr).acquiredFreq + fllNco`

**New trackResults fields**:
- `trackResults(channelNr).fllDiscr` (1×N)
- `trackResults(channelNr).fllDiscrFilt` (1×N)

### `initSettings.m`

No changes needed. Existing `fllDampingRatio` and `fllNoiseBandwidth` are sufficient.

### `plotTracking.m`

Optional: overlay FLL discriminator on the PLL discriminator subplot (Row 3). Two traces on same axes, distinguishable by color.

### `trackingv.m`

Not modified. FLL assistance only applies to scalar tracking in this scope.

## Key Parameters

| Parameter | Value | Source |
|-----------|-------|--------|
| `fllNoiseBandwidth` | 10 Hz | `initSettings.m` (existing) |
| `fllDampingRatio` | 0.7 | `initSettings.m` (existing) |
| `PDIcarr` | 0.001 s | Same as PLL integration time |
| FLL loop gain `k` | 1.0 | Standard for frequency-locked loops |

## Non-Goals

- No switch to enable/disable FLL — always active
- No lock detector for FLL→PLL handoff
- No vector tracking FLL
- No FLL-specific C/N₀ estimation
