# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SoftGNSS v3.0 is an open-source software-defined GNSS (GPS/Galileo) receiver implemented in MATLAB. It processes raw intermediate frequency (IF) samples from GNSS front-ends to perform satellite acquisition, signal tracking, and navigation solution computation. The codebase was adapted from the textbook "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach" by Borre, Akos, et.al. and is used for academic purposes in the "AV423 Satellite Navigation" course at Shanghai Jiao Tong University.

**License**: GNU General Public License v2 (see `license.txt`)

## Common Development Tasks

### Running the Software
```matlab
% In MATLAB/Octave console:
init  % Main entry point - initializes settings and starts processing
```

The `init.m` script:
1. Cleans up the environment and adds required paths (`include/`, `geoFunctions/`)
2. Loads settings from `initSettings.m`
3. Probes and plots raw IF data using `probeData.m`
4. Automatically starts GNSS processing via `postProcessing.m` (hardcoded `gnssStart = 1`)

### No Traditional Build System
This is a pure MATLAB project with **no compilation or build steps**. Code runs directly in MATLAB/Octave.

### Testing
- No automated test suite exists
- Test with provided sample data files (download links in README.md):
  - `gnss0.bin` (60.5 seconds of IF data)
  - `gnsa14.bin` (40 seconds of IF data)
- Testing process: Run `init.m` — processing starts automatically

### Dependencies
- MATLAB or GNU Octave
- MATLAB's Mapping Toolbox (for `skyplot`/`skyPlot`)

### Setup Gotcha
`initSettings.m` contains hardcoded file paths (e.g. `data\gnss0.bin`). Change `settings.fileName` to point to your actual data file location before running. Data files tracked by `.gitignore` — place sample files under `data/`.

## Code Architecture and Structure

### Core Processing Flow

The receiver operates in two tracking modes controlled by `settings.VLLen`:

**Scalar Tracking** (`settings.VLLen = 0`, default):
```
init.m → postProcessing.m → acquisition.m → tracking.m → postNavigation.m
```

**Vector Tracking** (`settings.VLLen = 1`):
```
init.m → postProcessing.m → acquisition.m → tracking.m → postNavigation.m → trackingv.m
```
Vector tracking first runs the full scalar chain (with `skipAcquisition=1` and `VLLen=1` in settings), loads saved results, then invokes `trackingv.m` which couples tracking and navigation in a single Kalman-filtered loop.

**Detailed Signal Chain:**
1. **Initialization** (`init.m`): Environment setup and configuration loading
2. **Data Probing** (`probeData.m`): Visualization of raw IF data
3. **Acquisition** (`acquisition.m`): 2D search (frequency × code phase) for visible satellites
4. **Tracking** (`tracking.m`): DLL/PLL for continuous signal tracking. Also handles:
   - Per-channel C/N₀ estimation (VSM, PRM, MOM)
   - Histogram-based data bit synchronization (`histBitSync.m`)
   - Bit-boundary-aligned data-bit stripping for PRM C/N₀
5. **Navigation Solution** (`postNavigation.m`): Navigation message decoding, pseudorange calculation, and position solving. Optionally performs additional data-bit stripping for PRM C/N₀ correction.
6. **Vector Tracking** (`trackingv.m`): Coupled tracking and navigation using navigation solution feedback (only when `VLLen = 1`)

### I/Q Data Format Handling

`postProcessing.m` handles two data formats via `settings.fileType`:
- **fileType 1** (8-bit real): Samples read directly, `dataAdaptCoeff = 1`
- **fileType 2** (8-bit I/Q): Samples interleaved I0,Q0,I1,Q1,... — code splits into even/odd indices and assembles complex data:
  ```matlab
  data1 = data(1:2:end);
  data2 = data(2:2:end);
  data = data1 + 1i .* data2;
  ```
  `dataAdaptCoeff = 2` doubles the byte read amount to account for I+Q pairs.

### Module Organization

#### Signal Processing Module (`include/`)
- **Core algorithms**: `generateCAcode.m`, `ephemeris.m`, `calcLoopCoef.m`
- **C/No estimation**: `CNoVSM.m` (Variance Summing Method), `CNoPRM.m` (Power Ratio Method), `CNoMOM.m` (Moments Method). All three are independent estimators taking `(I, Q, T)` inputs. `CNoPRM.m` additionally accepts an optional 5th `dataBits` parameter (±1 vector) for navigation data bit stripping to improve accuracy. Each enabled individually via `settings.CNo.enableVSM`, `.enablePRM`, `.enableMOM`.
- **Bit synchronization**: `histBitSync.m` — finds the 20ms data bit boundary in GPS L1 C/A tracking by maximizing energy of coherent 20ms sums across 20 candidate offsets. Returns `tau` (boundary offset 0..19) and `confidence` (ratio >2.0 = reliable). `decodeBitSigns.m` — given a bit boundary, decodes ±1 data bit signs from an I_P window for C/No bit stripping. Used both in `tracking.m` (real-time) and `postNavigation.m` (post-processing correction).
- **Utilities**: `makeCaTable.m`, `parityCheck.m`, `navPartyChk.m`, `findTransTime.m`, `invert.m`, `twosComp2dec.m`, `checkPhase.m`
- **Initialization**: `preRun.m` (channel state initialization from acquisition results)
- **Visualization**: `showChannelStatus.m`, `skyPlot.m`

#### Geodetic Functions Module (`geoFunctions/`)
- **Position solving**: `leastSquarePos.m` (least squares positioning)
- **Coordinate transformations**: `cart2geo.m`, `geo2cart.m`, `cart2utm.m`, `togeod.m`, `topocent.m`
- **Satellite calculations**: `satpos.m` (satellite position computation)
- **Atmospheric corrections**: `tropo.m` (tropospheric delay)
- **Utilities**: `deg2dms.m`, `dms2mat.m`, `mat2dms.m`, `findUtmZone.m`, `roundn.m`, `e_r_corr.m`, `clksin.m`, `clsin.m`, `check_t.m`, `R.m`, `R_BL.m`

#### Visualization Module
- `plotAcquisition.m` - Acquisition results bar chart
- `plotTracking.m` - Per-channel tracking results: 4-row layout with scatter plot, nav bits, PLL/DLL discriminators (raw + filtered), correlation envelopes, and a full-width C/N₀ subplot (Row 4) showing VSM/PRM/MOM estimates with legend
- `plotNavigation.m` - Navigation solutions: UTM coordinate variations, 3D position plot, and satellite sky plot (via `uipanel` + `skyplot`)
- `probeData.m` - Raw data histogram and time-domain plot

#### Configuration Management
- `initSettings.m` - Default configuration initialization (function)
- `setSettings.m` / `setSettings.fig` - GUI for interactive configuration

### Bit Synchronization and Data-Bit Stripping

The receiver uses a histogram-based method to find GPS L1 C/A data bit boundaries (20ms bits). This enables data-bit stripping for improved PRM C/N₀ estimation.

**In tracking.m** (per-channel, real-time):
1. After `bitSyncMinData` ms (default 1200), `histBitSync.m` finds the bit boundary `tau`
2. If confidence > 1.5, `trackResults(channelNr).bitBoundary` is set
3. On each PRM C/N₀ update, the code aligns the integration window to bit boundaries, determines bit signs per complete 20ms block, and strips the signs from I/Q before calling `CNoPRM`

**In postNavigation.m** (post-processing correction):
1. For each active channel, `histBitSync.m` is re-run on the full I_P record
2. `decodeBitSigns.m` generates ±1 data bit signs aligned to the boundary
3. PRM C/N₀ is recomputed with the full-record bit-stripped I/Q, producing more accurate estimates than the real-time version

### Key Data Structures

#### `settings` Structure (Configuration Parameters)
Central configuration object passed to all processing functions. Key fields:
- `settings.msToProcess` - Processing duration in milliseconds (default 36000)
- `settings.fileName`, `settings.fileType`, `settings.dataType` - Input file configuration
- `settings.samplingFreq`, `settings.IF` - Sampling and intermediate frequencies. Default: IF=38.4kHz, fs=8.1838MHz (GN3S sampler)
- `settings.acqSatelliteList` - List of PRNs to acquire (default 1:32)
- `settings.acqThreshold` - Acquisition detection threshold (default 2.5)
- `settings.dllNoiseBandwidth`, `settings.pllNoiseBandwidth` - Tracking loop bandwidths
- `settings.fllDampingRatio`, `settings.fllNoiseBandwidth` - FLL parameters (configured but not actively used in current scalar tracking)
- `settings.navSolRate` - Navigation solution update rate (default 10 Hz)
- `settings.VLLen` - 0 = scalar tracking, 1 = vector tracking
- `settings.skipAcquisition` - Skip acquisition and load saved results
- `settings.enableFastTracking` - Fast tracking mode (0/1)
- **C/No settings** (`settings.CNo.*`):
  - `enableVSM`, `enablePRM`, `enableMOM` - Toggle each estimator
  - `VSMinterval`, `PRM_K`, `PRM_M`, `MOMinterval` - Accumulation parameters
  - `enableBitSync`, `bitSyncMinData`, `bitSyncBlockSize`, `bitSyncMinBlocks`, `bitSyncReRun` - Bit synchronization parameters

#### `trackResults` Structure (Tracking Results, per channel)
- `trackResults(ch).status` - '-' (no lock), 'T' (tracking)
- `trackResults(ch).PRN` - Satellite PRN number
- `trackResults(ch).I_P`, `.Q_P`, `.I_E`, `.Q_E`, `.I_L`, `.Q_L` - Correlator outputs (1×N arrays)
- `trackResults(ch).carrFreq`, `.codeFreq` - Tracked frequencies
- `trackResults(ch).dllDiscr`, `.pllDiscr`, `.dllDiscrFilt`, `.pllDiscrFilt` - Discriminator outputs
- `trackResults(ch).absoluteSample` - Absolute sample index at each ms
- `trackResults(ch).CNo.VSMValue/Index`, `.PRMValue/Index`, `.MOMValue/Index` - C/N₀ estimates at their respective intervals
- `trackResults(ch).bitBoundary` - Detected bit boundary offset (0..19), or -1 if unknown/failed

#### `channel` Structure (Channel State)
- `channel.PRN`, `.acquiredFreq`, `.codePhase`, `.status`

### Data Flow
```
Raw IF data file (settings.fileName)
    ↓
probeData(settings) - Data visualization
    ↓
acquisition(data, settings) → acqResults
    ↓
preRun(acqResults, settings) → channel
    ↓
tracking(fid, channel, settings) → trackResults
    │  └─ histBitSync(I_P) → bitBoundary (per channel, after ~1200ms)
    │  └─ CNoVSM/CNoPRM/CNoMOM at configured intervals
    │     └─ CNoPRM with bit-stripped I/Q when bitBoundary known
    ↓
findPreambles(trackResults, settings) → subFrameStart, activeChnList
    ↓
ephemeris(navBits, ...) → eph (per satellite)
    ↓
findTransTime(...) → transmitTime
calculatePseudoranges(transmitTime, rxTime, ...) → raw pseudoranges
    ↓
satpos(transmitTime, PRN, eph) → satellite positions
    ↓
leastSquarePos(satPositions, pseudoranges, ...) → xyzdt (position + clock)
    ↓
cart2geo / cart2utm → geographic/UTM coordinates
    ↓
navSolutions structure (ECEF/UTM/geographic coordinates)
    ↓
[optional] histBitSync + decodeBitSigns → corrected PRM C/N₀ in trackResults
```

When `settings.VLLen = 1`, after the above chain completes, `trackingv.m` runs a coupled tracking+navigation loop using `navSolutions` and `eph` as feedback via a Kalman filter.

## Configuration Management

### Modifying Settings
1. **Edit `initSettings.m` directly** - For permanent configuration changes
2. **Use GUI** - Run `setSettings` to open configuration interface
3. **Command line** - Modify `settings` structure in MATLAB workspace for temporary changes

### Important Configuration Notes
- Data file path in `settings.fileName` must point to valid IF data
- `settings.fileType` must match data format (1: 8-bit real, 2: 8-bit I/Q)
- `settings.skipAcquisition` can be set to 1 to skip acquisition phase and load `acqresults.mat`
- `settings.VLLen` switches between scalar tracking (0) and vector tracking (1)
- `settings.msToProcess` should be >= 36000 to ensure enough navigation subframes
- The default IF is 38.4 kHz (low-IF from GN3S sampler); sampling rate is 8.1838 MHz (16.3676/2)
- C/N₀ estimators each operate at their own interval defined in `settings.CNo.*`; VSM at 400ms, PRM at K=200ms, MOM at 200ms
- Bit synchronization needs ≥1200ms of tracking data before first attempt; PRM accuracy improves significantly when bit boundary is known
- `.mat` files are gitignored; intermediate results (`trackingResults.mat`, `navSolutions.mat`, `acqresults.mat`) are saved/loaded automatically

## Key File Reference

### Essential Entry Points
- `init.m` - Main entry point script (hardcoded auto-start)
- `initSettings.m` - Configuration initialization function
- `postProcessing.m` - Main processing script coordinating acquisition, tracking, and navigation

### Core Processing Functions
- `acquisition.m` - Satellite signal acquisition (2D search)
- `tracking.m` - Scalar signal tracking with DLL/PLL and C/N₀ estimation
- `trackingv.m` - Vector tracking (coupled tracking+navigation loop)
- `postNavigation.m` - Navigation solution computation
- `findPreambles.m` - Detect TLM preamble in navigation data stream
- `calculatePseudoranges.m` - Compute raw pseudoranges from transmit/receive times

### C/N₀ Estimation and Bit Sync
- `CNoVSM.m` - Variance Summing Method
- `CNoPRM.m` - Power Ratio Method with optional data-bit stripping
- `CNoMOM.m` - Moments Method (2nd/4th envelope moments)
- `histBitSync.m` - Histogram-based 20ms bit boundary detection
- `decodeBitSigns.m` - Decode ±1 bit signs from I_P using known boundary

### Data Files
- `gnss0.bin` - 60.5 seconds of IF data (8-bit I/Q samples)
- `gnsa14.bin` - 40 seconds of IF data (8-bit I/Q samples)
- Download links available in `README.md`

### Documentation
- `README.md` - Project overview and data file information
- `license.txt` - GNU GPL v2 license terms

## Code Style Reference

Key conventions:
- **4-space indentation**, **camelCase** variables, **UPPERCASE** constants
- Every function header documents inputs/outputs with types and descriptions in a standard comment block
- Use `%%` for major code sections, `%` for inline comments
- GPL v2 license header on all source files
- CVS `$Id$` records in original files are preserved for provenance
