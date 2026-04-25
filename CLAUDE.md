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
1. Cleans up the environment and adds required paths
2. Loads settings from `initSettings.m`
3. Probes and plots raw IF data using `probeData.m`
4. Prompts user to enter "1" to start GNSS processing via `postProcessing.m`

### No Traditional Build System
This is a pure MATLAB project with **no compilation or build steps**. Code runs directly in MATLAB/Octave environment.

### Testing
- No automated test suite exists
- Test with provided sample data files (download links in README.md):
  - `gnss0.bin` (60.5 seconds of IF data)
  - `gnsa14.bin` (40 seconds of IF data)
- Testing process: Run `init.m` and enter "1" to initiate GNSS processing after data probing
- Interactive workflow requires user input

### Dependencies
- MATLAB or GNU Octave
- Paths to `include/` and `geoFunctions/` are added automatically in `init.m`

### Setup Gotcha
`initSettings.m` contains hardcoded file paths (e.g. `C:\Users\AltBOC\...`). Change `settings.fileName` to point to your actual data file location before running.

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
Vector tracking first runs the full scalar chain (with `skipAcquisition=1` and `VLLen=1` in settings), loads saved results, then invokes `trackingv.m` which couples tracking and navigation in a single loop.

**Detailed Signal Chain:**
1. **Initialization** (`init.m`): Environment setup and configuration loading
2. **Data Probing** (`probeData.m`): Visualization of raw IF data
3. **Acquisition** (`acquisition.m`): 2D search (frequency × code phase) for visible satellites
4. **Tracking** (`tracking.m`): DLL (Delay Lock Loop) and PLL (Phase Lock Loop) for continuous signal tracking
5. **Navigation Solution** (`postNavigation.m`): Navigation message decoding, pseudorange calculation, and position solving
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
- **Utilities**: `makeCaTable.m`, `parityCheck.m`, `navPartyChk.m`, `CNoVSM.m`, `findTransTime.m`, `invert.m`, `twosComp2dec.m`, `checkPhase.m`
- **Initialization**: `preRun.m` (channel state initialization)
- **Visualization**: `showChannelStatus.m`, `skyPlot.m`
- **C/No estimation**: `CNoVSM.m` (VSM), `CNoPRM.m` (PRM), `CNoMOM.m` (MOM) — three C/N₀ estimation methods. VSM uses variance statistics; PRM uses wideband/narrowband power ratio; MOM uses 2nd/4th envelope moments. Enabled individually via `settings.CNo.enableVSM`, `.enablePRM`, `.enableMOM`.

#### Geodetic Functions Module (`geoFunctions/`)
- **Position solving**: `leastSquarePos.m` (least squares positioning)
- **Coordinate transformations**: `cart2geo.m`, `geo2cart.m`, `cart2utm.m`
- **Satellite calculations**: `satpos.m` (satellite position computation)
- **Atmospheric corrections**: `tropo.m` (tropospheric delay)

#### Visualization Module
- `plotAcquisition.m` - Acquisition results visualization
- `plotTracking.m` - Tracking results visualization
- `plotNavigation.m` - Navigation solutions visualization
- `probeData.m` - Raw data plotting

#### Configuration Management
- `initSettings.m` - Default configuration initialization (function)
- `setSettings.m` / `setSettings.fig` - GUI for interactive configuration

### Key Data Structures

#### `settings` Structure (Configuration Parameters)
Central configuration object passed to all processing functions. Key fields:
- `settings.msToProcess` - Processing duration in milliseconds
- `settings.fileName` - Path to IF data file
- `settings.samplingFreq`, `settings.IF` - Sampling and intermediate frequencies
- `settings.acqSatelliteList` - List of PRNs to acquire
- `settings.dllNoiseBandwidth`, `settings.pllNoiseBandwidth` - Tracking loop parameters
- `settings.navSolRate` - Navigation solution update rate

#### `acqResults` Structure (Acquisition Results)
- `acqResults.carrFreq` - Carrier frequencies for each PRN (1×32 array)
- `acqResults.codePhase` - Code phases for each PRN (1×32 array)
- `acqResults.peakMetric` - Peak correlation metrics (1×32 array)

#### `trackResults` Structure (Tracking Results)
Per-channel tracking data with fields like:
- `trackResults.status` - Channel status ('-': closed, 'T': tracking)
- `trackResults.I_P`, `trackResults.Q_P` - In-phase and quadrature prompt correlator outputs
- `trackResults.dllDiscr`, `trackResults.pllDiscr` - DLL and PLL discriminator outputs

#### `channel` Structure (Channel State)
- `channel.PRN` - Satellite PRN number
- `channel.acquiredFreq` - Acquired carrier frequency
- `channel.codePhase` - Code phase
- `channel.status` - Current status

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
```

When `settings.VLLen = 1`, after the above chain completes, `trackingv.m` runs a coupled tracking+navigation loop using `navSolutions` and `eph` as feedback.

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
- `settings.plotTracking` controls tracking results visualization
- The default IF is 38.4 kHz (low-IF from GN3S sampler); sampling rate is 16.3676/2 MHz
- `settings.msToProcess` should be >= 36000 to ensure enough navigation subframes

## Key File Reference

### Essential Entry Points
- `init.m` - Main entry point script
- `initSettings.m` - Configuration initialization function
- `postProcessing.m` - Main processing script coordinating acquisition, tracking, and navigation

### Core Processing Functions
- `acquisition.m` - Satellite signal acquisition (2D search)
- `tracking.m` - Scalar signal tracking with DLL/PLL
- `trackingv.m` - Vector tracking (coupled tracking+navigation loop)
- `postNavigation.m` - Navigation solution computation
- `findPreambles.m` - Detect TLM preamble in navigation data stream
- `calculatePseudoranges.m` - Compute raw pseudoranges from transmit/receive times

### Data Files
- `gnss0.bin` - 60.5 seconds of IF data (8-bit I/Q samples)
- `gnsa14.bin` - 40 seconds of IF data (8-bit I/Q samples)
- Download links available in `README.md`

### Documentation
- `README.md` - Project overview and data file information
- `AGENTS.md` - Detailed AI agent guidelines including code style and testing procedures
- `license.txt` - GNU GPL v2 license terms

## Code Style Reference

See `AGENTS.md` for detailed code style guidelines (function documentation patterns, variable naming, struct usage, error handling, MATLAB-specific patterns).

Key conventions at a glance:
- **4-space indentation**, **camelCase** variables, **UPPERCASE** constants
- Every function header documents inputs/outputs with types and descriptions
- Use `%%` for major code sections, `%` for inline comments