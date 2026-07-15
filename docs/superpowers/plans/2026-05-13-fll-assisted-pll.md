# FLL-Assisted PLL Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add FLL (Frequency Lock Loop) frequency assistance to the existing PLL carrier tracking loop, using a cross-product discriminator with dynamic `carrFreqBasis` update.

**Architecture:** FLL runs as an outer frequency loop in `tracking.m`, feeding a dynamic `carrFreqBasis` into the inner PLL. Uses existing `settings.fllNoiseBandwidth` and `settings.fllDampingRatio` from `initSettings.m`. Cross-product discriminator on adjacent 1ms I_P/Q_P pairs feeds a 2nd-order loop filter. FLL discriminator outputs are recorded and optionally plotted.

**Tech Stack:** MATLAB

---

### Task 1: Initialize FLL trackResults fields

**Files:**
- Modify: `tracking.m:79-84`

- [ ] **Step 1: Add fllDiscr and fllDiscrFilt fields to trackResults template**

After the existing PLL discriminator lines (82-83), insert:

```matlab
trackResults.pllDiscr       = inf(1, settings.msToProcess);
trackResults.pllDiscrFilt   = inf(1, settings.msToProcess);
trackResults.fllDiscr       = inf(1, settings.msToProcess);
trackResults.fllDiscrFilt   = inf(1, settings.msToProcess);
```

The full block should read:

```matlab
% Loop discriminators
trackResults.dllDiscr       = inf(1, settings.msToProcess);
trackResults.dllDiscrFilt   = inf(1, settings.msToProcess);
trackResults.pllDiscr       = inf(1, settings.msToProcess);
trackResults.pllDiscrFilt   = inf(1, settings.msToProcess);
trackResults.fllDiscr       = inf(1, settings.msToProcess);
trackResults.fllDiscrFilt   = inf(1, settings.msToProcess);
```

- [ ] **Step 2: Commit**

```bash
git add tracking.m
git commit -m "feat: add FLL discriminator fields to trackResults template"
```

---

### Task 2: Add FLL coefficients and state variables

**Files:**
- Modify: `tracking.m:122-130` (after `calcLoopCoef` for PLL)
- Modify: `tracking.m:184-186` (after PLL state vars)

- [ ] **Step 1: Add FLL loop coefficient calculation**

After the PLL `calcLoopCoef` call (lines 127-129), insert FLL coefficient calculation:

```matlab
% Calculate filter coefficient values
[tau1carr, tau2carr] = calcLoopCoef(settings.pllNoiseBandwidth, ...
    settings.pllDampingRatio, ...
    0.25);

%--- FLL variables ---------------------------------------------------------
% Calculate FLL filter coefficient values
[tau1fll, tau2fll] = calcLoopCoef(settings.fllNoiseBandwidth, ...
    settings.fllDampingRatio, ...
    1.0);
```

- [ ] **Step 2: Add FLL state variables per channel**

After the PLL state variables (lines 185-186), insert:

```matlab
%carrier/Costas loop parameters
oldCarrNco   = 0.0;
oldCarrError = 0.0;

%FLL loop parameters
oldFllNco   = 0.0;
oldFllError = 0.0;
prevI_P     = 0.0;
prevQ_P     = 0.0;
```

- [ ] **Step 3: Commit**

```bash
git add tracking.m
git commit -m "feat: add FLL coefficients and state variables per channel"
```

---

### Task 3: Add FLL processing in main loop

**Files:**
- Modify: `tracking.m:323-338` (PLL section — insert FLL before PLL)

- [ ] **Step 1: Insert FLL discriminator and filter before PLL**

Replace the current PLL section (lines 323-338) with FLL + PLL:

```matlab
%% Find FLL error and update carrier frequency basis --------------------

% FLL discriminator, skip first iteration. The discriminator returns phase
% change in cycles over one integration interval; the loop filter converts
% this to a frequency correction.
if loopCnt > 1
    fllError = fllDiscriminator(prevI_P, prevQ_P, I_P, Q_P);

    % FLL filter
    fllNco = oldFllNco + (tau2fll/tau1fll) * ...
        (fllError - oldFllError) + fllError * (PDIcarr/tau1fll);
    oldFllNco   = fllNco;
    oldFllError = fllError;

    % Update carrier frequency basis
    carrFreqBasis = channel(channelNr).acquiredFreq + fllNco;
else
    fllError = 0;
    fllNco   = 0;
end

% Store previous I_P/Q_P for next FLL iteration
prevI_P = I_P;
prevQ_P = Q_P;

%% Find PLL error and update carrier NCO ----------------------------------

% Implement carrier loop discriminator (phase detector)
carrError = atan(Q_P / I_P) / (2.0 * pi);

% Implement carrier loop filter and generate NCO command
carrNco = oldCarrNco + (tau2carr/tau1carr) * ...
    (carrError - oldCarrError) + carrError * (PDIcarr/tau1carr);
oldCarrNco   = carrNco;
oldCarrError = carrError;

% Modify carrier freq based on NCO command
carrFreq = carrFreqBasis + carrNco;

trackResults(channelNr).carrFreq(loopCnt) = carrFreq;
trackResults(channelNr).remCarrPhase(loopCnt) = remCarrPhase;
```

- [ ] **Step 2: Commit**

```bash
git add tracking.m
git commit -m "feat: add FLL cross-product discriminator and filter before PLL"
```

---

### Task 4: Record FLL discriminator outputs

**Files:**
- Modify: `tracking.m:359-362` (record discriminators section)

- [ ] **Step 1: Add FLL discriminator recording**

After the PLL discriminator recording lines, add FLL recording:

```matlab
trackResults(channelNr).dllDiscr(loopCnt)       = codeError;
trackResults(channelNr).dllDiscrFilt(loopCnt)   = codeNco;
trackResults(channelNr).pllDiscr(loopCnt)       = carrError;
trackResults(channelNr).pllDiscrFilt(loopCnt)   = carrNco;
trackResults(channelNr).fllDiscr(loopCnt)       = fllError;
trackResults(channelNr).fllDiscrFilt(loopCnt)   = fllNco;
```

- [ ] **Step 2: Commit**

```bash
git add tracking.m
git commit -m "feat: record FLL discriminator and NCO in trackResults"
```

---

### Task 5: Add FLL traces to plotTracking

**Files:**
- Modify: `plotTracking.m:104-144` (PLL discriminator subplots)

- [ ] **Step 1: Add FLL discriminator overlay on PLL raw subplot**

Change the Raw PLL discriminator plot (handles 2,1) to also show FLL:

Replace lines 104-112:

```matlab
%----- PLL/FLL discriminator unfiltered --------------------------------
plot  (handles(2, 1), timeAxisInSeconds, ...
                      trackResults(channelNr).pllDiscr, 'r', ...
                      timeAxisInSeconds, ...
                      trackResults(channelNr).fllDiscr, 'g');      

grid  (handles(2, 1));
axis  (handles(2, 1), 'tight');
xlabel(handles(2, 1), 'Time (s)');
ylabel(handles(2, 1), 'Amplitude');
title (handles(2, 1), 'Raw PLL/FLL discriminator');
hLegend = legend(handles(2, 1), 'PLL', 'FLL');
```

- [ ] **Step 2: Add FLL NCO overlay on PLL filtered subplot**

Change the Filtered PLL discriminator plot (handles 3,1) to also show FLL:

Replace lines 136-144:

```matlab
%----- PLL/FLL discriminator filtered ----------------------------------
plot  (handles(3, 1), timeAxisInSeconds, ...
                      trackResults(channelNr).pllDiscrFilt, 'b', ...
                      timeAxisInSeconds, ...
                      trackResults(channelNr).fllDiscrFilt, 'm');      

grid  (handles(3, 1));
axis  (handles(3, 1), 'tight');
xlabel(handles(3, 1), 'Time (s)');
ylabel(handles(3, 1), 'Amplitude');
title (handles(3, 1), 'Filtered PLL/FLL discriminator');
hLegend = legend(handles(3, 1), 'PLL', 'FLL');
```

- [ ] **Step 3: Commit**

```bash
git add plotTracking.m
git commit -m "feat: add FLL discriminator traces to plotTracking"
```

---

### Task 6: Verification

**Files:** None (read-only)

- [ ] **Step 1: Check MATLAB code analysis**

```matlab
% In MATLAB:
checkcode tracking.m
checkcode plotTracking.m
```

Expected: No new errors or warnings related to FLL changes.

- [ ] **Step 2: Dry-run tracking (if MATLAB available)**

Run `init` and verify:
- Tracking completes without errors
- `trackResults.fllDiscr` and `trackResults.fllDiscrFilt` are populated (not all inf)
- FLL discriminator values are non-zero after the first few ms
- `carrFreqBasis` changes over time (no longer fixed at acquisition freq)
- Plot shows FLL traces (green/magenta) alongside PLL traces

- [ ] **Step 3: Commit any verification fixes if needed**
