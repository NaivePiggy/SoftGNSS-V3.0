function tests = testFllDiscriminator
%TESTFLLDISCRIMINATOR Unit tests for the FLL phase-difference discriminator.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'include'));
end

function testPhaseDeltaIsIndependentOfAmplitude(testCase)
phaseDeltaCycles = 0.0125;
phaseDelta = 2 * pi * phaseDeltaCycles;

errorSmall = fllDiscriminator(100, 0, ...
    100 * cos(phaseDelta), 100 * sin(phaseDelta));
errorLarge = fllDiscriminator(10000, 0, ...
    10000 * cos(phaseDelta), 10000 * sin(phaseDelta));

verifyEqual(testCase, errorSmall, phaseDeltaCycles, 'AbsTol', 1e-12);
verifyEqual(testCase, errorLarge, phaseDeltaCycles, 'AbsTol', 1e-12);
end

function testPhaseDeltaIgnoresDataBitPolarityFlip(testCase)
phaseDeltaCycles = 0.0125;
phaseDelta = 2 * pi * phaseDeltaCycles;

errorNoFlip = fllDiscriminator(100, 0, ...
    100 * cos(phaseDelta), 100 * sin(phaseDelta));
errorWithFlip = fllDiscriminator(100, 0, ...
    -100 * cos(phaseDelta), -100 * sin(phaseDelta));

verifyEqual(testCase, errorNoFlip, phaseDeltaCycles, 'AbsTol', 1e-12);
verifyEqual(testCase, errorWithFlip, phaseDeltaCycles, 'AbsTol', 1e-12);
end

function testFoldedAtan2KeepsDataBitFlipEquivalent(testCase)
phaseDeltaCycles = -0.2;
phaseDelta = 2 * pi * phaseDeltaCycles;

errorNoFlip = fllDiscriminator(100, 0, ...
    100 * cos(phaseDelta), 100 * sin(phaseDelta));
errorWithFlip = fllDiscriminator(100, 0, ...
    -100 * cos(phaseDelta), -100 * sin(phaseDelta));

verifyEqual(testCase, errorNoFlip, phaseDeltaCycles, 'AbsTol', 1e-12);
verifyEqual(testCase, errorWithFlip, phaseDeltaCycles, 'AbsTol', 1e-12);
end
