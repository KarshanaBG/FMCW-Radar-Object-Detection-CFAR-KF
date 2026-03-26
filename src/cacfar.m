clc; clear; close all;

%% 1️⃣ Load .mat File (Radar IQ Data)
matFilePath = 'D:\ACVs\College\Final year Project\radar_data.mat';  
loadedData = load(matFilePath);

if isfield(loadedData, 'IQ_data')
    IQ_data = loadedData.IQ_data;  % Shape: (1 × 32 × 256)
else
    error('IQ_data not found in the .mat file.');
end

disp(['IQ_data Size: ', mat2str(size(IQ_data))]);  

%% 2️⃣ Define Radar Parameters
Fs = 4e6;  % Sampling rate (Hz)
fc = 77e9; % Radar operating frequency (Hz)
c = 3e8;   % Speed of light (m/s)
lambda = c / fc;  % Wavelength
numChirps = size(IQ_data, 2);      % 32 Chirps
numADCSamples = size(IQ_data, 3);  % 256 ADC Samples

%% 3️⃣ Compute Range Profile (FFT Along ADC Samples)
rangeFFT = fft(IQ_data, [], 3);  
rangeProfile = abs(squeeze(rangeFFT(1,1,:)));  % Take 1st chirp for analysis

%% 4️⃣ Compute Range Axis (Distance Estimation)
rangeBins = (0:numADCSamples-1) * (c / (2 * Fs));  % Convert ADC bins to meters

%% 5️⃣ Apply CA-CFAR Detection
% CA-CFAR Parameters
numTrainingCells = 10;  % Number of training cells
numGuardCells = 2;      % Number of guard cells
alpha_factor = 5;       % Threshold scaling factor

% Initialize detection mask
cfarMask = zeros(1, numADCSamples);

for i = (numTrainingCells + numGuardCells + 1):(numADCSamples - numTrainingCells - numGuardCells)
    % Define training region (excluding guard cells)
    trainingCells = [rangeProfile(i - numTrainingCells - numGuardCells : i - numGuardCells - 1); ...
                     rangeProfile(i + numGuardCells + 1 : i + numGuardCells + numTrainingCells)];
    
    % Compute threshold from training cells (mean-based CA-CFAR)
    threshold = alpha_factor * mean(trainingCells);
    
    % Compare with CUT (Cell Under Test)
    if rangeProfile(i) > threshold
        cfarMask(i) = 1;  % Object detected
    end
end

% Extract detected object positions
detectedRanges = rangeBins(cfarMask == 1); 

%% 6️⃣ Compute Doppler Spectrum (Range-Doppler Map)
dopplerFFT = fftshift(fft(IQ_data, [], 2), 2);
dopplerSpectrum = abs(squeeze(dopplerFFT(1, :, :))); 

% Compute Doppler Axis (Velocity Estimation)
dopplerBins = linspace(-Fs/2, Fs/2, numChirps) * (lambda / 2); % Doppler shift to velocity

%% 7️⃣ Plot Both Graphs as Subplots
figure;

% 🔵 Subplot 1: Range Profile with CFAR Detections
subplot(1,1,1);
plot(rangeBins, 10*log10(rangeProfile), 'b', 'LineWidth', 2); hold on;
scatter(rangeBins(cfarMask == 1), 10*log10(rangeProfile(cfarMask == 1)), 'ro', 'filled'); % Mark detections
title('Range Profile with CA-CFAR Detection');
xlabel('Range (m)');
ylabel('Amplitude (dB)');
legend('Range Profile', 'Detected Objects');
grid on;



disp('✅ CA-CFAR Object Detection with Subplot Output Completed!');
