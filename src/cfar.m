clc; clear; close all;

matFilePath = 'D:\ACVs\College\Final year Project\radar_data.mat';  
loadedData = load(matFilePath);  

if isfield(loadedData, 'IQ_data')
    IQ_data = loadedData.IQ_data;  % Shape: (1 × 32 × 256)
else
    error('IQ_data not found in the .mat file.');
end

disp(['IQ_data Size: ', mat2str(size(IQ_data))]);  

Fs = 4e6;  % Sampling rate (Hz)
fc = 77e9; % Radar operating frequency (Hz)
c = 3e8;   % Speed of light (m/s)
numChirps = size(IQ_data, 2);      % 32 Chirps
numADCSamples = size(IQ_data, 3);  % 256 ADC Samples

% Compute Range Profile (FFT Along ADC Samples)
rangeFFT = fft(IQ_data, [], 3);  
rangeProfile = abs(squeeze(rangeFFT(1,1,:)));  % Take 1st chirp (since object is stationary)

% Compute Range Axis (Distance Estimation)
rangeBins = (0:numADCSamples-1) * (c / (2 * Fs));  % Convert ADC bins to meters

% Apply CFAR Detection on Range Profile
cfarDetector = phased.CFARDetector( ...
    'NumTrainingCells', 10, ...    % Number of training cells
    'NumGuardCells', 2, ...        % Number of guard cells
    'ThresholdFactor', 'Auto', ... % Automatically calculate threshold
    'Method', 'CA');               % Cell Averaging CFAR

% Convert to power for CFAR processing
rangeProfilePower = rangeProfile.^2;

% Apply CFAR detector along the range axis
cfarMask = cfarDetector(rangeProfilePower, 1:numADCSamples);

figure;
plot(rangeBins, 10*log10(rangeProfile), 'b', 'LineWidth', 2); hold on;
scatter(rangeBins(cfarMask==1), 10*log10(rangeProfile(cfarMask==1)), 'ro', 'filled'); % Mark detections
title('Range Profile with CFAR Detection');
xlabel('Range (m)');
ylabel('Amplitude (dB)');
legend('Range Profile', 'Detected Objects');
grid on;
