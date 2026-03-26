clc; clear; close all;

%% Load .mat File (Radar IQ Data)
matFilePath = 'D:\ACVs\College\Final year Project\radar_data.mat';  
loadedData = load(matFilePath);

if isfield(loadedData, 'IQ_data')
    IQ_data = loadedData.IQ_data;  
else
    error('IQ_data not found in the .mat file.');
end

disp(['IQ_data Size: ', mat2str(size(IQ_data))]);  

%% Define Radar Parameters
Fs = 4e6;  % Sampling rate (Hz)
fc = 77e9; % Radar operating frequency (Hz)
c = 3e8;   % Speed of light (m/s)
numChirps = size(IQ_data, 2);      % 32 Chirps
numADCSamples = size(IQ_data, 3);  % 256 ADC Samples

%% Compute Range Profile (FFT Along ADC Samples)
rangeFFT = fft(IQ_data, [], 3);  
rangeProfile = abs(squeeze(rangeFFT(1,1,:)));  

%% Compute Range Axis (Distance Estimation)
rangeBins = (0:numADCSamples-1) * (c / (2 * Fs));  

%% CFAR Detection (Cell-Averaging CFAR)
numTrainingCells = 10;  
numGuardCells = 2;      
alpha_factor = 5.0;     

cfarMask = zeros(1, numADCSamples);

for i = numTrainingCells + numGuardCells + 1 : numADCSamples - numTrainingCells - numGuardCells
    trainingCells = [rangeProfile(i - numTrainingCells - numGuardCells : i - numGuardCells - 1);
                     rangeProfile(i + numGuardCells + 1 : i + numGuardCells + numTrainingCells)];

    avgNoise = mean(trainingCells);
    threshold = alpha_factor * avgNoise;
    
    if rangeProfile(i) > threshold
        cfarMask(i) = 1;  
    end
end

%% Manual UKF Tracking
dt = 0.1;  
x = [0; 0];  % Initial state [position; velocity]
P = eye(2);  % Covariance matrix
Q = 0.01 * eye(2);  
R = 1;  

detectedRanges = rangeBins(cfarMask == 1);
numFrames = 10;  
trackedPositions = zeros(numFrames, 1);

for k = 1:numFrames
    % Prediction Step
    A = [1 dt; 0 1];  
    x = A * x;  
    P = A * P * A' + Q;  

    % Measurement Update (Only if objects detected)
    if ~isempty(detectedRanges)
        measuredPosition = mean(detectedRanges);  
        K = P(:,1) / (P(1,1) + R);  
        x = x + K * (measuredPosition - x(1));  
        P = (eye(2) - K * [1 0]) * P;  
    end
    
    trackedPositions(k) = x(1);
end

%% Plot Results
figure;

% Subplot 1: Range Profile with CFAR Detections

plot(rangeBins, 10*log10(rangeProfile), 'b', 'LineWidth', 2); hold on;
scatter(rangeBins(cfarMask==1), 10*log10(rangeProfile(cfarMask==1)), 'ro', 'filled');  
title('Range Profile with CFAR Detection');
xlabel('Range (m)');
ylabel('Amplitude (dB)');
legend('Range Profile', 'Detected Objects');
grid on;

% Subplot 2: UKF Tracking of Detected Objects
figure;

plot(1:numFrames, trackedPositions, 'k-o', 'LineWidth', 2, 'MarkerFaceColor', 'g');
title('UKF Object Tracking');
xlabel('Time Frames');
ylabel('Tracked Object Position (m)');
grid on;

disp('CFAR detection and manual UKF tracking complete!');
