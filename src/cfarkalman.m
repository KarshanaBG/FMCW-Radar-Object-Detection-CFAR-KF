clc; clear; close all;

%% Load .mat File (Radar IQ Data)
matFilePath = 'D:\ACVs\College\Final year Project\radar_data.mat';  
loadedData = load(matFilePath);

if isfield(loadedData, 'IQ_data')
    IQ_data = loadedData.IQ_data;  % Shape: (1 × 32 × 256)
else
    error('IQ_data not found in the .mat file.');
end

disp(['IQ_data Size: ', mat2str(size(IQ_data))]);  

%%  Define Radar Parameters
Fs = 4e6;  % Sampling rate (Hz)
fc = 77e9; % Radar operating frequency (Hz)
c = 3e8;   % Speed of light (m/s)
numChirps = size(IQ_data, 2);      % 32 Chirps
numADCSamples = size(IQ_data, 3);  % 256 ADC Samples

%% Compute Range Profile (FFT Along ADC Samples)
rangeFFT = fft(IQ_data, [], 3);  
rangeProfile = abs(squeeze(rangeFFT(1,1,:)));  % Take 1st chirp (assuming stationary object)

%% Compute Range Axis (Distance Estimation)
rangeBins = (0:numADCSamples-1) * (c / (2 * Fs));  % Convert ADC bins to meters

%%️⃣ Apply CFAR for Object Detection
cfarDetector = phased.CFARDetector( ...
    'NumTrainingCells', 10, ...    % Number of training cells
    'NumGuardCells', 2, ...        % Number of guard cells
    'ThresholdFactor', 'Auto', ... % Automatically calculate threshold
    'Method', 'CA');               % Cell Averaging CFAR

rangeProfilePower = rangeProfile.^2;  % Convert to power for CFAR processing
cfarMask = cfarDetector(rangeProfilePower, 1:numADCSamples);

detectedRanges = rangeBins(cfarMask == 1); % Get detected object positions

%% Initialize Kalman Filter for Object Tracking
numDetections = length(detectedRanges);
trackedPositions = zeros(numDetections, 1); % Store filtered positions

% Define Kalman Filter Parameters
dt = 1; % Time step
A = [1 dt; 0 1];  % State transition matrix
H = [1 0];        % Measurement matrix
Q = [0.01 0; 0 0.01]; % Process noise covariance
R = 0.5;          % Measurement noise covariance
P = eye(2);       % Initial covariance matrix

% Initialize state vector (position, velocity)
if numDetections > 0
    x = [detectedRanges(1); 0];  % Assume first detection as initial position
else
    x = [0; 0];  % Default position if no detections
end

%% Apply Kalman Filter for Each Detected Object
for i = 1:numDetections
    % Predict Step
    x = A * x; 
    P = A * P * A' + Q;
    
    % Update Step
    K = P * H' / (H * P * H' + R);  % Kalman Gain
    x = x + K * (detectedRanges(i) - H * x);
    P = (eye(2) - K * H) * P;
    
    % Store the tracked position
    trackedPositions(i) = x(1);
end

%% Plot CFAR Detections and Kalman-Filtered Output
figure;
plot(rangeBins, 10*log10(rangeProfile), 'b', 'LineWidth', 2); hold on;
scatter(detectedRanges, 10*log10(rangeProfile(cfarMask == 1)), 'ro', 'filled'); % CFAR Detections
scatter(trackedPositions, 10*log10(rangeProfile(cfarMask == 1)), 'go', 'filled'); % Kalman Filter Output
title('Range Profile with CFAR Detection & Kalman Filtering');
xlabel('Range (m)');
ylabel('Amplitude (dB)');
legend('Range Profile', 'CFAR Detections', 'Kalman Filtered Track');
grid on;

disp('CFAR Detection and Kalman Filtering Completed!');
