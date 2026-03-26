clc; clear; close all;

%% 1️⃣ Load .mat File
matFilePath = 'D:\ACVs\College\Final year Project\radar_data.mat';  
loadedData = load(matFilePath);  

%% 2️⃣ Extract IQ Data (Ensuring Correct Shape)
if isfield(loadedData, 'IQ_data')
    IQ_data = loadedData.IQ_data;  % Shape: (1 × 32 × 256)
else
    error('IQ_data not found in the .mat file.');
end

disp(['IQ_data Size: ', mat2str(size(IQ_data))]);  

%% 3️⃣ Define Radar Parameters
Fs = 4e6;  % Sampling rate (Hz)
fc = 77e9; % Radar operating frequency (Hz)
c = 3e8;   % Speed of light (m/s)
numChirps = size(IQ_data, 2);      % 32 Chirps
numADCSamples = size(IQ_data, 3);  % 256 ADC Samples

%% 4️⃣ Compute Range Profile (FFT Along ADC Samples)
rangeFFT = fft(IQ_data, [], 3);  
rangeProfile = abs(squeeze(rangeFFT(1,1,:)));  % Take 1st chirp 

%% 5️⃣ Compute Range Axis (Distance Estimation)
rangeBins = (0:numADCSamples-1) * (c / (2 * Fs));  % Convert ADC bins to meters

%% 6️⃣ Apply CA-CFAR Detection
cfarDetector = phased.CFARDetector( ...
    'NumTrainingCells', 10, ...    
    'NumGuardCells', 2, ...        
    'ThresholdFactor', 'Auto', ... 
    'Method', 'CA');  % ✅ Fix: "RS" replaced with "CA"

rangeProfilePower = rangeProfile.^2;
cfarMask = cfarDetector(rangeProfilePower, 1:numADCSamples);

detectedRanges = rangeBins(cfarMask==1);  
detectedValues = rangeProfile(cfarMask==1);

%% 7️⃣ Particle Filter Setup
numParticles = 1000;  
particles = detectedRanges + randn(numParticles, 1) * 0.1;  % Initialize particles around detected objects
weights = ones(numParticles, 1) / numParticles;  

%% 8️⃣ Particle Filter Update (Tracking)
for t = 1:10  
    particles = particles + randn(numParticles, 1) * 0.05;  
    
    % Weighting based on proximity to detected objects
    for i = 1:numParticles
        weights(i) = sum(exp(-0.5 * ((particles(i) - detectedRanges) / 0.2).^2));
    end
    
    % Normalize Weights
    weights = weights / sum(weights);

    % Resampling using CDF (Replaces randsample)
    cdf = cumsum(weights);
    randomValues = rand(numParticles, 1);
    indices = arrayfun(@(r) find(cdf >= r, 1, 'first'), randomValues);
    particles = particles(indices);
    
    weights = ones(numParticles, 1) / numParticles;  
end

%% 9️⃣ Plot Results
figure;

% 📊 Subplot 1: Range Profile with CFAR Detections

plot(rangeBins, 10*log10(rangeProfile), 'b', 'LineWidth', 2); hold on;
scatter(detectedRanges, 10*log10(detectedValues), 'ro', 'filled'); 
title('Range Profile with CFAR Detection');
xlabel('Range (m)');
ylabel('Amplitude (dB)');
legend('Range Profile', 'Detected Objects');
grid on;

% 📊 Subplot 2: Spectrum Representation (Fixed Dimension Error)
figure;
plot(rangeBins, abs(squeeze(rangeFFT(1,1,:))), 'r', 'LineWidth', 2);  % ✅ Fixed
title('Spectrum Representation');
xlabel('Range (m)');
ylabel('FFT Magnitude');
grid on;

disp('✅ Object detection and tracking completed successfully!');
