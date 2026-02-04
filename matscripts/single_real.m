%% Underwater Acoustic Parameters
c = 1500;               % Speed of sound in water (m/s)
fc = 30e3;              % Center frequency for processing (30 kHz)
lambda = c/fc;          % Wavelength for array geometry
fs = 400e3;             % Sampling frequency (400 kHz)
Ts = 1/fs;
N_samples = 128;
SNR_dB = 10;

%% Hydrophone Array Configuration
N_elements = 4;
d = lambda/2;
array_pos = (0:N_elements-1)*d;

%% Acoustic Sources - Define as OFFSETS from center frequency
source_doas = [80, -40];
N_sources = length(source_doas);
source_powers = [1, 1];

% Define frequency OFFSETS from center frequency (not absolute frequencies)
f_offset = [-500, 500, 0];  % This gives: 22kHz, 28kHz, 35kHz

%% Generate PROPER Analytic Signals First
fprintf('Generating proper analytic signals...\n');
t = (0:N_samples-1) * Ts;
X_analytic = zeros(N_samples, N_elements);

for src_idx = 1:N_sources
    % Generate analytic (complex) signal at the correct frequency
    % If f_offset is [0,0,0], all sources are at carrier (30kHz)
    
    % Add a random starting phase to the carrier
    % This ensures the carrier waves are not perfectly in sync
    random_carrier_phase = 2*pi*rand; 
    
    baseband_signal = sqrt(source_powers(src_idx)) * ...
                     exp(1j*2*pi*f_offset(src_idx)*t + 1j*random_carrier_phase);
    
    % Decorrelate the Amplitude Modulation (AM)
    % By giving each source a random phase for its envelope, they become
    % statistically independent signals.
    random_am_phase = 2*pi*rand;
    % Much faster decorrelating modulation
    am_freq = 10e3 + 5e3*rand;  % Random 10-15 kHz modulation
    am_modulation = 1 + 0.3*sin(2*pi*am_freq*t + random_am_phase);
    %am_modulation = 1 + 0.1*sin(2*pi*50*t + random_am_phase);
    
    baseband_signal = baseband_signal .* am_modulation;
    
    % Steering vector (using center frequency for array geometry)
    sv = exp(-1j * 2 * pi * array_pos/lambda * sind(source_doas(src_idx)));
    
    % Add to hydrophone array
    X_analytic = X_analytic + baseband_signal.' * sv;
end



%% Convert to REAL signal (what transducers actually measure)
fprintf('Converting to real signal...\n');
X_real = real(X_analytic .* exp(1j*2*pi*fc*t'));  % Upconvert to carrier

%% Add REAL Noise
noise_power = 10^(-SNR_dB/10);
real_noise = sqrt(noise_power) * randn(size(X_real));

% Add colored noise components
for i = 1:5
    noise_freq = 10e3 + 30e3*rand;  % Noise in broader band
    colored_noise = 0.1*sqrt(noise_power) * randn * ...
                    sin(2*pi*noise_freq*t + 2*pi*rand)';
    real_noise = real_noise + colored_noise * ones(1, N_elements);
end

X_real_noisy = X_real + real_noise;

%% Convert BACK to Analytic for MUSIC processing
fprintf('Converting back to analytic signal for processing...\n');

% Method 1: Demodulate to baseband
%X_analytic_processed = zeros(size(X_real_noisy));
X_analytic_processed = hilbert(X_real_noisy);

% for i = 1:N_elements
%     % Downconvert by multiplying by complex exponential at carrier frequency
%     baseband = X_real_noisy(:, i) .* exp(-1j*2*pi*fc*t');
% 
%     % Apply low-pass filter to remove high-frequency components
%     [b, a] = butter(6, 40e3/(fs/2));  % 40 kHz cutoff
%     baseband_filtered = filter(b, a, baseband);
% 
%     X_analytic_processed(:, i) = baseband_filtered;
% end

% Method 2: Alternatively, use Hilbert transform (simpler but less accurate)
% X_analytic_hilbert = hilbert(X_real_noisy);

%% Continue with MUSIC processing using the properly processed analytic signal
X_analytic = X_analytic_processed;  % Use the properly processed signal

% Compute sample covariance matrix
R_xx = (X_analytic' * X_analytic) / N_samples;
R_xx = (R_xx + R_xx') / 2;  % Ensure Hermitian

%Save c++ data 
save("test_data/input_data.mat", "X_real_noisy", "R_xx");
save("vars/variables", "R_xx")

%% Rest of your MUSIC processing remains the same...
[eigenvects, sEDArg] = eig(R_xx);

eigen_vals = eig(R_xx);
save("vars/variables", "eigen_vals", "-append")
eigen_vects = eigenvects; % different name for consistent convention for testing
save("vars/variables", "eigen_vects", "-append")
noise_eigenvects = eigenvects(:,N_sources+1:end);
save("vars/variables", "noise_eigenvects", "-append")

% Compute steering vectors
elSpacing = 0.5;
elementPos = (0:N_elements-1)*elSpacing;
scanAng = -90:90;
sv = steervec(elementPos,scanAng);

% Save steering vector 
save("vars/variables", "sv", "-append")

% Calculate spatial spectrum
projection = sv' * noise_eigenvects; 
projection_abs_squared = sum(abs(projection).^2,2);
D = sum(abs((sv'*noise_eigenvects)).^2,2)+eps(1);
spec = sqrt(1./D).';
save("vars/variables", "spec", "-append")
specAng = scanAng;

% Find DOA
[~,locs] = findpeaks(spec,'SortStr','descend');
D = min(N_sources,length(locs));

%% Method 1: Using musicdoa (Spectral MUSIC)
fprintf('=== Using musicdoa (Spectral MUSIC) ===\n');
[estimated_doas_music, music_spectrum] = musicdoa(R_xx, N_sources);

save("vars/variables", "estimated_doas_music", "-append")

fprintf('True DOAs:        %s degrees\n', mat2str(source_doas, 2));
fprintf('Estimated DOAs:   %s degrees\n', mat2str(estimated_doas_music, 2));

errors_music = abs(sort(estimated_doas_music) - sort(source_doas));
fprintf('Estimation errors: %s degrees\n\n', mat2str(errors_music, 2));

%% Method 2: Using rootmusicdoa (Root-MUSIC)
fprintf('=== Using rootmusicdoa (Root-MUSIC) ===\n');
estimated_doas_root = rootmusicdoa(R_xx, N_sources);

fprintf('True DOAs:        %s degrees\n', mat2str(source_doas, 2));
fprintf('Estimated DOAs:   %s degrees\n', mat2str(estimated_doas_root, 2));

errors_root = abs(sort(estimated_doas_root) - sort(source_doas));
fprintf('Estimation errors: %s degrees\n\n', mat2str(errors_root, 2));

%% Performance Comparison
fprintf('=== PERFORMANCE COMPARISON ===\n');
rmse_music = sqrt(mean(errors_music.^2));
rmse_root = sqrt(mean(errors_root.^2));

fprintf('RMSE - musicdoa:      %.3f degrees\n', rmse_music);
fprintf('RMSE - rootmusicdoa:  %.3f degrees\n', rmse_root);

fprintf('\nSimulation complete! Real-to-analytic conversion implemented.\n');


% Save variables 
