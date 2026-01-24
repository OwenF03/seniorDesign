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

%% Verify Frequency Spectrum
fprintf('Verifying frequency content...\n');
figure('Position', [100, 100, 1200, 400]);

% Plot 1: Real signal spectrum
subplot(1,3,1);
NFFT = 2048;
f_axis = (-NFFT/2:NFFT/2-1) * fs/NFFT;
spectrum_real = fftshift(fft(X_real_noisy(:,1), NFFT));
plot(f_axis/1e3, 20*log10(abs(spectrum_real)), 'b-', 'LineWidth', 1);
xlabel('Frequency (kHz)');
ylabel('Magnitude (dB)');
title('Real Signal Spectrum');
xlim([0, 100]); grid on;

% Plot 2: Analytic signal spectrum (after processing)
subplot(1,3,2);
spectrum_analytic = fftshift(fft(X_analytic_processed(:,1), NFFT));
plot(f_axis/1e3, 20*log10(abs(spectrum_analytic)), 'r-', 'LineWidth', 1);
xlabel('Frequency (kHz)');
ylabel('Magnitude (dB)');
title('Processed Analytic Signal Spectrum');
xlim([-50, 50]); grid on;

% Plot 3: Zoom on signal band
subplot(1,3,3);
plot(f_axis/1e3, 20*log10(abs(spectrum_analytic)), 'r-', 'LineWidth', 2);
xlabel('Frequency (kHz)');
ylabel('Magnitude (dB)');
title('Zoom: Signal Band (20-40 kHz equivalent)');
xlim([-25, 25]); grid on;

% Mark expected frequencies
expected_freqs = f_offset / 1e3;  % Convert to kHz
hold on;
for i = 1:length(expected_freqs)
    xline(expected_freqs(i), 'k--', sprintf('%.0f kHz', expected_freqs(i)));
end

%% Continue with MUSIC processing using the properly processed analytic signal
fprintf('Proceeding with MUSIC DOA estimation...\n');
X_analytic = X_analytic_processed;  % Use the properly processed signal

% Compute sample covariance matrix
R_xx = (X_analytic' * X_analytic) / N_samples;
R_xx = (R_xx + R_xx') / 2;  % Ensure Hermitian

%% Rest of your MUSIC processing remains the same...
[eigenvects, sEDArg] = eig(R_xx);

noise_eigenvects = eigenvects(:,N_sources+1:end);

% Compute steering vectors
elSpacing = 0.5;
elementPos = (0:N_elements-1)*elSpacing;
scanAng = -90:90;
sv = steervec(elementPos,scanAng);

% Calculate spatial spectrum
projection = sv' * noise_eigenvects; 
projection_abs_squared = sum(abs(projection).^2,2);
D = sum(abs((sv'*noise_eigenvects)).^2,2)+eps(1);
spec = sqrt(1./D).';
specAng = scanAng;

% Find DOA
[~,locs] = findpeaks(spec,'SortStr','descend');
D = min(N_sources,length(locs));

%% Method 1: Using musicdoa (Spectral MUSIC)
fprintf('=== Using musicdoa (Spectral MUSIC) ===\n');
[estimated_doas_music, music_spectrum] = musicdoa(R_xx, N_sources);

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

%% Enhanced Visualization
figure('Position', [100, 100, 1200, 900]);

% Subplot 1: MUSIC Spectrum
subplot(2,3,1);
angles = -90:1:90;
plot(angles, 10*log10(abs(music_spectrum)), 'b-', 'LineWidth', 2);
hold on;
plot(source_doas, interp1(angles, 10*log10(abs(music_spectrum)), source_doas), ...
     'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'True DOAs');
xlabel('Angle (degrees)');
ylabel('MUSIC Spectrum (dB)');
title('MUSIC Spectrum');
legend('Location', 'best');
grid on;

% Subplot 2: Sample Covariance Matrix
subplot(2,3,2);
imagesc(abs(R_xx));
colorbar;
title('Sample Covariance Matrix');
xlabel('Sensor Index');
ylabel('Sensor Index');
axis equal tight;

% Subplot 3: Real vs Analytic Signal Comparison
subplot(2,3,3);
plot(t(1:100)*1e6, X_real_noisy(1:100, 1), 'b-', 'LineWidth', 2, 'DisplayName', 'Real Signal');
hold on;
plot(t(1:100)*1e6, real(X_analytic(1:100, 1)), 'r--', 'LineWidth', 1, 'DisplayName', 'Real(Analytic)');
plot(t(1:100)*1e6, imag(X_analytic(1:100, 1)), 'g--', 'LineWidth', 1, 'DisplayName', 'Imag(Analytic)');
xlabel('Time (\mus)');
ylabel('Amplitude');
title('Real vs Analytic Signals (Sensor 1)');
legend('Location', 'best');
grid on;

% Subplot 4: Signal Spectra
subplot(2,3,4);
NFFT = 1024;
f = fs/2 * linspace(0, 1, NFFT/2+1);
P_real = abs(fft(X_real_noisy(:,1), NFFT)).^2 / NFFT;
P_analytic = abs(fft(X_analytic(:,1), NFFT)).^2 / NFFT;
plot(f/1e3, 10*log10(P_real(1:NFFT/2+1)), 'b-', 'DisplayName', 'Real Signal');
hold on;
plot(f/1e3, 10*log10(P_analytic(1:NFFT/2+1)), 'r-', 'DisplayName', 'Analytic Signal');
xlabel('Frequency (kHz)');
ylabel('Power Spectral Density (dB)');
title('Frequency Spectrum Comparison');
legend('Location', 'best');
grid on;

% Subplot 5: Reconstruction Error
subplot(2,3,5);
error_signal = X_real_noisy(1:100,1) - real(X_analytic(1:100,1));
plot(t(1:100)*1e6, error_signal, 'k-', 'LineWidth', 1);
xlabel('Time (\mus)');
ylabel('Amplitude');
title('Reconstruction Error');
grid on;

fprintf('\nSimulation complete! Real-to-analytic conversion implemented.\n');


% Save variables 