%% Corrected Simulation
clear; close all; clc;

c = 1500;
fs = 400e3;
Ts = 1/fs;
N_samples = 256;  % Increased for better resolution
SNR_dB = 10;
N_elements = 4;

% TWO PINGERS at different frequencies
pinger_freqs = [25e3, 30e3];
pinger_doas = [30, -15];
N_pingers = 2;
pinger_powers = [1, 0.8];

% Use REFERENCE frequency for array geometry
fc_ref = 27.5e3;  % Geometric mean of 25 and 30 kHz
lambda_ref = c / fc_ref;
d = lambda_ref / 2;
array_pos = (0:N_elements-1) * d;

t = (0:N_samples-1) * Ts;
X_real = zeros(N_samples, N_elements);

% Generate each pinger with PROPER time delays
for p = 1:N_pingers
    freq = pinger_freqs(p);
    angle_deg = pinger_doas(p);
    
    % Calculate time delay for each sensor (in seconds)
    % tau = (sensor_position / c) * sin(angle)
    time_delays = array_pos * sind(angle_deg) / c;
    
    % Generate signal at each sensor with proper time delay
    for sensor = 1:N_elements
        % Time vector shifted by delay
        t_delayed = t - time_delays(sensor);
        
        % Real sine wave with time delay
        X_real(:, sensor) = X_real(:, sensor) + ...
            sqrt(pinger_powers(p)) * sin(2*pi*freq*t_delayed');
    end
end

% Add noise
noise_power = 10^(-SNR_dB/10);
X_real_noisy = X_real + sqrt(noise_power) * randn(size(X_real));

%% Process EACH pinger separately
for p = 1:N_pingers
    fprintf('\n=== Processing Pinger %d (%.1f kHz) ===\n', p, pinger_freqs(p)/1e3);
    
    % Bandpass filter around this pinger's frequency
    fc = pinger_freqs(p);
    bw = 4e3;  % 4 kHz bandwidth
    [b, a] = butter(4, [fc-bw/2, fc+bw/2]/(fs/2));
    X_filtered = zeros(size(X_real_noisy));
    for ch = 1:N_elements
        X_filtered(:, ch) = filtfilt(b, a, X_real_noisy(:, ch));  % Use filtfilt for zero-phase
    end
    
    % Convert to analytic
    X_analytic = hilbert(X_filtered);
    
    % Compute covariance
    R_xx = (X_analytic' * X_analytic) / N_samples;
    R_xx = (R_xx + R_xx') / 2;
    
    % MUSIC with normalized half-wavelength spacing
    elSpacing = 0.5;
    [estimated_doa, spectrum] = musicdoa(R_xx, 1);
    
    fprintf('True DOA: %d degrees\n', pinger_doas(p));
    fprintf('Estimated DOA: %.1f degrees\n', estimated_doa);
    fprintf('Error: %.1f degrees\n\n', abs(estimated_doa - pinger_doas(p)));
    
    % Plot spectrum
    figure;
    angles = -90:90;
    plot(angles, 10*log10(abs(spectrum)), 'b-', 'LineWidth', 2);
    hold on;
    xline(pinger_doas(p), 'r--', 'LineWidth', 2, 'Label', 'True DOA');
    xline(estimated_doa, 'g--', 'LineWidth', 2, 'Label', 'Estimated');
    xlabel('Angle (degrees)');
    ylabel('MUSIC Spectrum (dB)');
    title(sprintf('Pinger %d (%.1f kHz)', p, fc/1e3));
    grid on;
    legend('Location', 'best');
end
