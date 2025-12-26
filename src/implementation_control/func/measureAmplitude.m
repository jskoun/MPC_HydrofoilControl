function [A_mean, A_time, f_dom] = measureAmplitude(t, y, settleFraction)
    % t: time vector
    % y: signal (e.g., height, pitch, etc.)
    % settleFraction: fraction of time to ignore for transient (e.g. 0.2)

    if nargin < 3
        settleFraction = 0.2;
    end

    % 1. Remove initial transient
    N = numel(y);
    startIdx = round(N * settleFraction);
    t_ss = t(startIdx:end);
    y_ss = y(startIdx:end);

    % 2. Remove mean offset
    y_ss = y_ss - mean(y_ss);

    % 3. Peak detection for amplitude
    [pks_max, ~] = findpeaks(y_ss);
    [pks_min, ~] = findpeaks(-y_ss);
    pks_min = -pks_min;

    if isempty(pks_max) || isempty(pks_min)
        warning('No clear peaks detected.');
        A_mean = NaN;
    else
        A_mean = (mean(pks_max) - mean(pks_min)) / 2; % amplitude estimate
    end
    % 
    % % 4. Frequency estimate via FFT (optional)
    % Fs = 1 / mean(diff(t));
    % Y = fft(y_ss);
    % L = length(y_ss);
    % P2 = abs(Y / L);
    % P1 = P2(1:L/2+1);
    % P1(2:end-1) = 2*P1(2:end-1);
    % f = Fs * (0:(L/2)) / L;
    % [~, idx] = max(P1);
    % f_dom = f(idx); % dominant frequency
    % 
    % % 5. Track amplitude over time (optional sliding window)
    % win = round(0.5 / f_dom * Fs); % half-period window
    % A_time = movmax(y_ss, win) - movmin(y_ss, win);
    % A_time = A_time / 2;
end
