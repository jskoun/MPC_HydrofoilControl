function [state, control_signal, actuator_output, reference_resampled, time_sim, metrics] = benchmark(model, signal_names, setpoint_reference, T_sim, Utrim, scale)


% Run experiment (simulation) and return results plus benchmarks
% Model - the model to run the exepriment on
% signal_names - The signals to use
% setpoint_reference - reference tracking/actuator setpoints
% T_sim - Simulation time_sim
% 
% Outputs 
% state - state values during simulation time
% control_signal - Generated control commands
% actuator_output - Actuator actual values
% reference_resampled - Exactly what it says. You need to resample the reference to perform benchmarks.
% time_sim - Simulation time points



% I would make Utrim global since it is always the same, 
% matlab however does this, as everything else, in a 
% convoluted way.

if nargin < 6 || isempty(scale)
    scale.state = [0.1 deg2rad(5) deg2rad(5)].^(-1);
    scale.input = [deg2rad(5) deg2rad(5) deg2rad(5)].^(-1);
end

reference_time = setpoint_reference.Time;

simOut = sim(model, ...
    'StopTime', num2str(T_sim));

logs = simOut.logsout;


N = length(signal_names.state);

time_sim = logs.get(signal_names.state(1)).Values.Time;  % Use time vector from any signal
Xsim = zeros(length(time_sim), N);

for i = 1:N
    sig = logs.get(signal_names.state(i));
    Xsim(:, i) = sig.Values.Data;
end



Usim = logs.get(signal_names.input_actual).Values.Data;
Usim = squeeze(permute(Usim, [3, 1, 2]));
ctrlUsim = logs.get(signal_names.input).Values.Data;
ctrlUsim = squeeze(permute(ctrlUsim, [3, 1, 2]));
ctrlTime = logs.get(signal_names.input).Values.Time;

% resample reference signals wrt time_sim:
reference_resampled = zeros(length(Xsim(:,1)), N);
for i=1:N
    given_reference = setpoint_reference.Data(:,i+1); % first one is rpm
    reference_resampled(:,i) = interp1(reference_time, given_reference, time_sim, "linear","extrap"); 
end

state = Xsim;
actuator_output = Usim;

%%
% this ruins how general my function is
% I should look into oop/function templates etc 

xb_vel = logs.get(signal_names.x_bf_velocity).Values.Data; % keep track for Utrim

% get indices of xb_vel found in Utrim
idx = max(1, min(round((xb_vel - 5) / 0.5) + 1, numel(Utrim))); 
% apply these in calculating offset
minimum_u = cell2mat(Utrim(idx).');
% which you shall remove from Usim but only 3 first are foil angles
input_offset = Usim - minimum_u(:,1:3) ;
%%

% Upsample control_signal to match Usim (e.g., repeat each value 10 times)

control_signal = interp1(ctrlTime, ctrlUsim, time_sim, "previous","extrap");

metrics = evaluatePerformance(time_sim, scale.state.*reference_resampled(:,1:3), scale.state.*Xsim(:,1:3), scale.input.*input_offset, xb_vel); % z, phi, theta tracking
%disp(metrics)

%% performance metric %%
% does not scale, please pre-scale your states
function metrics = evaluatePerformance(time, reference, state, input, velocity)

    % calculate performance of control system by measuring ref, state, input 
    % deviations and calculating a performance idex.
    % ISE - like criterion

    error = state - reference;
    %figure
    %hold on
    %plot(time, sum(error(:,1:3).^2,2));
    metrics.ISE = trapz(time, sum(error.^2,2));
    metrics.IAE = trapz(time, sum(abs(error),2));
    metrics.ITAE = trapz(time, time .* sum(abs(error),2));

    metrics.ISEheave = trapz(time, error(:,1).^2);
    metrics.ISEpitch = trapz(time, error(:,3).^2);
    metrics.ISEroll = trapz(time, error(:,2).^2);

    metrics.Sunk = any(diff(sign(state(:,1))) ~= 0);
    metrics.Converge = has_converged(state(:,1), reference(:,1), 0.1, 0.02, 0.008);

    metrics.controlEffort = trapz(time, sum(input.^2,2));

    metrics.velocity = velocity;
end

function converged = has_converged(y, r, frac, tol_mean, tol_std)
    % frac = fraction of tail to analyze (e.g., 0.1)
    % tol_mean = acceptable mean absolute error
    % tol_std = acceptable oscillation level
    N = numel(y);
    idx_start = round((1 - frac) * N);
    
    err = y(idx_start:end) - r(idx_start:end);
    
    mean_err = mean(abs(err));
    std_err = std(err);
    
    converged = (mean_err < tol_mean) && (std_err < tol_std);

end

end

