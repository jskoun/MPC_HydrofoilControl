function [setpoint_timeseries, setpoint_signals] = reference_generator(T_sim, Ts, ref_defs)
% Generate reference signals with constants, ramps, and steps
% 
% T_sim     - total simulation time (scalar)
% Ts        - sampling time (scalar)
% ref_defs  - cell array of structs, each defining one signal with fields:
%             .type: 'const' | 'step' | 'ramp'
%             .channel: column index in output
%             .start_time: start of event (s)
%             .increment: end value for step and ramp  
%             .duration: time it takes for ramp to reach set point
%             .value: constant at which the signal initializes
    time_ref = (0:Ts:T_sim)';
    n_samples = length(time_ref);
    
    % Determine number of channels
    n_channels = max(cellfun(@(s) s.channel, ref_defs));
    U = zeros(n_samples, n_channels);

    for k = 1:numel(ref_defs)
        def = ref_defs{k};
        ch = def.channel; 
        switch def.type
            case 'const'
                U(:, ch) = def.value;
            case 'smoothramp'
                idx = (time_ref >= def.start_time) & (time_ref <= (def.start_time+def.duration));

                index_array = find(idx==1);
                last_val_idx = index_array(1);


                tau = (time_ref(idx)-def.start_time)/def.duration; % normalized 0..1
                s = 3*tau.^2 - 2*tau.^3;  % smooth step profile
                U(idx, ch) = U(last_val_idx, ch) + def.increment * s;
                
                % Hold final value
                idx_hold = time_ref > (def.start_time+def.duration);
                U(idx_hold, ch) = U(last_val_idx, ch) + def.increment;
            case 'ramp'
                idx = (time_ref >= def.start_time) & (time_ref <= (def.start_time+def.duration));
                slope = def.increment / def.duration;
                index_array = find(idx==1);
                last_val_idx = index_array(1);
                U(idx, ch) = U(last_val_idx, ch)+slope * (time_ref(idx) - def.start_time);
                % Hold final value after ramp
                idx_hold = time_ref > (def.start_time+def.duration);
                U(idx_hold, ch) = U(last_val_idx, ch) + def.increment;
            case 'step'
                idx = time_ref >= def.start_time;
                index_array = find(idx==1);
                last_val_idx = index_array(1);
                U(idx, ch) = U(last_val_idx, ch)+def.increment; % value before + step
            otherwise
                disp('Unknown type: %s', def.type);
        end
    end

    setpoint_timeseries = timeseries(U, time_ref);
    setpoint_signals.time = time_ref;
    setpoint_signals.signals.values = U;
end