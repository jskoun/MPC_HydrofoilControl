function base_seed = resetSensorSeeds()
%RESETSEENSORSEEDS  Randomizes all sensor noise seeds and updates workspace.
%
%   base_seed = resetSensorSeeds() 
%   Randomly generates a new base seed (using system time) and assigns
%   deterministic offsets for each sensor seed. Updates the variables in
%   the MATLAB base workspace, so Simulink models immediately use them.
%
%   The function returns the base_seed so you can log it for reproducibility.

    base_seed = randi(1e6);

    % Compute individual seeds with deterministic offsets
    seeds = struct( ...
        'seed_sonar', base_seed + 1, ...
        'seed_accX',  base_seed + 2, ...
        'seed_accZ',  base_seed + 3, ...
        'seed_gps',   base_seed + 4, ...
        'seed_gyroX', base_seed + 5, ...
        'seed_gyroY', base_seed + 6, ...
        'seed_pitch', base_seed + 7, ...
        'seed_roll',  base_seed + 8, ...
        'seed_waves', base_seed + 9);

    % Push to base workspace for Simulink
    fields = fieldnames(seeds);
    for i = 1:numel(fields)
        assignin('base', fields{i}, seeds.(fields{i}));
    end

    % Optional: print to command window for traceability
    fprintf('Sensor seeds reset (base_seed = %d):\n', base_seed);
    disp(seeds);
end