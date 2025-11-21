function [setpoints, wave_data, sim_params] = sim_setup(varargin)


%SIM_SETUP  Initialize simulation parameters and reference signals.
%
% Usage examples:
%   sim_setup();
%   sim_setup('model', 'hydrofoil_sim_unified', 'sim_params', struct('Ts', 0.05));
%   sim_setup('ref_defs', {struct('type','step','channel',4,'start_time',1,'increment',0.1)});
%   sim_setup('wave_data', struct('amplitude',0.1,'omega',1.2));
%
% Outputs:
%   setpoints   : struct with .reference and .signals
%   wave_data   : struct with applied wave configuration
%   sim_params  : struct with Ts and Tsim
% 
% Behavior note: user-set ref_defs APPEND to defaults


    %% --- Default values ---
    defaults.model = "hydrofoil_sim_unified";
    defaults.sim_params = struct('Ts', 0.1, 'Tsim', 5, 'Ts_reference_preview', 0.1);
    defaults.rpm_const = 7;%1.1695e3;
    defaults.ref_defs = {
        struct('type','const','channel',1,'value',defaults.rpm_const), ...
        struct('type','const','channel',2,'value',-0.3), ...
        struct('type','const','channel',3,'value',0), ...
        struct('type','const','channel',4,'value',0), ...
        struct('type','const','channel',5,'value',0), ...
        struct('type','const','channel',6,'value',0), ...
        struct('type','const','channel',7,'value',0)
    };
    defaults.wave_data = struct( ...
        'amplitude', 0, ...
        'omega', 0, ...
        'wavenumber', 0, ...
        'phase', 0, ...
        'direction', 0);

    %% --- Input parser setup ---
    p = inputParser;
    p.FunctionName = 'sim_setup';

    addParameter(p, 'model', defaults.model, @(x)ischar(x) || isstring(x));
    addParameter(p, 'sim_params', defaults.sim_params, @(x)isstruct(x));
    addParameter(p, 'ref_defs', defaults.ref_defs, @(x)iscell(x));
    addParameter(p, 'wave_data', defaults.wave_data, @(x)isstruct(x));
    addParameter(p, 'assign_base', false, @(x)islogical(x) || isscalar(x)); % do not change workspace

    parse(p, varargin{:});
    args = p.Results;

    %% --- Merge provided structs with defaults ---
    sim_params = merge_structs(defaults.sim_params, args.sim_params);
    wave_data  = merge_structs(defaults.wave_data,  args.wave_data);

    % Append user refs if they differ from defaults
    if ~isequal(args.ref_defs, defaults.ref_defs)
        ref_defs = [defaults.ref_defs, args.ref_defs];
    else
        ref_defs = defaults.ref_defs;
    end

    %% --- Generate reference signals ---
    % set_reference holds timeseries data, set_signals holds struct 
    % for MPC previewing
    [set_reference, set_signals] = reference_generator(sim_params.Tsim, sim_params.Ts, ref_defs);
    set_signals.signals.values = set_signals.signals.values(:,2:end);
    setpoints = struct('reference', set_reference, 'preview', set_signals);

    %% --- Optional base workspace assignment ---
    if args.assign_base
        assignin('base', 'model', args.model);
        assignin('base', 'T_s', sim_params.Ts);
        assignin('base', 'Tsim', sim_params.Tsim);
        assignin('base', 'wave_data', wave_data);
        assignin('base', 'setpoints', setpoints);
    end

    fprintf('Simulation setup complete: model=%s, Ts=%.3f, Tsim=%.2f\n', ...
            string(args.model), sim_params.Ts, sim_params.Tsim);
end


%% --- Helper: merge two structs safely ---
function s_out = merge_structs(defaults, overrides)
    s_out = defaults;
    if isempty(overrides), return; end
    fields_ = fieldnames(overrides);
    for i = 1:numel(fields_)
        f = fields_{i};
        s_out.(f) = overrides.(f);
    end
end
