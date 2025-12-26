%% set model %%
model = "hydrofoil_sim_unified";

control_flag = 2;
sensorfeedback_flag = 0;

%% set simulation time, reference signal %%

Tsim = 6;                 % total simulation time
Ts = 0.1;                   % sample time
Ts_reference_preview = 0.1; % diferent here
% Initialize signal
rpm_const = 1.1695e+03; % trim constant (7m/s)

% struct('type','ramp','channel',1,'start_time',2,'duration',4,'increment',1000), ...
% struct('type','smoothramp','channel',4,'start_time',1.1,'duration',0.5,'increment',deg2rad(2)), ...
% struct('type','smoothramp','channel',4,'start_time',1.6,'duration',0.6,'increment',-deg2rad(3.5)), ...
% struct('type','smoothramp','channel',4,'start_time',2.2,'duration',0.4,'increment',+deg2rad(1.5)), ...
% struct('type','ramp','channel',1,'start_time',1,'duration',2,'increment',+800), ...
% wave_data.amplitude = 0.1;
% wave_data.omega = 4;
% wave_data.wavenumber = wave_data.omega^2/g;
% wave_data.direction=0;
[setpoints, wave_data] = sim_setup( ...
    'sim_params',struct('Tsim',Tsim, 'Ts',Ts), ...
    'ref_defs', {struct('type','ramp','channel',1,'start_time',1,'duration',2,'increment',3), ...
    struct('type','smoothramp','channel',2,'start_time',2,'duration',1,'increment',-0.1)}); % wave_data dont matter if you set random sea

setpoint_reference = setpoints.reference;
setpoint_signals = setpoints.preview;
% setpoint_signals are fed to the reference previewing MPC controllers
% need to remove the rpm_reference 

% needs rework, u is input twice!
signal_names = struct('state',["z" "phi" "theta" "w" "p" "q"],'input','u_ctrl','x_bf_velocity','u','input_actual','actuator_position');

controllers = containers.Map( ...
    {'LQI', 'LQI-S', 'LQI-SW', 'MPC', 'MPC-I', 'MPC-O', 'MPC-IO', 'MPC-ID', 'MPC-IP', 'MPC-IDP', 'LPV-MPC-IP', 'LPV-MPC-IDP', 'MPC-IOP'}, ...    % keys (controller names)
    {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13} ...                          % values (control flags)
);


controller_subset= {'MPC-IDP', 'LPV-MPC-IDP'};

controller_subset= {'LQI', 'LQI-S', 'LQI-SW'};

controller_subset= {'MPC-IP', 'LPV-MPC-IDP', 'LQI-SW'};

N = length(controller_subset);
Xsim = cell(1, N);
control_signal = cell(1, N);
Usim = cell(1, N);
time_sim = cell(1,N);
metrics_results = cell(1,N);

for i=1:N
    controller_name = controller_subset{i};
    control_flag= controllers(controller_name);
    [Xsim{i}, control_signal{i}, Usim{i}, reference, time_sim{i}, metrics_results{i}] = benchmark(model, signal_names, setpoint_reference, Tsim, Utrim);
end

% Assume time_sim and reference are the same for all controllers
N = numel(Xsim); % Number of controllers
colorsmap = orderedcolors('dye');
colors = colorsmap(1:N,:);



f = figure;
t = tiledlayout(3,1);
t.TileSpacing = 'compact';
t.Padding = 'compact';

pick_linestyle = {'-', ':', '-.', '--'};

%% Helper for overlaying plots
plot_overlay = @(ax, time, ydata, color, label, idx) ...
    plot(ax, time, ydata, 'Color', color, 'LineWidth', 1.2, 'LineStyle', pick_linestyle(idx), 'DisplayName', label);

plot_overlay_stairs = @(ax, time, ydata, color, label, idx) ...
    stairs(ax, time, ydata, 'Color', color, 'LineWidth', 1.2, 'LineStyle', pick_linestyle(idx), 'DisplayName', label);

ax1 = nexttile(1); hold on;
plot(ax1, setpoint_signals.time, -setpoint_signals.signals.values(:,1), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
for i = 1:N
    plot_overlay(ax1, time_sim{i}, -Xsim{i}(:,1), colors(i,:), controller_subset{i}, i);
end
ylabel('Heave $z$ (m)'); title('Height Tracking'); legend('show');



% State 3
% ax2 = nexttile(2); hold on;
% plot(ax2, time_sim{N}, rad2deg(reference(:,3)), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
% for i = 1:N
%     plot_overlay(ax2, time_sim{i}, rad2deg(Xsim{i}(:,3)), colors(i,:), controller_subset{i}, i);
% end
% ylabel('Pitch $\theta$ (deg)'); title('Pitch Tracking'); legend('show');


ax2 = nexttile(2); hold on;
plot_overlay(ax2, time_sim{i}, metrics_results{i}.velocity, colors(i,:), controller_subset{i}, 1);
ylabel('$u$ (m/s)'); title('$u$, Forward Velocity in $\{b\}$'); %legend('show');

ax3 = nexttile(3); hold on;
for i = 1:N
    u_cmd = control_signal{i};
    plot_overlay_stairs(ax3, time_sim{i}, rad2deg(u_cmd(:,1)), colors(i,:), ['cmd signal ' controller_subset{i}], i);
end
xlabel('Time (s)'); ylabel('$u_1$ (deg)'); title('Fore Starboard Command'); %legend('show');


figure

t = tiledlayout(3,1);
t.TileSpacing = 'compact';
t.Padding = 'compact';


ax1 = nexttile(1); hold on;
for i = 1:N
    plot_overlay(ax1, time_sim{i}, -Xsim{i}(:,4), colors(i,:), controller_subset{i}, i);
end

ax2 = nexttile(2); hold on;
for i = 1:N
    plot_overlay(ax2, time_sim{i}, rad2deg(Xsim{i}(:,5)), colors(i,:), controller_subset{i}, i);
end

ax3 = nexttile(3); hold on;
for i = 1:N
    plot_overlay(ax3, time_sim{i}, rad2deg(Xsim{i}(:,6)), colors(i,:), controller_subset{i}, i);
end

%exportgraphics(f, 'myfigure.pdf', 'ContentType','vector', 'BackgroundColor','none');