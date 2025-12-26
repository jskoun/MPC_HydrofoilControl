close all

V_min = 5;
V_max = 15;
N = 21;

V_vals = linspace(V_min, V_max, N); 

figure; hold on;
eig_vals = zeros(7, N); 

for i = 1:N
    A = A_all{i};
    eigs_i = eig(A);                      % Compute eigenvalues
    eigs_i = sort(eigs_i, 'descend');     % Sort (e.g., by magnitude or real part)
    eig_vals(:, i) = eigs_i;              % Store column-wise
end
% Now plot: real parts
eig_vals = eig_vals(:,2:end);
figure;
plot(V_vals(2:end), real(eig_vals.'), 'LineWidth', 1.5);  % Transpose for rows = different eig index
xlabel('Speed');
ylabel('Real part of eigenvalues');
title('Sorted Eigenvalues (Real Part)');
grid on;



figure; hold on; grid on;

rpm = zeros(1,N);
inputs = zeros(3, N);
for i = 1:N
    rpm(i) = Utrim{i}(4);
    inputs(:,i) = Utrim{i}(1:3);

end

% Define colors. You can adjust the RGB values as needed.
navy_blue      = [0 0.23 0.5];
light_blue     = [0.4 0.65 0.9];
dark_orange    = [0 0 0];
brown_orange   = [0.2 0.2 0.2];

yyaxis left
% rpm: dark orange, dashed
plot(V_vals, rpm, 'LineWidth', 1.5, 'Color', dark_orange, 'LineStyle','--')
hold on
% Thrust: brown/dark orange, solid
J = (1-propwake)*V_vals./(propD.*(rpm./60));
thrust = polyval(propKtp,J).*rho.*(rpm./60).^2*propD.^4;
plot(V_vals, 10*thrust, 'LineWidth', 1.5, 'Color', brown_orange, 'LineStyle','-')
ylabel("rpm, $10\cdot$Thrust (N)", 'Interpreter', 'latex')

yyaxis right
% Angle F: navy blue, solid
plot(V_vals, rad2deg(inputs(1,:)), 'LineWidth', 1.5, 'Color', navy_blue, 'LineStyle','-')
% Angle A: light blue, solid
plot(V_vals, rad2deg(inputs(3,:)), 'LineWidth', 1.5, 'Color', light_blue, 'LineStyle','-')

leg = legend(["rpm","Thrust","Angle F","Angle A"]);
xlabel("V (m/s)", 'Interpreter', 'latex')
ylabel("Foil deflection (deg)", 'Interpreter', 'latex')
set(gca,'TickLabelInterpreter','latex')


yyaxis left
ax = gca;
ax.YColor = [0 0 0]; % black
yyaxis right
ax.YColor = [0 0 0]; % black

% Indices of interest
state_idx = [1, 3, 4, 6];
input_idx = 3;

% Preallocate storage for the gains
num_sys = numel(sys_array);

B43 = zeros(num_sys, 1);  % Effect of input 3 on w (state 4)
B63 = zeros(num_sys, 1);  % Effect of input 3 on q (state 6)

for i = 1:num_sys
    sys = sys_array{i};

    sys.A(6,4) = 0;
    sys.A(6,5) = 0;
    sys.A(6,7) = 0;

    red_sys = ss(sys.A([1:3, 5:7], [1:3, 5:7]), sys.B([1:3, 5:7],[1:3]), eye(6), 0);
    
    B43(i) = red_sys.B(4,3);  % Input 3 → w dynamics
    B63(i) = red_sys.B(6,3);  % Input 3 → q dynamics
end
% Plot
figure;
plot(V_vals, B43, 'LineWidth', 1.5); hold on;
plot(V_vals, B63, 'LineWidth', 1.5);
xlabel('Velocity (m/s)');
ylabel('B(i,3) – Input 3 Effect on State i');
legend('B(4,3): w', 'B(6,3): q', 'Location', 'Best');
title('Effect of Input 3 on w and q Dynamics vs Velocity');
grid on;