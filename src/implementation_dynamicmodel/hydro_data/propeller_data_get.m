% Compute KT and KQ for advance velocites Ja

rho = 1025;   % Density of water (kg/m^3)
D = 0.3;        % Propeller diameter (m)
PD = 1.1;     % Pitch/diameter ratio (typically 0.5-2.5)
AEAO = 0.35;  % Blade area ratio (ratio of expanded blade area to propeller disk area)
z = 3;        % Number of propeller blades

Ja = 0:0.01:1.23;
KT = zeros(1,length(Ja));
KQ = zeros(1,length(Ja));
eta = zeros(1,length(Ja));
for i = 1:length(Ja)
    [KT(i), KQ(i)] = wageningen(Ja(i),PD,AEAO,z);
    eta_0 = Ja(i)/(2*pi) * (KT(i)/KQ(i));
    if (KT(i) > 0 && KQ(i) >0 && Ja(i) >0)
        eta(i) = eta_0;
    else
        eta(i) = 0;
    end
end

figure;
hold on; grid on; box on;

ax = gca;
colors = ax.ColorOrder;

plot(Ja, KT, 'Color', colors(1,:), 'LineWidth', 1.5); % first color (blue)
plot(Ja, 10*KQ, 'Color', colors(2,:), 'LineWidth', 1.5); % second color
plot(Ja, eta, 'Color', colors(5,:), 'LineWidth', 1.5); % green (fifth)

xlim(ax, [0,1.25])
xlabel('Advance ratio $J$', 'Interpreter', 'latex');
ylabel('$K_T , \; 10K_Q , \; \eta$', 'Interpreter', 'latex');
title('Wageningen B-Series $z=3$, $A_E/A_O=0.35$, $P/D=1.1$','Interpreter', 'latex')
legend({'$K_T$', '$10K_Q$', '$\eta$'}, ...
       'Interpreter', 'latex', 'Location', 'Best');

set(gca, 'TickLabelInterpreter', 'latex');

hold off;