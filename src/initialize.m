addpath(genpath("implementation_control"))
addpath(genpath("implementation_dynmodel"))

model_parameters
% run make_linmodel_array
% creates linearizations
make_linmodel_array

calculate_lqr
calculate_mpc

% For presentable plots

%set(groot, 'defaultTextInterpreter','latex');
%set(groot, 'defaultLegendInterpreter','latex');
%set(groot, 'defaultAxesTickLabelInterpreter','latex');


