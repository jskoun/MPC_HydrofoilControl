function mpc_controller = generate_mpc(plant_model, Utrim, varargin)
% calculate_mpc Create an MPC controller from an existing continuous-time plant
% 
% Usage:
%   mpc_controller = generate_mpc(plant_model, Utrim)
%   mpc_controller = generate_mpc(plant_model, Utrim, 'Ts', 0.1, ...
%       'PredictionHorizon', 10, 'ControlHorizon', 2, ...
%       'MVWeights', [0.2 0.2 0.2], 'MVRateWeights', [0 0 0], ...
%       'OVWeights', [1.5 1 1.5 0.2 0.2 0.2], 'ECRWeight', 100000, ...
%       'OutDist', 'integrators')

%% Input parser
p = inputParser;

% Required args: plant_model, Utrim
% Optional parameters with defaults:
p.addParameter('Ts', 0.1, @(x) isnumeric(x) && x>0);
p.addParameter('PredictionHorizon', 10, @(x) isnumeric(x) && x>0);
p.addParameter('ControlHorizon', 2, @(x) isnumeric(x) && x>0);

p.addParameter('MVWeights', [0.2 0.2 0.2], @(x) isnumeric(x));
p.addParameter('MVRateWeights', [0.15 0.15 0.2], @(x) isnumeric(x));
p.addParameter('OVWeights', [4 3 4 0.1 0.1 0.1], @(x) isnumeric(x));
p.addParameter('ECRWeight', 100000, @(x) isnumeric(x) && isscalar(x));

p.addParameter('OutDist', 'integrators'); % Could be 'integrators', SS model, or []
p.parse(varargin{:});

Ts = p.Results.Ts;
PredictionHorizon = p.Results.PredictionHorizon;
ControlHorizon    = p.Results.ControlHorizon;

MVWeights    = p.Results.MVWeights;
MVRateWeights= p.Results.MVRateWeights;
OVWeights    = p.Results.OVWeights;
ECRWeight    = p.Results.ECRWeight;
OutDist      = p.Results.OutDist;

maxDeflect = deg2rad(10);

%% Create discrete-time model
if plant_model.Ts == 0
    dplant = c2d(plant_model, Ts);
else
    dplant = plant_model;
end

%% Create MPC controller
mpc_controller = mpc(dplant, Ts, PredictionHorizon, ControlHorizon);

%% Nominal values
mpc_controller.Model.Nominal.U = Utrim;
mpc_controller.Model.Nominal.Y = [-0.3;0;0;0;0;0];

%% Scaling
mpc_controller.MV(1).ScaleFactor = maxDeflect;
mpc_controller.MV(2).ScaleFactor = maxDeflect;
mpc_controller.MV(3).ScaleFactor = maxDeflect;

mpc_controller.OV(1).ScaleFactor = 0.2;
mpc_controller.OV(2).ScaleFactor = maxDeflect;
mpc_controller.OV(3).ScaleFactor = maxDeflect;

mpc_controller.OV(4).ScaleFactor = 0.5;
mpc_controller.OV(5).ScaleFactor = 0.5;
mpc_controller.OV(6).ScaleFactor = 0.5;

%% Constraints
for i = 1:3
    mpc_controller.MV(i).Min     = -maxDeflect;
    mpc_controller.MV(i).Max     = maxDeflect;
    mpc_controller.MV(i).RateMax = deg2rad(3);
end

%% Weights
mpc_controller.Weights.MV     = MVWeights;
mpc_controller.Weights.MVRate = MVRateWeights;
mpc_controller.Weights.OV     = OVWeights;
mpc_controller.Weights.ECR    = ECRWeight;

%% Output disturbance model
if isa(OutDist, 'char')
    setoutdist(mpc_controller, OutDist);
else 
    setoutdist(mpc_controller, 'model', OutDist);
end

end
