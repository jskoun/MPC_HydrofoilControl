
% develops appropriate LTI-ss systems [cont_plant] for various MPC variants %
% uses generate_mpc to design controllers based on the plant models %

% Current controllers: 

% Linear MPC
% mpc, mpc_O, mpc_I, mpc_IO, mpc_IDP

% LPV MPC
% lpvmpc_IODP

% used for benchmarks: mpc_IP, mpc_IDP, lpvmpc_IDP

Ts = 0.1;
PredictionHorizon = 10;
ControlHorizon = 2;

%maxDeflect = deg2rad(10);


%% Base MPC 
cont_plant = reduced_model; % apparently it is a copy, not a pointer
cont_plant.InputName = {'uFS','uFP','uA'};
cont_plant.OutputName = {'z','phi','theta', 'w', 'p', 'q'};

mpc_base = generate_mpc(cont_plant, [Utrim{5}(1:3)'], 'OutDist', tf(zeros(6,1)));

%% Output Disturbance 
% % Disturbance model: d(k+1) = d(k) + w(k)
% Ad = eye(3);
% Bd = 0.1*eye(3);                 % no exogenous inputs
% Cd = [0 0 0;0 0 0;0 0 1;0 1 0;0 0 0;1 0 0];         % 6 outputs, 3 disturbance states on θ w q
% Cd = [1 0 0;0 1 0;0 0 1;0 0 0;0 0 0;0 0 0];   
% Dd = zeros(6,3);                 % no direct feedthrough
% 
% Gd = ss(Ad, Bd, Cd, Dd, Ts);
% Gd.StateName  = {'dz','dphi','dtheta'};
% Gd.InputName = {'wzeta','wphi','wtheta'};
% Gd.OutputName = {'z','phi','theta','w','p','q'};

mpc_O = generate_mpc(cont_plant, [Utrim{5}(1:3)']);
setoutdist(mpc_O, 'integrators');

%% Input Disturbance
cont_plant_I = ss(reduced_model.A, [reduced_model.B reduced_model.B], reduced_model.C, [reduced_model.D reduced_model.D]);
cont_plant_I.InputGroup.ManipulatedVariables = [1 2 3];
cont_plant_I.InputGroup.UnmeasuredDisturbances = [4 5 6];
cont_plant_I.OutputGroup.MeasuredOutputs = [1 2 3 4 5 6];


cont_plant_I.InputName = {'uFS','uFP','uA', 'ud1','ud2','ud3'};
cont_plant_I.OutputName = {'z','phi','theta', 'w', 'p', 'q'};

mpc_I = generate_mpc(cont_plant_I, [Utrim{5}(1:3)';0;0;0], ...
    'OutDist', tf(zeros(6,1)), ...
    'Ts', Ts, ...
    'PredictionHorizon',PredictionHorizon,...
    'ControlHorizon',ControlHorizon);
Gd_in = mpc_I.getindist; % faster response to changes in input dynamics
Gd_in.B = 0.4*Gd_in.B;
setindist(mpc_I,'model',Gd_in)

%% Input + Output Disturbance

mpc_IO = generate_mpc(cont_plant_I, [Utrim{5}(1:3)';0;0;0], ...
    'Ts', Ts, ...
    'PredictionHorizon',PredictionHorizon,...
    'ControlHorizon',ControlHorizon); 

% i you let matlab decide on outdist it actually chooses theta, p, q
% % Disturbance model: d(k+1) = d(k) + w(k)
Ad = eye(3);
Bd = 0.1*eye(3);                 % no exogenous inputs
Cd = [0 0 0;0 0 0;1 0 0;0 0 0;0 1 0;0 0 1];         % 6 outputs, 3 disturbance states on θ w q
Dd = zeros(6,3);                 % no direct feedthrough
Gd_out = ss(Ad, Bd, Cd, Dd, Ts);
Gd_out.StateName  = {'dtheta','dw','dq'};
Gd_out.InputName = {'wtheta','ww','wq'};
Gd_out.OutputName = {'z','phi','theta','w','p','q'};

Gd_in = mpc_IO.getindist; 
Gd_in.B = 0.4*Gd_in.B;
setindist(mpc_IO,'model', Gd_in)
setoutdist(mpc_IO, 'model', Gd_out)

%% Input - Delay - P

% A: [n x n], B: [n x m], C: [p x n]
n = size(reduced_model.A,1); m = size(reduced_model.B,2); p = size(reduced_model.C,1);
tau = 0.1;
Adelpred = [reduced_model.A, reduced_model.B; zeros(m,n), -1/tau*eye(m)];
Bdelpred = [zeros(n,m);(1/tau)*eye(m)];
%B_d = [reduced_model.B;zeros(m,m)];
Cdelpred = [reduced_model.C, zeros(p, m)];
D_u = zeros(p, m);
D_d = zeros(p, m);

cont_plant_IDP = ss(Adelpred, [Bdelpred Bdelpred], Cdelpred, [D_u D_d]);

cont_plant_IDP.InputGroup.ManipulatedVariables = [1 2 3];
cont_plant_IDP.InputGroup.UnmeasuredDisturbances = [4 5 6];
cont_plant_IDP.OutputGroup.MeasuredOutputs = [1 2 3 4 5 6];

mpc_ID = generate_mpc(cont_plant_IDP, [Utrim{5}(1:3)';0;0;0], ...
    'OutDist', tf(zeros(6,1)), ...
    'Ts', Ts, ...
    'PredictionHorizon',PredictionHorizon,...
    'ControlHorizon',ControlHorizon);
Gd_in = mpc_ID.getindist; 
Gd_in.B = 1.5*Gd_in.B;
setindist(mpc_ID,'model',Gd_in)


%% Adaptive LPV-MPC
reduced_dsys_array = cell(1,length(reduced_sys_array));
U_offset = zeros(6, length(reduced_sys_array));
Y_offset = zeros(6, length(reduced_sys_array));

tau = 0.1;

for i=1:length(reduced_sys_array)

    Adelpred = [reduced_sys_array{i}.A, reduced_sys_array{i}.B; zeros(m,n), -1/tau*eye(m)];
    Bdelpred = [zeros(n,m);(1/tau)*eye(m)];
    Cdelpred = [reduced_sys_array{i}.C, zeros(p, m)];
    D_u = zeros(p, m);
    D_d = zeros(p, m);

    placeholder = ss(Adelpred, [Bdelpred Bdelpred], Cdelpred, [D_u D_d]);

    placeholder.InputGroup.ManipulatedVariables = [1 2 3];
    placeholder.InputGroup.UnmeasuredDisturbances = [4 5 6];
    placeholder.OutputGroup.MeasuredOutputs = [1 2 3 4 5 6];

    placeholder.InputName = {'uFS','uFP','uA', 'ud1','ud2','ud3'};
    placeholder.OutputName = {'z','phi','theta', 'w', 'p', 'q'};
    
    
    U_offset(:, i) = [transpose(Utrim{i}(1:3));0;0;0];
    Y_offset(:, i) = [-0.3;0;0;0;0;0];
    reduced_dsys_array{i} = c2d(placeholder, Ts);
end

dLTI_array_IDP = cat(3, reduced_dsys_array{:});
nominal_idx = 5;

%% nominal system %%
lpv_mpc_IDP = generate_mpc(reduced_dsys_array{nominal_idx}, U_offset(:,nominal_idx), ...
    'OutDist', tf(zeros(6,1)), ...
    'Ts', Ts, ...
    'PredictionHorizon',PredictionHorizon,...
    'ControlHorizon',ControlHorizon);

Gd_in = lpv_mpc_IDP.getindist; 
Gd_in.B = 1.5*Gd_in.B;
setindist(lpv_mpc_IDP,'model',Gd_in)

%% Adaptive LPV-MPC (I only)
reduced_dsys_array = cell(1,length(reduced_sys_array));
U_offset = zeros(6, length(reduced_sys_array));
X_offset = zeros(6, length(reduced_sys_array));

for i=1:length(reduced_sys_array)
    placeholder = ss(reduced_sys_array{i}.A, [reduced_sys_array{i}.B reduced_sys_array{i}.B], reduced_sys_array{i}.C, [reduced_sys_array{i}.D reduced_sys_array{i}.D]);

    placeholder.InputGroup.ManipulatedVariables = [1 2 3];
    placeholder.InputGroup.UnmeasuredDisturbances = [4 5 6];
    placeholder.OutputGroup.MeasuredOutputs = [1 2 3 4 5 6];

    placeholder.InputName = {'uFS','uFP','uA', 'ud1','ud2','ud3'};
    placeholder.OutputName = {'z','phi','theta', 'w', 'p', 'q'};


    U_offset(:, i) = [transpose(Utrim{i}(1:3));0;0;0];
    X_offset(:, i) = [-0.3;0;0;0;0;0];
    reduced_dsys_array{i} = c2d(placeholder, Ts);
end

dLTI_array_IP = cat(3, reduced_dsys_array{:});
nominal_idx = 5;

%% nominal system %%
lpv_mpc_IP = generate_mpc(reduced_dsys_array{nominal_idx}, U_offset(:,nominal_idx), ...
    'OutDist', tf(zeros(6,1)), ...
    'Ts', Ts, ...
    'PredictionHorizon',PredictionHorizon,...
    'ControlHorizon',ControlHorizon);

Gd_in = lpv_mpc_IP.getindist; 
Gd_in.B = 0.4*Gd_in.B;
setindist(lpv_mpc_IP,'model',Gd_in)



%% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'on';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';



%disturbance_model = getoutdist(mpc1);
%setoutdist(mpc1,'model',disturbance_model*0.5);
%setoutdist(mpc1,'model',ss(ones(6,1)));




