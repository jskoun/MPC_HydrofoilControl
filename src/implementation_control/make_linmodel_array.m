V_min = 5;
V_max = 15;
N = 21;

V_vals = linspace(V_min, V_max, N);  % Sample speeds
sys_array = cell(1, N);              % Store linearized models
A_all = cell(1, N); B_all = cell(1, N); C_all = cell(1, N); D_all = cell(1, N);
Utrim = cell(1,N);

Ts = 0.1;

for i = 1:N
    V = V_vals(i);
    
    % Example: construct nonlinear model with current V
    % or use Simulink's linearize command
    operpoint = find_operpoint(model = "simplified_sim_for_linearize", usted=V, u4=2500);
    linearized = find_linear_model(operpoint = operpoint);
    lmod = ss(linearized.A, linearized.B, eye(size(linearized.A)), 0);
    
    A_all{i} = lmod.A; B_all{i} = lmod.B; C_all{i} = lmod.C; D_all{i} = lmod.D;
    Utrim{i} = [operpoint.Inputs(1).u,operpoint.Inputs(2).u,operpoint.Inputs(3).u,operpoint.Inputs(4).u];
    sys_array{i} = lmod;
end


reduced_sys_array = cell(1, N);
for i=1:length(V_vals)
    % Exclude the 4th row and column
    A_reduced = sys_array{i}.A([1:3, 5:7], [1:3, 5:7]);
    
    % If you also need to adjust the vector B, exclude the 4th element
    B_reduced = sys_array{i}.B([1:3, 5:7],[1:3]);
    
    reduced_sys_array{i} = ss(A_reduced, B_reduced, eye(size(A_reduced)), 0);


    
end

LTI_array = cat(3, reduced_sys_array{:});

%% Get nominal model 
nominal_idx = 5;

linmod = sys_array{nominal_idx};


% clear numerical error
linmod.A(6,4) = 0;
linmod.A(6,5) = 0;
linmod.A(6,7) = 0;


% Exclude the 4th row and column ( u DOF )
A_reduced = linmod.A([1:3, 5:7], [1:3, 5:7]);

% If you also need to adjust the vector B, exclude the 4th element
B_reduced = linmod.B([1:3, 5:7],[1:3]);

reduced_model = ss(A_reduced, B_reduced, eye(size(A_reduced)), 0);