function dimensionalize_model(mass, r_g, InertiaTensor)
    % dimensionalize_model: computes and pushes rigid-body matrices to base workspace
    %
    % Usage:
    %   dimensionalize_model()                        % uses default parameters
    %   dimensionalize_model(mass, r_g, InertiaTensor) % uses custom parameters

    % Default values
    if nargin < 1 || isempty(mass)
        mass = 215;
    end
    if nargin < 2 || isempty(r_g)
        r_g = [0; 0; 0];
    end
    if nargin < 3 || isempty(InertiaTensor)
        % Example inertia tensor (kg·m²) — adjust to your model
        InertiaTensor = diag([100, 120, 80]);
    end

    % Assign to base workspace
    assignin('base', 'mass', mass);
    assignin('base', 'r_g', r_g);
    assignin('base', 'InertiaTensor', InertiaTensor);

    % Skew-symmetric matrix function
    SMAT = @(x) [0, -x(3),  x(2);
                 x(3),  0, -x(1);
                -x(2),  x(1),  0];

    % Adjusted inertia about body frame origin
    InertiaTensor_b = InertiaTensor - mass * SMAT(r_g)^2;

    % Compute rigid-body mass matrix
    M_RB = [mass * eye(3),     -mass * SMAT(r_g);
            mass * SMAT(r_g),   InertiaTensor_b];
    M_RB_1 = inv(M_RB);

    % Push to base workspace
    assignin('base', 'M_RB', M_RB);
    assignin('base', 'M_RB_1', M_RB_1);
end