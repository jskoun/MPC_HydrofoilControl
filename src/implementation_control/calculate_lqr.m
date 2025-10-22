%reduced_model_pos = ss(A_reduced, B_reduced, eye(3), 0);
%LQR
weights_Q_lqr = [1 1 1 0 1 1];
weights_R_lqr = [1 1 1];
Q_lqr = diag(weights_Q_lqr);
R_lqr = diag(weights_R_lqr);
[Klqr, P, E] = lqr(reduced_model, Q_lqr, R_lqr);

[Klqr_scaled, P, E] = lqr(scaled_model, Q_lqr, Dinput.'*R_lqr*Dinput);

%LQRi

%Augmented System:
Aaug = [A_reduced zeros(6,3); eye(3) zeros(3,6)];
Baug = [B_reduced;zeros(3,3)];


weights_Q_lqi = ([0.1, maxDeflect, maxDeflect, 0.5, 0.5, 0.5, 0.1, maxDeflect, maxDeflect].^(-1).*[0.2 0.15 0.25 0.2 0.2 0.2 0.2 0.2 0.4]).^2; %[4 1 2 0.5 0.5 0.5 4 4 4];
weights_R_lqi = ([maxDeflect, maxDeflect, maxDeflect].^(-1).*[0.2, 0.2, 0.2]).^2;
Q_lqi = diag(weights_Q_lqi);
R_lqi = diag(weights_R_lqi);

weights_Q_lqi_sc = [5 2 3 0.5 0.5 0.5 2 2 2];
weights_R_lqi_sc = [0.1 0.1 0.1];
Q_lqi_sc = diag(weights_Q_lqi_sc);
R_lqi_sc = diag(weights_R_lqi_sc);


[Klqi, P, E] = lqr(Aaug, Baug, Q_lqi, R_lqi);


sysaug = ss(Aaug, Baug, eye(9), 0);


% augment the already scaled model
%Augmented System:
Ascaled_aug = [scaled_model.A zeros(6,3); eye(3) zeros(3,6)];
Bscaled_aug = [scaled_model.B;zeros(3,3)];


[Klqi_scaled, P, E] = lqr(Ascaled_aug, Bscaled_aug, Q_lqi_sc, R_lqi_sc);

sysaug_scaled = ss(Ascaled_aug, Bscaled_aug, eye(9), 0);

Dstate_aug = [Dstate zeros(6,3);zeros(3,6) Dstate(1:3,1:3)];

Klqi_dim = Dinput*Klqi_scaled*Dstate_aug^(-1);

%% discrete %%

sys_d = c2d(sysaug, 0.1);
Klqid = dlqr(sys_d.A, sys_d.B, Q_lqi, R_lqi);   
%% %%

%% discrete scaled %%

sys_d = c2d(sysaug_scaled, 0.1);
Klqid_scaled = dlqr(sys_d.A, sys_d.B, Q_lqi_sc, R_lqi_sc);

Kintegral = Klqid(:,7:9);
Ka = -pinv(Kintegral)*2;
act_lims = [-maxDeflect, maxDeflect;-maxDeflect, maxDeflect;-maxDeflect, maxDeflect];