%% Simulation Data %%
% Here you can tweak the simulation knobs, affecting the physical model,
% sensor noise, filter specs etc

mass = 215;
g = 9.81;
rho=1024.8;


SMAT = @(x) [0 -x(3) x(2);
            x(3) 0 -x(1);
            -x(2) x(1) 0];

%% VESSEL MODEL %%
%CG
xg = 0;
yg = 0;
zg = 0;

r_g = [xg, yg, zg];

%Boat Numerics
[Ixx, Iyy, Izz, Ixy, Ixz, Iyz] = deal(31, 503, 1, 0, 0, 0);
% don't set moments of inertia to 0 if no known value
% it will cause inf error

InertiaTensor = [Ixx, -Ixy, -Ixz;
                -Ixy, Iyy, -Iyz;
                -Ixz, -Iyz, Izz];
%FOSSEN MATRICES
M_RB = [mass*eye(3), -mass*SMAT(r_g);
       mass*SMAT(r_g), InertiaTensor-mass*SMAT(r_g)^2];

M_RB_1 = M_RB^(-1);

% this does the same as the above again ! Useful for perturbed
dimensionalize_model(mass, r_g, InertiaTensor)

%CRB is a function of the body-fixed velocity vector - will be calculated
%in-simulation


% Foil Placement
dxf = 1.1;
dyf = 0.62;
dzf = 0.75;

dxa = 2.45;
dza = 0.96;

dya = 0;

foilFP_ARM = [dxf, -dyf, dzf];
foilFS_ARM = [dxf, dyf, dzf];
foilA_ARM = [-dxa, dya, dza];

%Propeller Placement

xpr = -dxa; %AFT
ypr = 0;
zpr = dza;

PropPlacement = [xpr, ypr, zpr];
%optional
propwake = 0.05;

% prop data
propD = 0.3;

prop_kt_data = readmatrix('hydro_data/prop_kt_data.csv');
prop_kq_data = readmatrix('hydro_data/prop_kq_data.csv');

propKtp = polyfit(prop_kt_data(:,1), prop_kt_data(:,2), 2);
propKqp = polyfit(prop_kq_data(:,1), prop_kq_data(:,2), 2);


% foil data
AfoilFS=0.048;
AfoilFP=0.048;

tFS = 0.12*0.08;
tFP = 0.12*0.08;
tsF = 0.12*0.08;

tA = 0.12*0.08;
tsA = 0.12*0.1;


spanFP = 0.76;
spanFS = 0.76;

AfoilA=0.046;
spanA = 0.76;

csF = 0.08; % strut chord
csA = 0.1;




% Free Surface Effects
MAC_FS = 0.08;
MAC_FP = 0.08;
MAC_A = 0.08;

% unsteady lift:
usted = 7;
wagC1 = 0.165;
wagC2 = 0.335;
wage1 = 0.0455*(usted/(MAC_FP/2));
wage2 = 0.3*(usted/(MAC_FP/2));


% xflr5 output data (csv)
data0012 = readmatrix('hydro_data/0012_foil_Re500.csv');
data63412wing = readmatrix('hydro_data/63412_elwing_7ms.csv');
alpha0012 = data0012(:,1);
cl0012 = data0012(:,2);
cd0012 = data0012(:,3);

alpha63412wing = data63412wing(:,1);
cl63412wing = data63412wing(:,3);
cd63412wing = data63412wing(:,6);

cdi63412wing = data63412wing(:,4); % important for free surface effects
cdv63412wing = data63412wing(:,5); %cd = cdv+cdi

cm63412wing = data63412wing(:,9); % cm


% lateral damping:
%FOIL - STRUT
% foil: Ldamp = -qCLVp \int y^2c(y) dy
% strut: Ldamp = -qCLVp \int z^2c dz = -0.5 rho V p c (zs^3-zwl^3)/3

fsint = @(y) y.^2.*0.08.*sqrt(1-((y-(foilFS_ARM(2)))./0.4).^2);
LFSd = integral(fsint, foilFS_ARM(2)-0.4, foilFS_ARM(2)+0.4);
%FP
fpint = @(y) y.^2.*0.08.*sqrt(1-((y-foilFP_ARM(2))./0.4).^2);
LFPd = integral(fpint, foilFP_ARM(2)-0.4, foilFP_ARM(2)+0.4);

%A
LAd = 0.08*pi*(0.8)^3/64;




% scale %
% max refers to 
% maximum allowed error (for states) and 
% maximum allowed deviation (for inputs)

max_z = 0.1;
max_phi = deg2rad(5);
max_theta = deg2rad(5);
max_w = 0.5;
max_p = deg2rad(30);
max_q = deg2rad(30);

max_deflection = deg2rad(10);

wave_data=struct('amplitude',0,'omega',0,'wavenumber',0,'phase',0,'direction',0);
Tsim=10;
[setpoints, wave_data] = sim_setup( ...
    'sim_params',struct('Tsim',Tsim), ...
    'ref_defs', {}, ...
    'wave_data',wave_data); % wave_data dont matter if you set random sea

setpoint_reference = setpoints.reference;
setpoint_signals = setpoints.preview;

%scale states%
Dstate = diag([max_z, max_phi, max_theta, max_w, max_p, max_q]);
%scale inputs%
Dinput = diag([max_deflection,max_deflection,max_deflection]);
%%%%%%%%%



%% SENSOR DATA %%
sonar_position = [dxf+0.5; 0; -0.1];
deltat = 1/100; % kalman filter sampling rate

var_sonar = 0.002^2;

psd_acc = (9.81*60*1e-6)^2;
var_gps = 0.05^2;

var_angle = deg2rad(0.2)^2;
psd_gyro = (1.222*1e-4)^2;

seed_sonar = 1234;
seed_accX = 1324;
seed_accZ = 1432;
seed_gps = 4321;

seed_gyroX = 243251;
seed_gyroY = 3124234;
seed_pitch = 7568;
seed_roll = 35642;
seed_waves = 2123;

cutoff_freq_c = 2;
sample_fs = 20;
cutoff_freq_d = cutoff_freq_c/(sample_fs/2);
[bA, bB, bC, bD] = butter(2, cutoff_freq_d, 'low');
butter_nth = ss(bA, bB, bC, bD);

maxDeflect = deg2rad(10);