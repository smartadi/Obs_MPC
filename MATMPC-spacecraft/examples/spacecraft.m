%------------------------------------------%

% spacecraft

%------------------------------------------%


%% Dimensions

nx=13;  % No. of differential states (pos:3, quat:4, v:3, omega:3)
nu=12;   % No. of controls (thrusters)
nz=0;   % No. of algebraic states
ny=25;  % No. of outputs (states + controls)
nyN=13; % No. of outputs at the terminal point (states)
np=0;   % No. of model parameters
nc=0;   % No. of general inequality constraints
ncN=0;  % No. of general inequality constraints
nbx = 0; % No. of bounds on states)
nbu = 12; % No. of bounds on controls (all)

% state and control bounds
nbx_idx = [];  % indexs of states which are bounded
nbu_idx = 1:12;  % indexs of controls which are bounded

%% create variables

import casadi.*

states   = SX.sym('states',nx,1);   % differential states
controls = SX.sym('controls',nu,1); % control input
alg      = SX.sym('alg',nz,1);      % algebraic states
params   = SX.sym('paras',np,1);    % parameters
refs     = SX.sym('refs',ny,1);     % references of the first N stages
refN     = SX.sym('refs',nyN,1);    % reference of the last stage
Q        = SX.sym('Q',ny,1);        % weighting matrix of the first N stages
QN       = SX.sym('QN',nyN,1);      % weighting matrix of the last stage
aux      = SX.sym('aux',ny,1);      % auxilary variable
auxN     = SX.sym('auxN',nyN,1);    % auxilary variable


%% Dynamics

p           =states(1:3);
q           =states(4:7);
v           =states(8:10);
omega       =states(11:13);
u           =controls(1:12);

% quaternion conversions
eta = q(1);
epsilon = q(2:4);
epsilon_x = [0, -epsilon(3), epsilon(2); epsilon(3), 0, -epsilon(1); -epsilon(2), epsilon(1), 0]; 
Rq = eye(3) + 2*eta*epsilon_x + 2*epsilon_x^2;  % rotation matrix, body to world
Mq = [eta, -epsilon'; epsilon, eta*eye(3)+epsilon_x]; % quaternion composition matrix

% model parameters
mu = 3.816e14; % Gravitational parameter of earth
%initialize mean motion of earth orbit of 6378km
n = sqrt(mu./(6738000.^3));
% control gain
thruster_force = 10;
% mass and inertia parameters
I = eye(3);
mass = 1;
% trusters positions
thruster_positions = [
    0.5,0,0
    0.5,0,0
    -0.5,0,0
    -0.5,0,0
    0,0.5,0
    0,0.5,0
    0,-0.5,0
    0,-0.5,0
    0,0,0.5
    0,0,0.5
    0,0,-0.5
    0,0,-0.5
    ];
%Define thruster force vectors, with them pointing 45 degrees from the surface normal.
thruster_force_vectors = -1*[
    0.707, 0.707, 0
    0.707, -0.707, 0
    -0.707, 0.707, 0
    -0.707, -0.707, 0
    0, 0.707, 0.707
    0, 0.707, -0.707
    0, -0.707, 0.707
    0, -0.707, -0.707
    0.707, 0, 0.707
    -0.707, 0, 0.707
    0.707, 0, -0.707
    -0.707, 0, -0.707
    ];

v_dot = [
    3*n^2 0 0
    0 0 0 
    0 0 -n^2
    ] * p + [
    0 2*n 0
    -2*n 0 0
    0 0 0] * v;

control_input = thruster_force*u;

control_torques = cross(thruster_positions,thruster_force_vectors).'*control_input;

omega_dot =  control_torques;

body_frame_forces = thruster_force_vectors.' * control_input;

v_dot = v_dot + Rq * body_frame_forces ./ mass;

% explicit ODE RHS
x_dot = [
    v
    1/2*Mq*[0;omega]
    v_dot
    omega_dot
    ];

% algebraic function
z_fun = [];

% implicit ODE: impl_f = 0
xdot = SX.sym('xdot',nx,1);
impl_f = xdot - x_dot;
     
%% Objectives and constraints

%q_err = Mq*[q_ref(1); -q_ref(2:4)];


% inner objectives
h = [
    p
    %q_err(2:end)
    q
    v
    omega
    u
    ];
hN = h(1:nyN);

% outer objectives
obji = 0.5*(h-refs)'*diag(Q)*(h-refs);
objN = 0.5*(hN-refN)'*diag(QN)*(hN-refN);

obji_GGN = 0.5*(aux-refs)'*(aux-refs);
objN_GGN = 0.5*(auxN-refN)'*(auxN-refN);

% general inequality path constraints
general_con = [];
general_con_N = [];

%% NMPC discretizing time length [s]

Ts_st = 0.1; % shooting interval time

% %% Model Parameters exportation
% modelParameters = struct(...
%     'name', 'spacecraft', ...
%     'g', g, ...
%     'mR', mR, ...
%     'JR', JR, ...
%     'c_t_raw', c_t_raw, ...
%     'c_tau_raw', c_tau_raw, ...
%     'rotor_max_vel', rotor_max_vel, ...
%     'c_direction', c_direction, ...
%     'p_mot', p_mot, ...
%     'theta', theta,...
%     'beta', beta, ...
%     'delta', delta, ...
%     'l', l);
% save('modelParameters.mat', 'modelParameters');
