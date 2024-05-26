%------------------------------------------%

% spacecraft

%------------------------------------------%


%% Dimensions

nx=6;  % No. of differential states (pos:3, v:3)
nu=3;   % No. of controls (linear acceleration)
nz=0;   % No. of algebraic states
ny=6+3;  % No. of outputs (states + controls)
nyN=6; % No. of outputs at the terminal point (states)
np=0;   % No. of model parameters
nc=0;   % No. of general inequality constraints
ncN=0;  % No. of general inequality constraints
nbx = 0; % No. of bounds on states)
nbu = 3; % No. of bounds on controls (all)

% state and control bounds
nbx_idx = [];  % indexs of states which are bounded
nbu_idx = [1,2,3];  % indexs of controls which are bounded

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

x           =states(1:6);
u           =controls(1:3);


% model parameters
A = [0,0,0,1,0,0;
     0,0,0,0,1,0;
     0,0,0,0,0,1;
     0,0,0,0,0,0;
     0,0,0,0,0,0;
     0,0,0,0,0,0];
B = [0 0 0;
     0 0 0;
     0 0 0;
     1 0 0;
     0 1 0;
     0 0 1];

C = [1,0,0,0,0,0;
     0,1,0,0,0,0;
     0,0,1,0,0,0];

% explicit ODE RHS
x_dot = A*x + B*u;

% algebraic function
z_fun = [];

% implicit ODE: impl_f = 0
xdot = SX.sym('xdot',nx,1);
impl_f = xdot - x_dot;
     
%% Objectives and constraints

% inner objectives
h = [
    x
    u
    ];
hN = h(1:nyN);

% observablitiy objectives
[y,k,l] = phi_set_L(C*x);

h = [
    l
    k
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



function [y,k,L] = phi_set_L(x)   
    
    k = 0.1*(norm(x-[0.5;0.5;0.5])^2 + 0.1);
    % r = 2*(0.5-rand(size(x)));
    r = (0.5-rand(size(x)));
    if norm(r)>= k
        r = k*r/norm(r);
    end
    y = x + r;
    L = 0.1*norm(x-[0.5;0.5;0.5]);


end