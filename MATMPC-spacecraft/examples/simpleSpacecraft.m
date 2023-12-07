%------------------------------------------%

% spacecraft

%------------------------------------------%


%% Dimensions

nx=6;  % No. of differential states (pos:3, v:3)
nu=3;   % No. of controls (linear acceleration)
nz=0;   % No. of algebraic states
ny=5+3;  % No. of outputs (states + controls)
nyN=5; % No. of outputs at the terminal point (states)
np=3;   % No. of model parameters
nc=1;   % No. of general inequality constraints
ncN=1;  % No. of general inequality constraints
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

p           =states(1:3);
v           =states(4:6);
u           =controls(1:3);




% model parameters
%mu = 3.816e14; % Gravitational parameter of earth
mu = 3.986e14; % Gravitational parameter of earth
%initialize mean motion of earth orbit of 6378km
a = 2e6;
n = sqrt(mu./(a.^3));

v_dot = [
    3*n^2 0 0
    0 0 0 
    0 0 -n^2
    ] * p + [
    0 2*n 0
    -2*n 0 0
    0 0 0] * v;

v_dot = v_dot + u;

% explicit ODE RHS
x_dot = [
    v
    v_dot
    ];

% algebraic function
z_fun = [];

% implicit ODE: impl_f = 0
xdot = SX.sym('xdot',nx,1);
impl_f = xdot - x_dot;
     
%% Objectives and constraints

d = p'*p;
prefDir = params(1:3);
p_normalized = p/norm(p);
cosWrtPrefDir = acos(p_normalized.'*prefDir);
%cosWrtPrefDir = 1/(p_normalized.'*prefDir);
% Current version
%sectorAmpl = norm(1-cosWrtPrefDir)^2;

% This one works right now
% sectorAmpl = norm(cosWrtPrefDir)^2;

j=0;



% Modified Observability 
sectorAmpl = 0;


% epsilon should evolve as well
% working version
% eps = 0.1;
% for i = 1:3
%     P = [p];
%     P(i) = P(i) + eps;
%     
%     p_normalized = P/norm(P);
%     cosWrtPrefDir = acos(p_normalized.'*prefDir);
%     sectorAmpl = sectorAmpl + norm(cosWrtPrefDir)^2;
%     
%     
%     
%     
%     P(i) = P(i) - eps;
%     
%     p_normalized = P/norm(P);
%     cosWrtPrefDir = acos(p_normalized.'*prefDir);
%     sectorAmpl = sectorAmpl + norm(cosWrtPrefDir)^2;
% end



eps = 0.1;
for i = 1:3
    P = [p];
    P(i) = P(i) + eps;
    
    p_normalized = P/norm(P);
    cosWrtPrefDir = acos(p_normalized.'*prefDir);
    sectorAmpl = sectorAmpl + norm(cosWrtPrefDir)^2;
    
    
    
    Pp = [p];
    Pp(i) = Pp(i) - eps;
    
    q_normalized = Pp/norm(Pp);
    cosWrtPrefDir = acos(q_normalized.'*prefDir);
    sectorAmpl = sectorAmpl + norm(cosWrtPrefDir)^2;
end

% sectorAmpl = 0;



% inner objectives
h = [
    v
    d
    sectorAmpl
    u
    ];
hN = h(1:nyN);

% outer objectives
obji = 0.5*(h-refs)'*diag(Q)*(h-refs);
objN = 0.5*(hN-refN)'*diag(QN)*(hN-refN);

obji_GGN = 0.5*(aux-refs)'*(aux-refs);
objN_GGN = 0.5*(auxN-refN)'*(auxN-refN);

% general inequality path constraints
general_con = [norm(p)-10];
general_con_N = [norm(p)-10];

%% NMPC discretizing time length [s]

Ts_st = 1; % shooting interval time

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
