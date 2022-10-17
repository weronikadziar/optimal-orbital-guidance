function model = model_translation_3D_convex_constraints()

import casadi.*

% Model name
model_name = 'translation_3D_convex';

% System parameters
nx = 6;                    % number of states
nu = 3;                    % number of inputs

% Symbolic variables
sym_x = SX.sym('x', [nx 1]);     % chaser relative pos and vel in LVLH frame
sym_u = SX.sym('u', [nu 1]);     % body forces in LVLH frame
sym_xdot = SX.sym('xdot', nx, 1);

% System dynamics
dyn_type = 'discrete';
mu = 3.986e14;       % Earth standard gravitational parameter
r0 = 800e3;          % orbit radius
n = sqrt(mu/r0^3);   % mean motion of target satellite
f_thr = 1;           % thrust magnitude
m = 1;               % chaser mass
dt = 1;              % discretization time step
Ad = [4-3*cos(n*dt), 0, 0, 1/n*sin(n*dt), 2/n*(1-cos(n*dt)), 0;
      6*(sin(n*dt)-n*dt), 1, 0, -2/n*(1-cos(n*dt)), 1/n*(4*sin(n*dt)-3*n*dt), 0;
      0, 0, cos(n*dt), 0, 0, 1/n*sin(n*dt);
      3*n*sin(n*dt), 0, 0, cos(n*dt), 2*sin(n*dt), 0;
      -6*n*(1-cos(n*dt)), 0, 0, -2*sin(n*dt), 4*cos(n*dt)-3, 0;
      0, 0, -n*sin(n*dt), 0, 0, cos(n*dt)];
Bd = f_thr/m*[1/n^2*(1-cos(n*dt)), 2/n^2*(n*dt-sin(n*dt)), 0;
              -2/n^2*(n*dt-sin(n*dt)), 4/n^2*(1-cos(n*dt))-3/2*(dt)^2, 0;
              0, 0, 1/n^2*(1-cos(n*dt));
              1/n*sin(n*dt), 2/n*(1-cos(n*dt)), 0;
              -2/n*(1-cos(n*dt)), 4/n*sin(n*dt)-3*dt, 0;
              0, 0, 1/n*sin(n*dt)];
dyn_expr_phi = Ad*sym_x + Bd*sym_u;

% Constraints
constr_Jbu = eye(nu);       % constrain control actions
constr_lbu = -ones(nu,1);
constr_ubu = ones(nu,1);

% Cost
cost_expr_y = sym_u;        % minimize control inputs along the way
cost_W = eye(nu);
cost_W_e = 100*eye(nx);     % minimize deviation from desired end point
cost_Vx_e = eye(nx);

% Time parameters
dt = 1;
T = 100;
N = 100;

% Populate structure
model.model_name = model_name;
model.nx = nx;
model.nu = nu;
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.dyn_type = dyn_type;
model.dyn_expr_phi = dyn_expr_phi;
model.constr_Jbu = constr_Jbu;
model.constr_lbu = constr_lbu;
model.constr_ubu = constr_ubu;
model.cost_expr_y = cost_expr_y;
model.cost_W = cost_W;
model.cost_W_e = cost_W_e;
model.cost_Vx_e = cost_Vx_e;
model.dt = dt;
model.T = T;
model.N = N;

end