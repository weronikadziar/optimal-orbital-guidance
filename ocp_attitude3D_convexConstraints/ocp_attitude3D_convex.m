function ocp = ocp_attitude3D_convex()

% DESCRIPTION
% Creates the Acados OCP object.

import casadi.*

%% Set up parameters

% System parameters
nx = 12;                    % number of states
nu = 3;                     % number of inputs

% Symbolic variables
sym_x = SX.sym('x', [nx 1]);     % chaser relative pos and vel in LVLH frame
sym_u = SX.sym('u', [nu 1]);     % body forces in LVLH frame
sym_xdot = SX.sym('xdot', nx, 1);

% System dynamics
dyn_type = 'explicit';
m = 1;                      % chaser mass
a = 1;                      % chaser length
J = eye(3)*m*a^2/6;         % moment of inertia of a cube
rho = 0;
R = reshape(sym_x(1:9),3,3);
w = sym_x(10:12);
A = rho/4*(eye(3)-R'*R);
omega = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
Rdot = R*(A+omega);
wdot = J\(sym_u-cross(w, J*w));
dyn_expr_f = vertcat(Rdot(:),wdot);

% Constraints: control actions
constr_Jbu = eye(nu);
constr_lbu = -ones(nu,1);
constr_ubu = ones(nu,1);

% Constraints: angular velocities
constr_expr_h = sym_x(10:12);
constr_lh = -0.1*ones(3,1);
constr_uh = 0.1*ones(3,1);

% Cost: keep the control actions low
cost_expr_y = sym_u;
cost_W = eye(nu);

% Cost: minimize deviation from desired end point
cost_W_e = 100*eye(nx);
cost_Vx_e = eye(nx);

% Time parameters
T = 200;
N = 100;

%% Create an OCP object

% Solver
nlp_solver = 'sqp';
qp_solver = 'partial_condensing_hpipm';   
qp_solver_cond_N = 5;

% Create the object
ocp_model = acados_ocp_model();
model_name = strcat('ocp',num2str(randi(100)));
ocp_model.set('name', model_name);
ocp_model.set('T', T);

% Symbolics
ocp_model.set('sym_x', sym_x);
ocp_model.set('sym_u', sym_u);
ocp_model.set('sym_xdot', sym_xdot);

% Cost
ocp_model.set('cost_type', 'nonlinear_ls');
ocp_model.set('cost_expr_y', cost_expr_y); 
ocp_model.set('cost_W', cost_W);
ocp_model.set('cost_type_e', 'linear_ls');
ocp_model.set('cost_Vx_e', cost_Vx_e);
ocp_model.set('cost_W_e', cost_W_e);

% Dynamics
ocp_model.set('dyn_type', dyn_type);
ocp_model.set('dyn_expr_f', dyn_expr_f);

% Constraints
ocp_model.set('constr_x0', zeros(nx,1));
ocp_model.set('constr_Jbu', constr_Jbu);
ocp_model.set('constr_lbu', constr_lbu);
ocp_model.set('constr_ubu', constr_ubu);
ocp_model.set('constr_expr_h', constr_expr_h);
ocp_model.set('constr_lh', constr_lh);
ocp_model.set('constr_uh', constr_uh);

% Acados ocp set opts
ocp_opts = acados_ocp_opts();
ocp_opts.set('param_scheme_N', N);
ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('sim_method', 'erk');
ocp_opts.set('qp_solver', qp_solver);
ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);

% Create ocp solver
ocp = acados_ocp(ocp_model, ocp_opts);

end