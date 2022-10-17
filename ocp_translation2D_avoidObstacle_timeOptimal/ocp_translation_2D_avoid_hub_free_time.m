function ocp = ocp_translation_2D_avoid_hub_free_time(params)

% Import Acados and Casadi
addpath("C:\Users\Weronika\acados\examples\acados_matlab_octave")
run("acados_env_variables_windows.m")
import casadi.*

%% Set up parameters

% System parameters
nx = 5;                    % number of states - positions and valocities + time factor tau
nu = 2;                    % number of inputs

% Time parameters
T = 1;
N = 100;

% Symbolic variables
sym_x = SX.sym('x', [nx 1]);     % chaser relative pos and vel in LVLH frame
sym_u = SX.sym('u', [nu 1]);     % body forces in LVLH frame
sym_xdot = SX.sym('xdot', nx, 1);

% System dynamics
dyn_type = 'explicit';
mu = 3.986e14;       % Earth standard gravitational parameter
r0 = 800e3;          % orbit radius
n = sqrt(mu/r0^3);   % mean motion of target satellite
m = params.mass;               % satellite mass
dyn_expr_f = sym_x(5)*[sym_x(3:4); 3*n^2*sym_x(1)+2*n*sym_x(4)+sym_u(1)/m; -2*n*sym_x(3)+sym_u(2)/m; 0];


% Constraints
constr_Jbu = eye(nu);       % constrain control actions
constr_lbu = -params.max_thrust*ones(nu,1);
constr_ubu = params.max_thrust*ones(nu,1);
a = params.min_to_target;
Q = diag([1/a, 1/a]);       % don't hit the target
constr_expr_h = norm(Q*sym_x(1:2));
constr_lh = 1;
constr_uh = 100;

% Cost
cost_expr_y = sym_u;        % minimize control inputs along the way
cost_W = eye(nu);
cost_W_e = blkdiag(100*eye(nx-1),1);
cost_Vx_e = eye(nx);

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
ocp_model.set('constr_lbx_0', repmat(-100,nx,1));
ocp_model.set('constr_ubx_0', repmat(100,nx,1));
ocp_model.set('constr_Jbx_0', eye(nx));
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
ocp_opts.set('print_level', 3);
ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);

% Create ocp solver
ocp = acados_ocp(ocp_model, ocp_opts);

end