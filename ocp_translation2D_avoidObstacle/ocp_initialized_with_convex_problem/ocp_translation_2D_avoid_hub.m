function ocp = ocp_translation_2D_avoid_hub(params)

% Import Acados and Casadi
addpath("C:\Users\Weronika\acados\examples\acados_matlab_octave")
run("acados_env_variables_windows.m")
import casadi.*

%% Set up parameters

% System parameters
nx = 4;                    % number of states
nu = 2;                    % number of inputs

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
Ad = [4-3*cos(n*dt), 0, 1/n*sin(n*dt), 2/n*(1-cos(n*dt));
      6*(sin(n*dt)-n*dt), 1, -2/n*(1-cos(n*dt)), 1/n*(4*sin(n*dt)-3*n*dt);
      3*n*sin(n*dt), 0, cos(n*dt), 2*sin(n*dt);
      -6*n*(1-cos(n*dt)), 0, -2*sin(n*dt), 4*cos(n*dt)-3];
Bd = f_thr/m*[1/n^2*(1-cos(n*dt)), 2/n^2*(n*dt-sin(n*dt));
              -2/n^2*(n*dt-sin(n*dt)), 4/n^2*(1-cos(n*dt))-3/2*(dt)^2;
              1/n*sin(n*dt), 2/n*(1-cos(n*dt));
              -2/n*(1-cos(n*dt)), 4/n*sin(n*dt)-3*dt];
dyn_expr_phi = Ad*sym_x + Bd*sym_u;

% Constraints
constr_Jbu = eye(nu);       % constrain control actions
constr_lbu = -ones(nu,1);
constr_ubu = ones(nu,1);
a = params.min_to_target;
Q = diag([1/a, 1/a]);       % don't hit the target
constr_expr_h = norm(Q*sym_x(1:2));
constr_lh = 1;
constr_uh = 100;
% constr_Jsh = eye(length(constr_expr_h));

% Cost
cost_expr_y = sym_u;        % minimize control inputs along the way
cost_W = eye(nu);
cost_W_e = 100*eye(nx);     % minimize deviation from desired end point
cost_Vx_e = eye(nx);

% Time parameters
T = 100;
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
ocp_model.set('dyn_expr_phi', dyn_expr_phi);

% Constraints
ocp_model.set('constr_x0', zeros(nx,1));
ocp_model.set('constr_Jbu', constr_Jbu);
ocp_model.set('constr_lbu', constr_lbu);
ocp_model.set('constr_ubu', constr_ubu);
ocp_model.set('constr_expr_h', constr_expr_h);
ocp_model.set('constr_lh', constr_lh);
ocp_model.set('constr_uh', constr_uh);
% ocp_model.set('constr_Jsh', constr_Jsh);

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