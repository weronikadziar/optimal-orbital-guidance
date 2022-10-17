function ocp = ocp_translation_2D_swarm(na)

% Import Acados and Casadi
addpath("C:\Users\Weronika\acados\examples\acados_matlab_octave")
run("acados_env_variables_windows.m")
import casadi.*

%% Set up parameters

% System parameters
ndim = 2;
nx = ndim*2*na;                 % number of states
nu = ndim*na;                 % number of inputs

% Symbolic variables
sym_x = SX.sym('x', [nx 1]);     % chasers relative pos and vel in LVLH frame
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
Ad_cell = repmat({Ad}, 1, na);
Ad_swarm = blkdiag(Ad_cell{:});
Bd = f_thr/m*[1/n^2*(1-cos(n*dt)), 2/n^2*(n*dt-sin(n*dt));
              -2/n^2*(n*dt-sin(n*dt)), 4/n^2*(1-cos(n*dt))-3/2*(dt)^2;
              1/n*sin(n*dt), 2/n*(1-cos(n*dt));
              -2/n*(1-cos(n*dt)), 4/n*sin(n*dt)-3*dt];
Bd_cell = repmat({Bd}, 1, na);
Bd_swarm = blkdiag(Bd_cell{:});
dyn_expr_phi = Ad_swarm*sym_x + Bd_swarm*sym_u;

% Constraints
constr_Jbu = eye(nu);       % constrain control actions
constr_lbu = -ones(nu,1);
constr_ubu = ones(nu,1);

constr_expr_h = SX.sym('pos_rel', [na*(na-1) 1]); % avoid bumping into other agents
constr_lh = repmat(3,na*(na-1),1);
constr_uh = repmat(100,na*(na-1),1);

i = 1;
for agent = 1:na
    pos_agent = sym_x(1+(agent-1)*ndim*2:2+(agent-1)*ndim*2);
    for neig = 1:na
        if neig ~= agent
            pos_neig = sym_x(1+(neig-1)*ndim*2:2+(neig-1)*ndim*2);
            pos_rel = pos_neig - pos_agent;
            constr_expr_h(i) = norm(pos_rel);
            i = i + 1;
        end
    end
end

% Cost
cost_Vu = eye(nu);          % keep control actions low
cost_Vx = zeros(nu,nx);
cost_W = eye(nu);
cost_y_ref = zeros(nu,1);

cost_Vx_e = eye(nx);        % reach desired final state
cost_W_e = eye(nx);

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
ocp_model.set('cost_type', 'linear_ls');
ocp_model.set('cost_Vu', cost_Vu); 
ocp_model.set('cost_Vx', cost_Vx);
ocp_model.set('cost_W', cost_W);
ocp_model.set('cost_y_ref', cost_y_ref);
ocp_model.set('cost_type_e', 'linear_ls');
ocp_model.set('cost_Vx_e', cost_Vx_e); 
ocp_model.set('cost_W_e', cost_W_e);
% ocp_model.set('cost_y_ref_e', cost_y_ref_e);

% Dynamics
ocp_model.set('dyn_type', dyn_type);
ocp_model.set('dyn_expr_phi', dyn_expr_phi);

% Constraints
ocp_model.set('constr_x0', zeros(nx,1));
ocp_model.set('constr_Jbu', constr_Jbu);
ocp_model.set('constr_lbu', constr_lbu);
ocp_model.set('constr_ubu', constr_ubu);
ocp_model.set('constr_expr_h', constr_expr_h)
ocp_model.set('constr_lh', constr_lh)
ocp_model.set('constr_uh', constr_uh)

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