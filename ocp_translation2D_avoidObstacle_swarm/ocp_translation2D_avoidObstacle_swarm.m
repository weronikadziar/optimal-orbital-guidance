function ocp = ocp_translation2D_avoidObstacle_swarm(N_agents)

% DESCRIPTION
% Creates the Acados OCP object for a swarm of N_agents.

import casadi.*

%% Set up parameters

% System parameters
ndim = 2;                              % number of spatial dimensions
nx = 2*ndim*N_agents;                  % number of states
nu = ndim*N_agents;                    % number of inputs

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
Ad_cell = repmat({Ad}, 1, N_agents);
Ad_swarm = blkdiag(Ad_cell{:});
Bd = f_thr/m*[1/n^2*(1-cos(n*dt)), 2/n^2*(n*dt-sin(n*dt));
              -2/n^2*(n*dt-sin(n*dt)), 4/n^2*(1-cos(n*dt))-3/2*(dt)^2;
              1/n*sin(n*dt), 2/n*(1-cos(n*dt));
              -2/n*(1-cos(n*dt)), 4/n*sin(n*dt)-3*dt];
Bd_cell = repmat({Bd}, 1, N_agents);
Bd_swarm = blkdiag(Bd_cell{:});
dyn_expr_phi = Ad_swarm*sym_x + Bd_swarm*sym_u;

% Constraints: control actions
constr_Jbu = eye(nu);
constr_lbu = -ones(nu,1);
constr_ubu = ones(nu,1);

% Constraints: collision avoidance between all agent pairs and the target
N_pairs = nchoosek(N_agents,2);
constr_expr_h = SX.sym('pos_rel', [N_pairs+N_agents 1]);

min_to_target = 5;
min_btw_agents = 5;

constr_lh = zeros(N_pairs+N_agents,1);
constr_uh = repmat(200,N_pairs+N_agents,1);

i = 1;
for agent = 1:N_agents
    pos_agent = sym_x(1+(agent-1)*ndim*2:2+(agent-1)*ndim*2);
    constr_expr_h(i) = norm(pos_agent);
    constr_lh(i) = min_to_target;
    i = i + 1;
    for neig = agent+1:N_agents
        pos_neig = sym_x(1+(neig-1)*ndim*2:2+(neig-1)*ndim*2);
        constr_expr_h(i) = norm(pos_neig - pos_agent);
        constr_lh(i) = min_btw_agents;
        i = i + 1;
    end
end

% Cost: keep the control actions low
cost_Vu = eye(nu);
cost_Vx = zeros(nu,nx);
cost_W = eye(nu);
cost_y_ref = zeros(nu,1);

% Cost: minimize deviations from the desired final state
cost_Vx_e = eye(nx);
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