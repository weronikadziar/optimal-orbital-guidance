function ocp = acados_ocp_obj(model)

% Set up acados interface
addpath("C:\Users\Weronika\acados\examples\acados_matlab_octave")
run("acados_env_variables_windows.m")

% Size
nx = model.nx;    % number of states

% Solver
nlp_solver = 'sqp';
qp_solver = 'partial_condensing_hpipm';   
qp_solver_cond_N = 5;

% Create the object
ocp_model = acados_ocp_model();
model_name = strcat('ocp',num2str(randi(100)));
ocp_model.set('name', model_name);
ocp_model.set('T', model.T);

% Symbolics
ocp_model.set('sym_x', model.sym_x);
ocp_model.set('sym_u', model.sym_u);
ocp_model.set('sym_xdot', model.sym_xdot);

% Cost
ocp_model.set('cost_type', 'nonlinear_ls');
ocp_model.set('cost_expr_y', model.cost_expr_y); 
ocp_model.set('cost_W', model.cost_W);
ocp_model.set('cost_type_e', 'linear_ls');
ocp_model.set('cost_Vx_e', model.cost_Vx_e);
ocp_model.set('cost_W_e', model.cost_W_e);

% Dynamics
ocp_model.set('dyn_type', model.dyn_type);
ocp_model.set('dyn_expr_phi', model.dyn_expr_phi);

% Constraints
ocp_model.set('constr_x0', zeros(nx,1));
ocp_model.set('constr_Jbu', model.constr_Jbu);
ocp_model.set('constr_lbu', model.constr_lbu);
ocp_model.set('constr_ubu', model.constr_ubu);

% Acados ocp set opts
ocp_opts = acados_ocp_opts();
ocp_opts.set('param_scheme_N', model.N);
ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('sim_method', 'erk');
ocp_opts.set('qp_solver', qp_solver);
ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);

% Create ocp solver
ocp = acados_ocp(ocp_model, ocp_opts);

end
