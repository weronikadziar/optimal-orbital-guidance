%% Define trajectory limits 
params = struct();
params.mass = 4; %4kg
params.max_thrust = 0.5; %0.5N
params.min_btw_agents = 5;
params.min_to_target = 5;
params.max_to_target_initial = 50;
params.min_to_target_initial = 25;
params.max_to_target_final = 10;
params.min_to_target_final = 5;
params.max_time = 60;

%% Create OCP objects
ocp_free_time = ocp_translation_2D_avoid_hub_free_time(params);
ocp_const_time = ocp_translation_2D_avoid_hub_const_time(params);
ocp_cvx = ocp_translation_2D_convex_continuous(params);
% ocp = ocp_translation_2D_avoid_hub_const_time(params);

%% Network training distribution
loader = load("C:\Users\Weronika\Documents\GitHub\imitate-orbital-nmpc\matlab_functions\ocp_examples\translation_2D_avoid_hub_free_final_time\data\data_translation_2D_avoid_hub_donut_11oct.mat");
data_1 = loader.data;
loader = load("C:\Users\Weronika\Documents\GitHub\imitate-orbital-nmpc\matlab_functions\ocp_examples\translation_2D_avoid_hub_free_final_time\data\data_translation_2D_avoid_hub_donut_11oct_2.mat");
data_2 = loader.data;
%%
loader = load("C:\Users\Weronika\Documents\GitHub\imitate-orbital-nmpc\matlab_functions\ocp_examples\translation_2D_avoid_hub_free_final_time\data\data_translation_2D_avoid_hub_donut_11oct_distribution.mat");
dist_1 = loader.data_distribution;
loader = load("C:\Users\Weronika\Documents\GitHub\imitate-orbital-nmpc\matlab_functions\ocp_examples\translation_2D_avoid_hub_free_final_time\data\data_translation_2D_avoid_hub_donut_11oct_2_distribution.mat");
dist_2 = loader.data_distribution;
data_1 = denormalize_data(data_1(:,7:end),dist_1);
data_2 = denormalize_data(data_2(:,7:end),dist_2);
data = [data_1; data_2];
[~, data_distribution] = normalize_data(data,[]);

%% Solve
N_jobs = 10;               % SET number of paths to generate
% [data_job, data_init, data_sol] = acados_solve(ocp_free_time, ocp_const_time, ocp_cvx, params, N_jobs);
% [data_job, data_sol] = acados_solve_const_time(ocp, ocp_cvx, params, N_jobs);
[data_job, data_init_ocp, data_init_net, data_init_net_ocp, data_sol_ocp, data_sol_net] = acados_solve_ocp_and_net(ocp_free_time, ocp_const_time, ocp_cvx, nn_obj, data_distribution, params, N_jobs);


%% Normalize and save dataset
filename = 'data_translation_2D_avoid_hub_donut_11oct_2';      % SET the file name
% [data_sol_norm, data_distribution] = normalize_data(data_sol,[]);
% data = [data_job, data_sol_norm];
% folder = fileparts(cd);
% save(strcat(folder,'/data/',filename,'.mat'), 'data', '-v7.3')
% save(strcat(folder,'/data/',filename,'_distribution.mat'), 'data_distribution', '-v7.3')