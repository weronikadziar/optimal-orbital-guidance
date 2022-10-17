%% Define trajectory limits 
params = struct();
params.min_btw_agents = 5;
params.min_to_target = 5;
params.max_to_target_initial = 50;
params.min_to_target_initial = 25;
params.max_to_target_final = 10;
params.min_to_target_final = 5;

%% Create OCP objects
ocp = ocp_translation_2D_avoid_hub(params);
ocp_init = ocp_translation_2D_convex();

%% Solve
N_jobs = 100;           % SET number of paths to generate
N_aug = 0;                 % SET number of augmented paths, set to 0 for NO augmentation
[data_job, data_sol] = acados_solve(ocp, ocp_init, params, N_jobs, N_aug);

%% Normalize and save dataset
filename = 'data_translation_2D_avoid_hub_donut_28sep';      % SET the file name
[data_sol_norm, distribution] = normalize_data(data_sol,[]);
data = [data_job; data_sol_norm];
folder = fileparts(cd);
% save(strcat(folder,'/data/',filename,'.mat'), 'data', '-v7.3')
% save(strcat(folder,'/data/',filename,'_distribution.mat'), 'distribution', '-v7.3')