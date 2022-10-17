%% Parameters
params = struct();
params.mass = 1; %4kg
params.max_thrust = 1; %0.5N
params.min_btw_agents = 5;
params.min_to_target = 5;
params.max_to_target_initial = 50;
params.min_to_target_initial = 25;
params.max_to_target_final = 10;
params.min_to_target_final = 5;
params.max_time = 100;
params.na = 2;      % SET number of agents

%% Create OCP object and load trained network                   
ocp = ocp_translation_2D_swarm(params);
% net = load("C:\Users\Weronika\Documents\GitHub\imitate-orbital-nmpc\matlab_functions\ocp_examples\translation_2D_avoid_hub\neural_network\neural_network_objects\nn_obj_2d_trans_avoidhub_fulltraj_donut_11oct_2.mat");
net = load("C:\Users\Weronika\Documents\GitHub\imitate-orbital-nmpc\matlab_functions\ocp_examples\translation_2D_avoid_hub\neural_network\neural_network_objects\nn_obj_2d_trans_avoidhub_fulltraj_donut_27sep.mat");
net = net.nn_obj;
net_swarm = load("C:\Users\Weronika\Documents\GitHub\imitate-orbital-nmpc\matlab_functions\ocp_examples\translation_2D_swarm\neural_network\neural_network_objects\nn_obj_2d_trans_avoidhub_swarm_2agents_hyperopt.mat");
net_swarm = net_swarm.nn_obj;
% distribution = load("C:\Users\Weronika\Documents\GitHub\imitate-orbital-nmpc\matlab_functions\ocp_examples\translation_2D_avoid_hub_free_final_time\data\data_translation_2D_avoid_hub_donut_11oct_2_distribution.mat");
distribution = load("C:\Users\Weronika\Documents\GitHub\imitate-orbital-nmpc\matlab_functions\ocp_examples\translation_2D_avoid_hub\data\data_translation_2D_avoid_hub_donut_28sep_distribution.mat");
distribution = distribution.distribution;
distribution_swarm = load("C:\Users\Weronika\Documents\GitHub\imitate-orbital-nmpc\matlab_functions\ocp_examples\translation_2D_swarm\data\data_translation_2D_convex_swarm_oct15_distribution.mat");
distribution_swarm = distribution_swarm.data_distribution;

%% Solve
params.N_jobs = 10;                  % SET number of paths to generate
[data_job, data_sol, data_net, data_net_swarm] = acados_solve(ocp, net, net_swarm, params, distribution, distribution_swarm);

%% Save datasets
% filename = 'data_translation_2D_convex_swarm_oct15_2';      % SET the file name
% folder = fileparts(cd);
% [data_sol_norm_2, data_distribution_2] = normalize_data(data_sol_2,[],params.na);
% data_2 = [data_job_2, data_sol_norm_2];
% save(strcat(folder,'/data/',filename,'.mat'), 'data_2', '-v7.3')
% save(strcat(folder,'/data/',filename,'_distribution.mat'), 'data_distribution_2', '-v7.3')