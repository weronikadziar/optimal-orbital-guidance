% Generate fuel-optimal trajectories for orbital proximity operations using
% 2D translation dynamics. A desired number of agents is spawned at some
% distance away from the target, and a set of random docking points is
% generated. The agents have to reach their docking points while avoiding
% collisions with each other and with the target. The OCP solver is
% initialized using an initial guess provided by a trained network. The
% network outputs a separate path for every agent from its initial to final
% state while avoiding the target. The swarm OCP refines the solution and
% makes sure that agents don't collide with each other.

% Parameters
params = struct();
params.N_agents = 2;                                      % SET number of agents
params.filename = 'translation2D_avoidObstacle_swarm2'; % SET name for saving the dataset

% Source the Acados environment if it's not done yet
env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
    your_acados_matlab_dir = "C:\Users\Weronika\acados\examples\acados_matlab_octave"; % SET your directory with the env file
	run(strcat(your_acados_matlab_dir,"\acados_env_variables_windows.m"))
end

% Create OCP object and load trained network and its training data distribution                   
ocp = ocp_translation2D_avoidObstacle_swarm(params.N_agents);
% net = load('net_translation2D_avoidObstacle.mat').net;
net = load("C:\Users\Weronika\Documents\GitHub\imitate-orbital-nmpc\matlab_functions\neural_network\new nn\nn_obj_2d_trans_avoidhub_fulltraj_test.mat").nn_obj;
net_distribution = load('data_distribution_translation2D_avoidObstacle.mat').net_distribution;

%% Solve, save the dataset and its distribution
params.save = 0;                                          % SET to 1 to save the dataset or 0 to not save it
params.N_jobs = 20;                                       % SET number of paths to generate
[data_job, data_sol, data_net] = acados_solve(ocp, net, net_distribution, params);

% Plot some example position trajectories
for i =1:10
    plot_paths(data_job, data_sol, data_net, params.N_agents)
end