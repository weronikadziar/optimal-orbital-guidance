%% Create OCP objects
na = 4;                      % SET number of agents (max 4)
ocp = ocp_translation_2D_swarm(na);
ocp_init = ocp_translation_2D_agent();

%% Solve
N_jobs = 1;                  % SET number of paths to generate
% data = agent_assignment_global(ocp, ocp_init, N_jobs);
% [inputs, outputs] = agent_assignment(ocp_init, N_jobs);
[data, ~, ocp_sol] = acados_solve(ocp, ocp_init, N_jobs, na);

%% Save datasets
filename = 'data_translation_2D_convex_swarm';      % SET the file name
folder = fileparts(fileparts(cd));
% save(strcat(folder,'/data/',filename,'.mat'), 'data')