%% Create OCP object
ocp = ocp_attitude_convex_constraints();

%% Solve
R = eye(3); 
xf = [R(:); zeros(3,1)];    % SET desired endpoint
N_start = 10;               % SET number of starting points for random walk
N_rand_steps = 2;         % SET number of random steps around the converged path
data = acados_solve(ocp, xf, N_start, N_rand_steps);

%% Save datasets
filename = 'data_attitude_convex_constraints';      % SET the file name
folder = fileparts(fileparts(cd));
% save(strcat(folder,'/data/',filename,'.mat'), 'data')