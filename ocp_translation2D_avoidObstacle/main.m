% Generate fuel-optimal trajectories for orbital proximity operations using
% 2D translation dynamics. An agent is spawned at some distance away from 
% the target, and a random docking point is generated. The agent has to 
% reach the docking points while avoiding collisions with the target. The 
% OCP solver is initialized using an initial guess provided by a similar
% OCP solver that uses all the same properties but it excludes any
% nonconvex constraints. The convex version of the problem will always
% converge, and the nonconvex solver will refine this solution. To generate
% more data faster, use the augmentation function. For each converged
% solution, the function will generate N_aug paths around the solution by
% using its sensitivity function.

% Define system parameters and trajectory limits 
params = struct();
params.filename = 'test_network'; % SET name for saving the dataset
params.mass = 1;                                    % SET spacecraft mass [kg]
params.max_thrust = 1;                              % SET maximum thrust [N]
params.T = 100;                                     % SET time horizon [s]
params.min_to_target = 5;                           % SET min distance to be kept from target at all times [m]
params.max_to_target_initial = 50;                  % SET max radius from target for the initial position [m]
params.min_to_target_initial = 25;                  % SET min radius from target for the initial position [m]
params.max_to_target_final = 10;                    % SET max radius from target for the final position [m]
params.min_to_target_final = 5;                     % SET min radius from target for the final position [m]
params.max_vel_initial = 0.1;                       % SET max initial velocity [m/s]

% Source the Acados environment if it's not done yet
env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
    your_acados_matlab_dir = "C:\Users\Weronika\acados\examples\acados_matlab_octave"; % SET your directory with the env file
	run(strcat(your_acados_matlab_dir,"\acados_env_variables_windows.m"))
end

% Create OCP objects
ocp = ocp_translation2D_avoidObstacle(params);
ocp_init = ocp_translation2D_convex(params);

%% Solve
params.save = 1;                                    % SET to 1 to save the datasets, or to 0 to not save the datasets
params.N_jobs = 100000;                                % SET number of paths to generate
params.N_aug = 0;                                   % SET number of augmented paths to generate for each converged solution
[data_job, data_sol, data_aug_job, data_aug_sol] = acados_solve(ocp, ocp_init, params);
plot_paths(data_job, data_sol)

