% Generate fuel-optimal trajectories for orbital proximity operations using
% 3D attitude dynamics. An agent is spawned at some attitude and has to
% reach some desired attitude and a zero velocity. Here, the desired
% attitude is the identity matrix. The problem is highly non-convex so it
% will often fail to converge. Once we get one converged solution, we make
% a small step in the initial condition and we use the sensitivity function
% from the solver to generate a path starting at this point. The
% sensitivity function is the derivative of the state and input trajectory
% with respect to the initial state. We perform this random walk until the
% whole space of possible initial states has been covered. This way, a
% dataset is generated and it can be used to train a neural network to plan
% trajectories so that in the future, we can just initialize with the
% output of the trained network.

% Source the Acados environment if it's not done yet
env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
    your_acados_matlab_dir = "C:\Users\Weronika\acados\examples\acados_matlab_octave"; % SET your directory with the env file
	run(strcat(your_acados_matlab_dir,"\acados_env_variables_windows.m"))
end

% Create OCP object
ocp = ocp_attitude3D_convex();

%% Solve and save the dataset
params = struct();
params.save = 0;                   % SET to 1 to save the dataset, otherwise set to 0
params.filename = 'my_file';       % SET the filename for saving the dataset
params.N_start = 100;              % SET number of starting points for random walk
params.N_rand_steps = 10;         % SET number of random steps around the converged path
data = acados_solve(ocp, params);