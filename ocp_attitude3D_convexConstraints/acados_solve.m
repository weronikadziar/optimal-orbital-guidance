function data = acados_solve(ocp, params)

% DESCRIPTION
% Solves the attitude OCP problem for a desired number of different initial
% conditions. Creates a dataset of converged solutions, which can be used
% to train a neural network to replicate the optimal trajectories for a
% given initial state.

% INPUTS
% - ocp: acados_ocp object for the problem
% - params: data structure with values of the parametrized variables
%           - filename: file name for saving the dataset
%           - save: indicates whether to save the dataset, 1 to save, 0 to
%                   not save
%           - N_start: number of different starting points around which to
%                      perform random walk
%           - N_rand_steps: number of random steps to perform around each
%                           starting point

% OUTPUTS
% - data: array with flattened state and input trajectories, dimension
%         (N_start + N_start*N_rand_steps, nx*(N+1)+nu*N)


% System dimensions
N_start = params.N_start;
N_rand_steps = params.N_rand_steps;
N = ocp.opts_struct.param_scheme_N;
nx = 12;
nu = 3;

% Desired final state
R = eye(3); 
xf = [R(:); zeros(3,1)];
ocp.set('cost_y_ref_e', xf);

fprintf('\nInitializing path planning...\n')

% Allocate an array for the dataset
data = zeros((N_start+1)*N_rand_steps,nx*(N+1)+nu*N);

% Max number of tries to find a converged solution, if exceeded, just quit
% trying
it_max = 10000;

% Count number of converged solutions
conv = 0;

tic

for job = 1:N_start

    % Find new starting point
    status = 1;
    it = 0;
    % Keep solving for different initial conditions until one solution
    % converges
    while status > 0 && it < it_max
        x0 = generate_initial_state();
        ocp.set('constr_x0', x0);
        init_x = zeros(nx,N+1);
        init_u = zeros(nu,N);
        ocp.set('init_x', init_x);
        ocp.set('init_u', init_u);
        ocp.solve();
        status = ocp.get('status');
        it = it + 1;
    end

    % If no solutions converged, the problem might be always infeasible
    if it == it_max
        disp('No feasible solution found')
        return
    end

    % If the code got up to here, then we've got one converged solution and
    % we can do a random walk around it to get more converged paths
    conv = conv + 1;    
    x_traj = ocp.get('x');
    u_traj = ocp.get('u');
    data(conv,:) = [x_traj(:); u_traj(:)]';

    % Make random steps and use sentitivity to initialize OCPs
    for i = 1:N_rand_steps
        [init_x, init_u] = augment_data(ocp, 1);
        x0 = init_x(:,1);
        ocp.set('constr_x0', x0);
        ocp.set('init_x', init_x);
        ocp.set('init_u', init_u);
        ocp.solve();
        status = ocp.get('status');
        if status == 0
            conv = conv + 1;
            x_traj = ocp.get('x');
            u_traj = ocp.get('u');
            data(conv,:) = [x_traj(:); u_traj(:)]';
        end
    end
    
    % Print progress
    progress = job/N_start*100;
    if rem(progress,2) == 0
        fprintf('%d%% complete\n', round(progress))
    end

end

toc

% Remove unconverged array rows
data = data(1:conv,:);

% Save the dataset if specified
if params.save == 1
    save(strcat(fileparts(cd),'\neural_network_training\data\',params.filename,'.mat'),'data');
end

end




