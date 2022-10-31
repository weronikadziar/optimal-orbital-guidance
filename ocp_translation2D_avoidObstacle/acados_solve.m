function [data_job, data_sol, data_aug_job, data_aug_sol] = acados_solve(ocp, ocp_init, params)

% DESCRIPTION
% Solves the OCP problem for a desired number of different initial and
% final states. At the end, it saves the converged solutions and their 
% distrubution as a .mat file.

% INPUTS
% - ocp: acados_ocp object for the docking and obstacle avoidance problem
% - ocp_init: acados_ocp object with all the same properties as the ocp
%             object but excluding any nonconvex constraints
% - params: data structure with values of the parametrized variables
%           (filename, N_jobs, mass, max_thrust, T, min_to_target,
%           max_to_target_initial, min_to_target_initial,
%           max_to_target_final, min_to_target_final, max_vel_initial)

% OUTPUTS
% - data_job: array with flattened initial states and desired final 
%             positions for all converged jobs, only the position of the 
%             desired final state is saved because the desired final 
%             velocity is always zero, dimension (N_jobs, nx+nx/2)
% - data_sol: array with flattened state and input trajectories of the OCP
%             solution for all converged jobs, 
%             dimension (N_jobs, (nx+nu)*N)
% - data_aug_job & data_aug_sol: equivalent to the first two arrays but
%                                involving trajectories from augmenting data 

N_jobs = params.N_jobs;             % number of paths to generate
N_aug = params.N_aug;               % number of augmented paths to generate for each converged solution
N = ocp.opts_struct.param_scheme_N; % number of discretization points
nx = 4;                             % number of states
nu = 2;                             % number of inputs

% Allocate arrays for datasets
data_job = zeros(N_jobs,nx+nx/2);
data_sol = zeros(N_jobs,(nx+nu)*N);
data_aug_job = [];
data_aug_sol = [];
if N_aug > 0
    data_aug_job = zeros(N_jobs,nx+nx/2);
    data_aug_sol = zeros(N_jobs,(nx+nu)*N);
end

fprintf('\nInitializing path planning...\n')

% Count number of converged solutions
conv = 0;

tic

for job = 1:N_jobs

    % Generate initial and final state
    x0 = generate_initial_state(params);
    xf = generate_final_state(params);

    % Solve the convex problem first
    ocp_init.set('constr_x0', x0);
    ocp_init.set('cost_y_ref_e', xf);
    ocp_init.solve();
    init_x = ocp_init.get('x');
    init_u = ocp_init.get('u');

    % Use the convex solution to initialize the nonconvex problem
    ocp.set('constr_x0', x0);
    ocp.set('cost_y_ref_e', xf);
    ocp.set('init_x', init_x);
    ocp.set('init_u', init_u);
    ocp.solve();
    x_traj = ocp.get('x');
    u_traj = ocp.get('u');
    status = ocp.get('status');

    % If solution converged, save it to the dataset
    if status == 0
        conv = conv + 1;
        data_job(conv,:) = [x0; xf(1:nx/2)]';
        x_traj = x_traj(:,2:end);
        data_sol(conv,:) = [x_traj(:); u_traj(:)]';
        % Augment data if specified
        if N_aug > 0
            [x_traj_aug, u_traj_aug] = augment_data(ocp, N_aug);
            data_aug_job((conv-1)*N_aug+1:conv*N_aug,:) = [x_traj_aug(:,1:nx), repmat(xf(1:nx/2)',N_aug,1)];
            data_aug_sol((conv-1)*N_aug+1:conv*N_aug,:) = [x_traj_aug(:,nx+1:end), u_traj_aug];
        end
    end

    % Print progress
    progress = job/N_jobs*100;
    if rem(progress,5) == 0
        fprintf('%d%% complete\n', round(progress))
    end

end

toc

% Remove unconverged array rows
data_job = data_job(1:conv,:);
data_sol = data_sol(1:conv,:);
data_aug_job = data_aug_job(1:conv*N_aug,:);
data_aug_sol = data_aug_sol(1:conv*N_aug,:);

% Save the datasets and distributions if specified
if params.save == 1

    % Calculate solution data distribution (needed for network training)
    data_distribution = get_distribution(data_sol);
    
    % Save dataset and its distribution
    data = [data_job, data_sol];
    save(strcat(fileparts(cd),'\neural_network_training\data\data_',params.filename,'.mat'), 'data', '-v7.3')
    save(strcat(fileparts(cd),'\neural_network_training\data\data_distribution_',params.filename,'.mat'), 'data_distribution', '-v7.3')
    
    % If augmenting data, get the distribution of the nominal and augmented
    % paths together
    if N_aug > 0
        data_aug_distribution = get_distribution([data_sol;data_aug_sol]);
        data_aug = [[data_job, data_sol]; [data_aug_job, data_aug_sol]];
        save(strcat(fileparts(cd),'\neural_network_training\data\data_aug_',params.filename,'.mat'), 'data_aug', '-v7.3')
        save(strcat(fileparts(cd),'\neural_network_training\data\data_aug_distribution_',params.filename,'.mat'), 'data_aug_distribution', '-v7.3')
    end

end

end