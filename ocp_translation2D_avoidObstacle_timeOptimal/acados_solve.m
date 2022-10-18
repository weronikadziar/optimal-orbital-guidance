function [data_job, data_sol] = acados_solve(ocp_timeOptimal, ocp_fuelOptimal, ocp_cvx, params)

% DESCRIPTION
% Solves the OCP problem for a desired number of different initial and
% final states. At the end, it saves the converged solutions and their 
% distrubution as a .mat file.

% INPUTS
% - ocp_timeOptimal: acados_ocp object for the time optimal docking and 
%                    obstacle avoidance problem
% - ocp_fuelOptimal: acados_ocp object for the fuel optimal docking and 
%                    obstacle avoidance problem
% - ocp_cvx: acados_ocp object with all the same properties as
%            ocp_fuelOptimal but excluding any nonconvex constraints
% - params: data structure with values of the parametrized variables
%           (filename, N_jobs, mass, max_thrust, max_time, min_to_target,
%           max_to_target_initial, min_to_target_initial,
%           max_to_target_final, min_to_target_final, max_vel_initial)

% OUTPUTS
% - data_job: array with flattened initial states and desired final 
%             positions for all converged jobs, only the position of the 
%             desired final state is saved because the desired final 
%             velocity is always zero, dimension (N_jobs, nx+nx/2)
% - data_sol: array with flattened state and input trajectories of the OCP
%             solution for all converged jobs, dimension (N_jobs, (nx+nu)*N)

N_jobs = params.N_jobs;             % number of paths to generate
N = ocp.opts_struct.param_scheme_N; % number of discretization points
nx = 4;                             % number of states excluding the time factor
nu = 2;                             % number of inputs

% Allocate arrays for datasets
data_job = zeros(N_jobs,nx+nx/2);
data_sol = zeros(N_jobs,(nx+nu)*N+1);

fprintf('\nInitializing path planning...\n')

% Count number of converged solutions
conv = 0;

tic

for job = 1:N_jobs

    % Generate initial and final state
    x0 = generate_initial_state(params);
    xf = generate_final_state(params);

    % Solve the convex problem first
    ocp_cvx.set('constr_x0', x0);
    ocp_cvx.set('cost_y_ref_e', xf);
    ocp_cvx.solve();
    init_x = ocp_cvx.get('x');
    init_u = ocp_cvx.get('u');

    % Use the convex solution to initialize the fuel-optimal obstacle avoidance problem
    ocp_fuelOptimal.set('constr_x0', x0);
    ocp_fuelOptimal.set('cost_y_ref_e', xf);
    ocp_fuelOptimal.set('init_x', init_x);
    ocp_fuelOptimal.set('init_u', init_u);
    ocp_fuelOptimal.solve();
    init_x = ocp_fuelOptimal.get('x');
    init_u = ocp_fuelOptimal.get('u');
    status = ocp_fuelOptimal.get('status');

    % If converged, use this to initialize the time-optimal obstacle avoidance problem
    if status == 0
        ocp_timeOptimal.set('constr_lbx', [x0; 0], 0);
        ocp_timeOptimal.set('constr_ubx', [x0; params.max_time], 0);
        ocp_timeOptimal.set('cost_y_ref_e', [xf; 0]);
        ocp_timeOptimal.set('init_x', [init_x; repmat(params.max_time, 1, N+1)]);
        ocp_timeOptimal.set('init_u', init_u);
        ocp_timeOptimal.solve();
        x_traj = ocp_timeOptimal.get('x');
        u_traj = ocp_timeOptimal.get('u');
        status = ocp_timeOptimal.get('status');

        % If solution converged, save it to the dataset
        if status == 0
            conv = conv + 1;
            data_job(conv,:) = [x0; xf(1:nx/2)]';
            time = x_traj(end,1);
            x_traj = x_traj(1:nx,2:end);
            data_sol(conv,:) = [time; x_traj(:); u_traj(:)]';
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
data = [data_job, data_sol];

% Calculate solution data distribution (needed for network training)
data_distribution = get_distribution(data_sol);

% Save dataset and its distribution
save(strcat('data_',params.filename,'.mat'), 'data', '-v7.3')
save(strcat('net_',params.filename,'_distribution.mat'), 'data_distribution', '-v7.3')

end