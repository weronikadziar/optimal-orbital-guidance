function [data_job, data_sol, data_net] = acados_solve(ocp, net, net_distribution, params)

% DESCRIPTION
% Solves the OCP problem for a desired number of different initial and
% final states. At the end, it saves the converged solutions and their 
% distrubution as a .mat file.

% INPUTS
% - ocp: acados_ocp object
% - net: trained SeriesNetwork, which takes the initial state and desired
%        final position, and outputs the full state and input trajectory
% - net_distribution: data structure with mean and standard deviation of
%                     the network training data, needed to denormalize the 
%                     output of the network
% - params: data structure with values of the parametrized variables
%           (N_jobs, N_agents, filename)

% OUTPUTS
% - data_job: array with flattened initial states and desired final 
%             positions for all converged jobs, only the position of the 
%             desired final state is saved because the desired final 
%             velocity is always zero, dimension (N_jobs, (nx+nx/2)*N_agents)
% - data_sol: array with flattened state and input trajectories of the OCP
%             solution for all converged jobs, dimension 
%             (N_jobs, (nx+nu)*N*N_agents)
% - data_net: array with flattened initial guess of the state and input
%             trajectories provided by the neural network, dimension
%             (N_jobs, (nx+nx/2)*N_agents)

N = ocp.opts_struct.param_scheme_N;     % number of discretization points
N_jobs = params.N_jobs;                 % number of paths to generate
N_agents = params.N_agents;             % number of agents
nx = 4;                                 % number of states of a single agent
nu = 2;                                 % number of inputs of a single agent

% All possible agent->destination allocations
perm = perms(1:N_agents);
p = length(perm);

% Allocate arrays for datasets
data_job = zeros(N_jobs*p,nx*N_agents*1.5);
data_sol = zeros(N_jobs*p,(nx+nu)*N*N_agents);
data_net = zeros(N_jobs*p,(nx+nu)*N*N_agents);

fprintf('\nInitializing path planning...\n')

% Count number of converged solutions
conv = 0;

tic

for job = 1:N_jobs

    % Generate initial and final states for all agents
    x0 = generate_initial_state(N_agents); 
    xf_list = generate_final_state(N_agents);

    % Loop over all possible agent->destination allocations
    for i = 1:p

        order = perm(i,:);
        xf = xf_list(:,order);

        % First plan paths for all agents separetely for initialization
        init_x = zeros(nx*N_agents,N+1);
        init_u = zeros(nu*N_agents,N);   
        for agent = 1:N_agents
            path = predict(net, [x0(:,agent); xf(1:nx/2,agent)]');
            path_denorm = path(1,:).*net_distribution.sol_sd + net_distribution.sol_mean;
            path_x = [x0(:,agent), reshape(path_denorm(1:N*nx),nx,N)];
            path_u = reshape(path_denorm(N*nx+1:end),nu,N);
            init_x(1+(agent-1)*nx:agent*nx,:) = path_x;
            init_u(1+(agent-1)*nu:agent*nu,:) = path_u;
        end
    
        % Now use these trajectories to initialize the OCP
        ocp.set('constr_x0', x0(:));
        ocp.set('cost_y_ref_e', xf(:));
        ocp.set('init_x', init_x);
        ocp.set('init_u', init_u);
        ocp.solve();
        x_traj = ocp.get('x');
        u_traj = ocp.get('u');
        status = ocp.get('status');
    
        % If solution converged, save it to the dataset
        if status == 0
            conv = conv + 1;
            xf = xf(1:nx/2,:);
            data_job(conv,:) = [x0(:); xf(:)]';
            x_traj = x_traj(:,2:end);
            data_sol(conv,:) = [x_traj(:); u_traj(:)]';
            init_x = init_x(:,2:end);
            data_net(conv,:) = [init_x(:); init_u(:)]';
        end

    end
    
    % Print progress
    progress = job/N_jobs*100;
    if rem(progress,1) == 0
        fprintf('%d%% complete\n', round(progress))
    end

end

toc

% Remove unconverged array rows
data_job = data_job(1:conv,:);
data_sol = data_sol(1:conv,:);
data_net = data_net(1:conv,:);

if params.save == 1

    % Calculate solution data distribution (needed for network training)
    data_distribution = get_distribution(data_sol,N_agents);
    
    % Save dataset and its distribution
    data = [data_job, data_sol];
    save(strcat(fileparts(cd),'\neural_network_training\data\data_',params.filename,'.mat'), 'data', '-v7.3')
    save(strcat(fileparts(cd),'\neural_network_training\data\data_distribution_',params.filename,'.mat'), 'data_distribution', '-v7.3')

end

end




