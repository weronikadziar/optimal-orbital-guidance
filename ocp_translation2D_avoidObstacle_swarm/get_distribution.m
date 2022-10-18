function distribution = get_distribution(data,N_agents)

% DESCRIPTION
% Computes the means and standard deviations of all states of a swarm OCP,
% and of the control inputs.

% INPUTS
% - data: dataset to analyze, array of dimension (N_jobs, (nx+nu)*na*N)
% - N_agents: number of agents in the swarm, scalar

% OUTPUTS
% - distribution: data structure with fields mean and sd 

% Dimensions
nx = 4;
nu = 2;
N = 100;

% Compute max and min values for each state (number of states = nx*na)
state_max = zeros(1,nx*N_agents);
state_min = zeros(1,nx*N_agents);
for i = 1:nx*N_agents
    state = data(:,i:nx*N_agents:nx*N_agents*N);
    state_max(i) = max(state(:));
    state_min(i) = min(state(:));
end

% Compute max and min values of control inputs
input = data(:,nx*N_agents*N+1:end);
input_max = max(input(:));
input_min = min(input(:));

% Make an array with max and min values for each entry of data
sol_max = [repmat(state_max,1,N), repmat(input_max,1,nu*N_agents*N)];
sol_min = [repmat(state_min,1,N), repmat(input_min,1,nu*N_agents*N)];

% Populate structure
distribution = struct();
distribution.mean = sol_min;
distribution.sd = sol_max - sol_min;


end