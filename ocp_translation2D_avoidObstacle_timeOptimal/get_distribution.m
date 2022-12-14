function distribution = get_distribution(data)

% DESCRIPTION
% Computes the means and standard deviations of all states and control inputs.

% INPUTS
% - data: dataset to analyze, array of dimension (N_jobs, 1+(nx+nu)*N)

% OUTPUTS
% - distribution: data structure with fields mean and sd 

% Dimensions
nx = 4;
nu = 2;
N = 100;

% Compute max and min of the optimal time
time_max = max(data(:,1));
time_min = min(data(:,1));

% Compute max and min values for each state
state_max = zeros(1,nx);
state_min = zeros(1,nx);
for i = 1:nx
    state = data(:,i+1:nx:nx*(N+1));
    state_max(i) = max(state(:));
    state_min(i) = min(state(:));
end

% Compute max and min values of control inputs
input = data(:,nx*(N+1)+2:end);
input_max = max(input(:));
input_min = min(input(:));

% Make an array with max and min values for each entry of data
sol_max = [time_max, repmat(state_max,1,N+1), repmat(input_max,1,nu*N)];
sol_min = [time_min, repmat(state_min,1,N+1), repmat(input_min,1,nu*N)];

% Populate structure
distribution = struct();
distribution.mean = sol_min;
distribution.sd = sol_max - sol_min;

end