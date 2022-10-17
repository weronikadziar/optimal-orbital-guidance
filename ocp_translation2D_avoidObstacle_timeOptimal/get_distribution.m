function distribution = get_distribution(data)
% 
% time_max = max(data(:,1));
% time_min = min(data(:,1));

state_max = [];
state_min = [];
for i = 1:4
    state = data(:,1+i:4:400);
    state_max = [state_max, max(state(:))];
    state_min = [state_min, min(state(:))];
end

input = data(:,402:end);
input_max = max(input(:));
input_min = min(input(:));

% sol_max = [time_max, repmat(state_max,1,100), repmat(input_max,1,200)];
% sol_min = [time_min, repmat(state_min,1,100), repmat(input_min,1,200)];
sol_max = [repmat(state_max,1,100), repmat(input_max,1,200)];
sol_min = [repmat(state_min,1,100), repmat(input_min,1,200)];
sol_mean = sol_min;
sol_sd = sol_max - sol_min;

distribution = struct();
distribution.sol_mean = sol_mean;
distribution.sol_sd = sol_sd;

end