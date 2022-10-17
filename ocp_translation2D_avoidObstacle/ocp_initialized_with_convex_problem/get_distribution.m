function distribution = get_distribution(data)

state_max = [];
state_min = [];
for i = 1:4
    state = data(:,i:4:400);
    state_max = [state_max, max(state(:))];
    state_min = [state_min, min(state(:))];
end

input = data(:,401:end);
input_max = max(input(:));
input_min = min(input(:));

job_max = [state_max, state_max(1:2)];
job_min = [state_min, state_min(1:2)];
job_mean = job_min;
job_sd = job_max - job_min;
sol_max = [repmat(state_max,1,100), repmat(input_max,1,200)];
sol_min = [repmat(state_min,1,100), repmat(input_min,1,200)];
sol_mean = sol_min;
sol_sd = sol_max - sol_min;

distribution = struct();
distribution.job_mean = job_mean;
distribution.job_sd = job_sd;
distribution.sol_mean = sol_mean;
distribution.sol_sd = sol_sd;

end