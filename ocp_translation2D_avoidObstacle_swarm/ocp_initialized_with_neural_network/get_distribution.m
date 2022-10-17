function distribution = get_distribution(data,na)

nx = 4;
nu = 2;
N = 100;

state_max = [];
state_min = [];
for i = 1:nx*na
    state = data(:,i:nx*na:nx*na*N);
    state_max = [state_max, max(state(:))];
    state_min = [state_min, min(state(:))];
end

input = data(:,nx*na*N+1:end);
input_max = max(input(:));
input_min = min(input(:));

job_idx = reshape(repmat(0:nx:(na-1)*nx,2,1),1,[])+repmat([1,2],1,na);
job_max = [state_max, state_max(job_idx)];
job_min = [state_min, state_min(job_idx)];
job_mean = job_min;
job_sd = job_max - job_min;
sol_max = [repmat(state_max,1,N), repmat(input_max,1,nu*na*N)];
sol_min = [repmat(state_min,1,N), repmat(input_min,1,nu*na*N)];
sol_mean = sol_min;
sol_sd = sol_max - sol_min;

distribution = struct();
distribution.state_max = state_max;
distribution.job_mean = job_mean;
distribution.job_sd = job_sd;
distribution.sol_mean = sol_mean;
distribution.sol_sd = sol_sd;

end