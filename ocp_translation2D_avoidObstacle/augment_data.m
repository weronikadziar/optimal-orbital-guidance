function [x_traj_aug, u_traj_aug] = augment_data(ocp, N_aug)

% DESCRIPTION
% Augments the converged solution by using the sensitivity of the solution
% with respect to the initial value. A small random step is made from the
% initial solution and the new path is generated.

% INPUTS
% - ocp: acados_ocp object with a converged solution
% - N_aug: number of new paths to generate for the given solution

% OUTPUTS
% - x_traj_aug: array with augmented state trajectories, dimension
%               (N_aug, nx*N)
% - u_traj_aug: array with augmented input trajectories, dimension
%               (N_aug, nu*N)

% Get the nominal paths and their dimensions
x_traj = ocp.get('x');
u_traj = ocp.get('u');
nx = size(x_traj,1);
nu = size(u_traj,1);
N = size(u_traj,2);

% Define sensitivity parameters and allocate sensitivity arrays
field = 'ex';
stage = 0;
sens_u = zeros(nx, nu, N);
sens_x = zeros(nx, nx, N+1);

% Get sensitivities w.r.t. initial state value with index
for index = 0:nx-1
    ocp.eval_param_sens(field, stage, index);
    sens_u(index+1,:,:) = ocp.get('sens_u');
    sens_x(index+1,:,:) = ocp.get('sens_x');
end

% Allocate arrays for augmented paths
x_traj_aug = zeros(N_aug,nx*(N+1));
u_traj_aug = zeros(N_aug,nu*N);

for i=1:N_aug
    
    x_traj_sens = x_traj;
    u_traj_sens = u_traj;

    % Random walk
    step = [5e-1,5e-1,1e-2,1e-2].*randn(1,4);
   
    % Loop over every state and make a small step for each state
    for j=1:nx      
        x_traj_sens = x_traj_sens + squeeze(sens_x(j,:,:))*step(j);
        u_traj_sens = u_traj_sens + squeeze(sens_u(j,:,:))*step(j);
    end
    
    % Add the augmented paths to the arrays
    x_traj_aug(N_aug,:) = x_traj_sens(:)';
    u_traj_aug(N_aug,:) = u_traj_sens(:)';

end

end