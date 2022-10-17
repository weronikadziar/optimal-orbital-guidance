function [x_traj_aug, u_traj_aug] = augment_data(ocp, N_aug)

x_traj = ocp.get('x');
u_traj = ocp.get('u');
nx = size(x_traj,1);
nu = size(u_traj,1);
N = size(u_traj,2);

field = 'ex';
stage = 0;
sens_u = zeros(nx, nu, N);
sens_x = zeros(nx, nx, N+1);

% get sensitivities w.r.t. initial state value with index
for index = 0:nx-1
    ocp.eval_param_sens(field, stage, index);
    sens_u(index+1,:,:) = ocp.get('sens_u');
    sens_x(index+1,:,:) = ocp.get('sens_x');
end

x_traj_aug = zeros(nx,N_aug*(N+1));
u_traj_aug = zeros(nu,N_aug*N);

for i=1:N_aug
    x_traj_sens = x_traj;
    u_traj_sens = u_traj;

    % Random walk
    R_flat = x_traj(1:9);
    R = reshape(R_flat,3,3);
    [yaw, pitch, roll] = dcm2angle(R);
    angles_sens = [yaw, pitch, roll] + randn(1,3)*deg2rad(3);
    R_sens = angle2dcm(angles_sens(1),angles_sens(2),angles_sens(3));
    R_sens_flat = R_sens(:);
    R_step = R_sens_flat' - R_flat;
    step = [R_step, randn(1,3)*deg2rad(0.1)];

    for j=1:nx      
        x_traj_sens = x_traj_sens + squeeze(sens_x(j,:,:))*step(j);
        u_traj_sens = u_traj_sens + squeeze(sens_u(j,:,:))*step(j);
    end
    
    x_traj_aug(:,(i-1)*(N+1)+1:i*(N+1)) = x_traj_sens;
    u_traj_aug(:,(i-1)*N+1:i*N) = u_traj_sens;

end

end