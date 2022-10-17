function [data, data_augmented] = acados_solve(ocp, xf, N_jobs, N_aug)

N = ocp.opts_struct.param_scheme_N;
nx = 6;
nu = 3;

ocp.set('cost_y_ref_e', xf);

fprintf('\nInitializing path planning...\n')

data = [];
data_augmented = [];

tic

for job = 1:N_jobs

    x0 = generate_initial_state();

    ocp.set('constr_x0', x0);

    % Initialization
    init_x = zeros(nx,N+1);
    init_u = zeros(nu,N);
    ocp.set('init_x', init_x);
    ocp.set('init_u', init_u);

    % Solve
    ocp.solve();
    x_traj = ocp.get('x');
    u_traj = ocp.get('u');
    status = ocp.get('status');

    if status == 0
        data = vertcat(data, [x_traj(:,1:end-1); u_traj]');
    end

    if N_aug > 0 && status == 0
        [x_traj_aug, u_traj_aug] = augment_data(ocp, N_aug);
        data_augmented = vertcat(data_augmented, [x_traj_aug; u_traj_aug]');
    end
    
    progress = job/N_jobs*100;
    if rem(progress,5) == 0
        fprintf('%d%% complete\n', round(progress))
    end

end

toc

end




