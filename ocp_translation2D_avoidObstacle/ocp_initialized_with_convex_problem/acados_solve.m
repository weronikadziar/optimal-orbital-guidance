function [data_job, data_sol] = acados_solve(ocp, ocp_init, params, N_jobs, N_aug)

N = ocp.opts_struct.param_scheme_N;
nx = 4;
nu = 2;

fprintf('\nInitializing path planning...\n')

data_job = zeros(N_jobs,nx+nx/2);
data_sol = zeros(N_jobs,(nx+nu)*N);
% data_augmented = [];

tic

conv = 0;

for job = 1:N_jobs

    x0 = generate_initial_state(params);
    ocp.set('constr_x0', x0);
    ocp_init.set('constr_x0', x0);
    xf = generate_final_state(params);
    ocp.set('cost_y_ref_e', xf);
    ocp_init.set('cost_y_ref_e', xf);

    % Initialization
    ocp_init.solve();
    init_x = ocp_init.get('x');
    init_u = ocp_init.get('u');
    ocp.set('init_x', init_x);
    ocp.set('init_u', init_u);

    % Slack variables
%     cost_Zl = 1e5;
%     cost_Zu = 0;
%     cost_zl = 0;
%     cost_zu = 0;
%     ocp.set('cost_Zl', cost_Zl);
%     ocp.set('cost_Zu', cost_Zu);
%     ocp.set('cost_zl', cost_zl);
%     ocp.set('cost_zu', cost_zu);

    % Solve
    ocp.solve();
    x_traj = ocp.get('x');
    u_traj = ocp.get('u');
    status = ocp.get('status');

    if status == 0
        conv = conv + 1;
        data_job(conv,:) = [x0; xf(1:nx/2)]';
        x_traj_flat = x_traj(:);
        data_sol(conv,:) = [x_traj_flat(nx+1:end); u_traj(:)]';
    end

%     if N_aug > 0 && status == 0
%         [x_traj_aug, u_traj_aug] = augment_data(ocp, N_aug);
%         data_augmented = vertcat(data_augmented, [x_traj_aug; u_traj_aug]');
%     end
    
    progress = job/N_jobs*100;
    if rem(progress,5) == 0
        fprintf('%d%% complete\n', round(progress))
    end

end

data_job = data_job(1:conv,:);
data_sol = data_sol(1:conv,:);

toc

end