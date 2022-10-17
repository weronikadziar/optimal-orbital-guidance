function [data_job, data_init, data_sol] = acados_solve(ocp_free_time, ocp_const_time, ocp_cvx, params, N_jobs)

N = ocp_cvx.opts_struct.param_scheme_N;
nx = 4;
nu = 2;

fprintf('\nInitializing path planning...\n')

data_job = zeros(N_jobs,nx+nx/2);
data_init = zeros(N_jobs,(nx+nu)*N);
data_sol = zeros(N_jobs,(nx+nu)*N+1);

conv = 0;

tic

for job = 1:N_jobs

    x0 = generate_initial_state(params);
    xf = generate_final_state(params);

    % Convex problem
    ocp_cvx.set('constr_x0', x0);
    ocp_cvx.set('cost_y_ref_e', xf);
    ocp_cvx.solve();
    init_x = ocp_cvx.get('x');
    init_u = ocp_cvx.get('u');

    % Use the convex solution to initialize hub avoidance problem
    ocp_const_time.set('constr_x0', x0);
    ocp_const_time.set('cost_y_ref_e', xf);
    ocp_const_time.set('init_x', init_x);
    ocp_const_time.set('init_u', init_u);
    ocp_const_time.solve();
    init_x = ocp_const_time.get('x');
    init_u = ocp_const_time.get('u');
    status = ocp_const_time.get('status');

    % If converged, use this to initialize the hub avoidance problem with
    % free final time
    if status == 0
        ocp_free_time.set('constr_lbx', [x0; 0], 0);
        ocp_free_time.set('constr_ubx', [x0; params.max_time], 0);
        ocp_free_time.set('cost_y_ref_e', [xf; 0]);
        ocp_free_time.set('init_x', [init_x; repmat(params.max_time, 1, N+1)]);
        ocp_free_time.set('init_u', init_u);
        ocp_free_time.solve();
        x_traj = ocp_free_time.get('x');
        u_traj = ocp_free_time.get('u');
        status = ocp_free_time.get('status');
        if status == 0
            conv = conv + 1;
            data_job(conv,:) = [x0; xf(1:nx/2)]';
            init_x = init_x(:,2:end);
            data_init(conv,:) = [init_x(:); init_u(:)]';
            time = x_traj(end,1);
            x_traj = x_traj(1:nx,2:end);
            data_sol(conv,:) = [time; x_traj(:); u_traj(:)]';
        end
    end
    
    progress = job/N_jobs*100;
    if rem(progress,5) == 0
        fprintf('%d%% complete\n', round(progress))
    end

end

data_job = data_job(1:conv,:);
data_init = data_init(1:conv,:);
data_sol = data_sol(1:conv,:);

toc

end