function [data_job, data_init_ocp, data_init_net, data_init_net_ocp, data_sol_ocp, data_sol_net] = acados_solve_ocp_and_net(ocp_free_time, ocp_const_time, ocp_cvx, net, distribution, params, N_jobs)

N = ocp_cvx.opts_struct.param_scheme_N;
nx = 4;
nu = 2;

fprintf('\nInitializing path planning...\n')

data_job = zeros(N_jobs,nx+nx/2);
data_init_ocp = zeros(N_jobs,(nx+nu)*N);
data_init_net = zeros(N_jobs,(nx+nu)*N);
data_sol_ocp = zeros(N_jobs,(nx+nu)*N+1);
data_sol_net = zeros(N_jobs,(nx+nu)*N+1);

tic

for job = 1:N_jobs

    x0 = generate_initial_state(params);
    xf = generate_final_state(params);
    data_job(job,:) = [x0; xf(1:nx/2)]';

    % Convex OCP problem
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
    init_x_new = init_x(:,2:end);
    data_init_ocp(job,:) = [init_x_new(:); init_u(:)]';

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
            time = x_traj(end,1);
            x_traj = x_traj(1:nx,2:end);
            data_sol_ocp(job,:) = [time; x_traj(:); u_traj(:)]';
        end
    end

    [init_x, init_u] = plan_initial_path(x0,xf,net,N,nx,nu,distribution);
%     init_x = smoothdata(init_x,2,'gaussian');
%     init_u = smoothdata(init_u,2,'gaussian');
    init_x_new = init_x(:,2:end);
    data_init_net(job,:) = [init_x_new(:); init_u(:)]';
    % Use the network prediction to initialize hub avoidance problem
    ocp_const_time.set('constr_x0', x0);
    ocp_const_time.set('cost_y_ref_e', xf);
    ocp_const_time.set('init_x', init_x);
    ocp_const_time.set('init_u', init_u);
    ocp_const_time.solve();
    init_x = ocp_const_time.get('x');
    init_u = ocp_const_time.get('u'); 
    init_x_new = init_x(:,2:end);
    data_init_net_ocp(job,:) = [init_x_new(:); init_u(:)]';

    status = ocp_const_time.get('status');

    if status == 0
        ocp_free_time.set('constr_lbx', [x0; 0], 0);
        ocp_free_time.set('constr_ubx', [x0; params.max_time], 0);
        ocp_free_time.set('cost_y_ref_e', [xf; 0]);
        ocp_free_time.set('init_x', [init_x; repmat(params.max_time, 1, N+1)]);
        ocp_free_time.set('init_u', init_u);
        ocp_free_time.solve();
        x_traj = ocp_free_time.get('x');
        u_traj = ocp_free_time.get('u');
        init_x_new = init_x(:,2:end);
        data_init_net(job,:) = [init_x_new(:); init_u(:)]';
    
        status = ocp_free_time.get('status');
        if status == 0
            time = x_traj(end,1);
            x_traj = x_traj(1:nx,2:end);
            data_sol_net(conv_net,:) = [time; x_traj(:); u_traj(:)]';
        end
    end
    
    progress = job/N_jobs*100;
    if rem(progress,5) == 0
        fprintf('%d%% complete\n', round(progress))
    end

end

toc

end