function [data_job, data_sol, data_net, data_net_swarm] = acados_solve(ocp, net, net_swarm, params, distribution, distribution_swarm)


N = ocp.opts_struct.param_scheme_N;
N_jobs = params.N_jobs;
nx = 4;     % for a single agent
nu = 2;     % for a single agent
na = params.na;

perm = perms(1:na);
p = length(perm);

fprintf('\nInitializing path planning...\n')

data_job = zeros(N_jobs*p,nx*na*1.5);
data_sol = zeros(N_jobs*p,(nx+nu)*na*N);
data_net = zeros(N_jobs*p,(nx+nu)*na*N);
data_net_swarm = zeros(N_jobs*p,(nx+nu)*na*N);

conv = 0;

tic

for job = 1:N_jobs

    x0 = generate_initial_state(params); 
    xf_list = generate_final_state(params);

    for i = 1:p

        order = perm(i,:);
        xf = xf_list(:,order);

        % First plan paths for all agents separetely for initialization
        [init_x, init_u] = plan_initial_path(x0,xf,net,N,nx,nu,na,distribution);
    
        % Now use these trajectories to initialize the swarm problem
        ocp.set('constr_x0', x0(:));
        ocp.set('cost_y_ref_e', xf(:));
        ocp.set('init_x', init_x);
        ocp.set('init_u', init_u);
    
        % Solve
        ocp.solve();
        x_traj = ocp.get('x');
        u_traj = ocp.get('u');
        status = ocp.get('status');
    
        if status == 0
            conv = conv + 1;
            xf_pos = xf(1:nx/2,:);
            data_job(conv,:) = [x0(:); xf_pos(:)]';
            x_traj_sol = x_traj(:,2:end);
            data_sol(conv,:) = [x_traj_sol(:); u_traj(:)]';
            init_x_flat = init_x(:);
            data_net(conv,:) = [init_x_flat(nx*na+1:end);init_u(:)]';
            xf_pos = xf(1:nx/2,:);
            path = predict(net_swarm,[x0(:); xf_pos(:)]');
            path_denorm = denormalize_data(path,distribution_swarm);
            data_net_swarm(conv,:) = path_denorm;
        end

    end
    
    progress = job/N_jobs*100;
    if rem(progress,1) == 0
        fprintf('%d%% complete\n', round(progress))
    end

end

toc

data_job = data_job(1:conv,:);
data_sol = data_sol(1:conv,:);
data_net = data_net(1:conv,:);
data_net_swarm = data_net_swarm(1:conv,:);

end




