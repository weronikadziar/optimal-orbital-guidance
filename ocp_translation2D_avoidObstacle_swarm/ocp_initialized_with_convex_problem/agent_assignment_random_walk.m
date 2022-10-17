function data = agent_assignment_random_walk(ocp, ocp_init, N_jobs)

N = ocp.opts_struct.param_scheme_N;
nx = 4;     % for a single agent
nu = 2;     % for a single agent
na = 4;
N_rand_steps = 100;

xf_list = [5  0 -5  0;      % all docking positions
           0  5  0 -5;
           0  0  0  0;
           0  0  0  0];

v = 1:4;
orders = perms(v);
p = length(orders);

fprintf('\nInitializing path planning...\n')

data = zeros(N*N_jobs,(nx+nu)*na+1);

tic

for job = 1:N_jobs

    data_perms = zeros(p*N,(nx+nu)*na);
    cost_perms = inf*ones(p,1); 
    x0 = generate_initial_state(na);

    for i = 1:24

        order = orders(i,:);
        xf = xf_list(:,order);

        % First plan paths for all agents separetely for initialization
        init_x = zeros(nx*na,N+1);
        init_u = zeros(nu*na,N);
    
        for agent = 1:na
            ocp_init.set('constr_x0', x0(:,agent));
            ocp_init.set('cost_y_ref_e', xf(:,agent));
            ocp_init.set('init_x', zeros(nx,N+1));
            ocp_init.set('init_u', zeros(nu,N));
            ocp_init.solve();
            init_x(1+(agent-1)*nx:agent*nx,:) = ocp_init.get('x');
            init_u(1+(agent-1)*nu:agent*nu,:) = ocp_init.get('u');
        end
    
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
        cost = ocp_init.get_cost();
 
        if status == 0
            data_perms((i-1)*N+1:i*N,:) = [x_traj(:,1:end-1)', u_traj'];
            cost_perms(i) = cost;
        end

    end

    [min_cost, idx] = min(cost_perms);
    data((job-1)*N+1:job*N,1:end-1) = data_perms((idx-1)*N+1:idx*N,:);
    data((job-1)*N+1:job*N,end) = repmat(min_cost,N,1);

    % Random walk

    

    
    

    
    progress = job/N_jobs*100;
    if rem(progress,5) == 0
        fprintf('%d%% complete\n', round(progress))
    end

end

toc

end




