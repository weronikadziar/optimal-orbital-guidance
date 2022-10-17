function [data, data_init_x, ocp] = acados_solve(ocp, ocp_init, N_jobs, na)

N = ocp.opts_struct.param_scheme_N;
nx = 4;     % for a single agent
nu = 2;     % for a single agent

cost_y_ref_e = [5  0 -5  0;      % all docking positions
                0  5  0 -5;
                0  0  0  0;
                0  0  0  0];
cost_y_ref_e = cost_y_ref_e(:,1:na);  % pick as many docking points as agents
ocp.set('cost_y_ref_e', cost_y_ref_e(:));

fprintf('\nInitializing path planning...\n')

data = [];
data_init_x = [];

tic

for job = 1:N_jobs

    x0 = generate_initial_state(na);

    % First plan paths for all agents separetely for initialization
    init_x = zeros(nx*na,N+1);
    init_u = zeros(nu*na,N);

    for agent = 1:na
        ocp_init.set('constr_x0', x0(:,agent));
        ocp_init.set('cost_y_ref_e', cost_y_ref_e(:,agent));
        ocp_init.set('init_x', zeros(nx,N+1));
        ocp_init.set('init_u', zeros(nu,N));
        ocp_init.solve();
        init_x(1+(agent-1)*nx:agent*nx,:) = ocp_init.get('x');
        init_u(1+(agent-1)*nu:agent*nu,:) = ocp_init.get('u');
    end

    % Now use these trajectories to initialize the swarm problem
    ocp.set('constr_x0', x0(:));
    ocp.set('init_x', init_x);
    ocp.set('init_u', init_u);

    % Solve
    ocp.solve();
    x_traj = ocp.get('x');
    u_traj = ocp.get('u');
    status = ocp.get('status');

    if status == 0
        data = vertcat(data, [x_traj(:,1:end-1); u_traj]');
        data_init_x = vertcat(data_init_x, init_x(:,1:end-1)');
    end
    
    progress = job/N_jobs*100;
    if rem(progress,5) == 0
        fprintf('%d%% complete\n', round(progress))
    end

end

toc

end




