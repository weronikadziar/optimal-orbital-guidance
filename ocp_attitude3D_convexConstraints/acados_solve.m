function data = acados_solve(ocp, xf, N_start, N_rand_steps)

N = ocp.opts_struct.param_scheme_N;
nx = 12;
nu = 3;

ocp.set('cost_y_ref_e', xf);

fprintf('\nInitializing path planning...\n')

data = [];

it_max = 10000;
it_tot = 0;
n_conv = 0;

tic

for job = 1:N_start

    % Find new starting point
    status = 1;
    it = 0;
    while status > 0 && it < it_max
        x0 = generate_initial_state();
        ocp.set('constr_x0', x0);
        init_x = zeros(nx,N+1);
        init_u = zeros(nu,N);
        ocp.set('init_x', init_x);
        ocp.set('init_u', init_u);
        ocp.solve();
        status = ocp.get('status');
        it = it + 1;
        it_tot = it_tot + 1;
    end

    if it == it_max
        disp('No feasible solution found')
        return
    end
    
    x_traj = ocp.get('x');
    u_traj = ocp.get('u');
    data = vertcat(data, [x_traj(:,1:end-1); u_traj]');

    % Make random steps and use sentitivity to initialize OCPs
    for i = 1:N_rand_steps
        [init_x, init_u] = augment_data(ocp, 1);
        x0 = init_x(:,1);
        ocp.set('constr_x0', x0);
        ocp.set('init_x', init_x);
        ocp.set('init_u', init_u);
        ocp.solve();
        status = ocp.get('status');
        if status == 0
            x_traj = ocp.get('x');
            u_traj = ocp.get('u');
            data = vertcat(data, [x_traj(:,1:end-1); u_traj]');
        else
            n_conv = n_conv + 1;
        end
    end
    
    progress = job/N_start*100;
    if rem(progress,2) == 0
        fprintf('%d%% complete\n', round(progress))
    end

end

toc

end




