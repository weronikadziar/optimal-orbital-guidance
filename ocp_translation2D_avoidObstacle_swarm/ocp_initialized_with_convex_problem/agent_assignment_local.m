function [inputs, outputs] = agent_assignment_local(ocp_init, N_jobs)

N = ocp_init.opts_struct.param_scheme_N;
na = 4;
nx = 4;
nu = 2;

inputs = zeros(N_jobs, na*nx);
outputs = zeros(N_jobs, 1);

dock = [5  0 -5  0;
        0  5  0 -5;
        0  0  0  0;
        0  0  0  0];
tic

for job = 1:N_jobs

    if rem(job/N_jobs*100,5) == 0
        fprintf('%d%% complete\n', job/N_jobs*100)
    end

    x0 = generate_initial_state(na);
    inputs(job,:) = x0(:);
    costs = zeros(4);
    for agent = 1:na
        x0_agent = x0(:, agent);
        for i = 1:na
            xf_agent = dock(:,i);
            ocp_init.set('constr_x0', x0_agent);
            ocp_init.set('cost_y_ref_e', xf_agent);
            ocp_init.set('init_x', zeros(nx,N+1));
            ocp_init.set('init_u', zeros(nu,N));
            ocp_init.solve();
            costs(agent,i) = ocp_init.get_cost();
        end
    end

    % Satellites with highest cost get to pick their docking point first
    costs_assign = costs;
    [~,order] = sort(sum(costs_assign,2),'descend');
    dest_id_list = zeros(1,4);
    for i = 1:4
        sat_id = order(i);
        [~,dest_id] = min(costs_assign(sat_id,:));
        costs_assign(:,dest_id) = inf;
        dest_id_list(sat_id) = dest_id;
    end

    [one_hot, class] = assignment_label(dest_id_list);
    outputs(job) = class;
end 

toc

end