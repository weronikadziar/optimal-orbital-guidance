function x0 = generate_initial_state(params)

% Pick a random position that's at least some distance away from other
% agents and from the central hub
nx = 4;
vel_max = 0.1;
n_pairs = nchoosek(params.na,2);

dist_min = [repmat(params.min_btw_agents,n_pairs,1); repmat(params.min_to_target_initial, params.na, 1)];
dist_list = zeros(n_pairs+params.na,1);

done = 0;
while done == 0
    i = 1;
    pos = rand(nx/2,params.na)*params.max_to_target_initial*2-params.max_to_target_initial;
    for agent = 1:params.na
        pos_agent = pos(:,agent);
        for neig = agent+1:params.na
            pos_neig = pos(:,neig);
            dist_list(i) = norm(pos_agent - pos_neig);
            i = i + 1;
        end
    end
    dist_list(i:end) = vecnorm(pos,2,1)';
    if dist_list >= dist_min
        done = 1;
    end
end

x0 = [pos; rand(nx/2,params.na)*vel_max*2-vel_max];

end