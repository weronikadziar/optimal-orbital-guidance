function xf = generate_final_state(params)

% Pick a random position that's within a donut around the target and at 
% least some distance away from other agents

nx = 4;
vel_max = 0.1;
n_pairs = nchoosek(params.na,2);

dist_min = [repmat(params.min_btw_agents,n_pairs,1); repmat(params.min_to_target_final,params.na,1)];
dist_max = [repmat(100,n_pairs,1); repmat(params.max_to_target_final,params.na,1)];
dist_list = zeros(n_pairs+params.na,1);

done = 0;
while done == 0
    i = 1;
    pos = rand(nx/2,params.na)*params.max_to_target_final*2-params.max_to_target_final;
    for agent = 1:params.na
        pos_agent = pos(:,agent);
        for neig = agent+1:params.na
            pos_neig = pos(:,neig);
            dist_list(i) = norm(pos_agent - pos_neig);
            i = i + 1;
        end
    end
    dist_list(i:end) = vecnorm(pos,2,1)';
    if all(dist_list >= dist_min) && all(dist_list <= dist_max)
        done = 1;
    end
end

xf = [pos; rand(nx/2,params.na)*vel_max*2-vel_max];

end