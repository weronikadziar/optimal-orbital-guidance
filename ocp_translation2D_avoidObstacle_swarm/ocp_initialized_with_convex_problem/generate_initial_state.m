function x0 = generate_initial_state(na)

% Pick a random position that's at least some distance away from other
% agents and from the central hub
n_dim = 2;
space_size = 100;
dist_min = [repmat(3,na*(na-1),1); repmat(20, na, 1)];
dist_list = zeros(na^2,1);
done = 0;
while done == 0
    i = 1;
    pos = rand(n_dim,na)*space_size-space_size/2;
    for agent = 1:na
        pos_agent = pos(:,agent);
        for neig = 1:na
            if neig ~= agent
                pos_neig = pos(:,neig);
                dist_list(i) = norm(pos_agent - pos_neig);
                i = i + 1;
            end
        end
    end
    dist_list(i:end) = vecnorm(pos,2,1)';
    if dist_list >= dist_min
        done = 1;
    end
end

x0 = [pos; rand(n_dim,na)*0.2-0.1];

end