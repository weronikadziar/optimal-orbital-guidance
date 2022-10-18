function x0 = generate_initial_state(N_agents)

% DESCRIPTION
% Generates a random position within a ring around the target defined by a 
% maximum radius of max_to_target and a minimum radius of min_to_target.
% Positions are generated such that all agents are at some distance from
% each other defined by min_btw_agents. Velocities are generated as random
% values within some limit max_vel.

% INPUT
% - N_agents

% OUTPUT
% - x0: flattened initial states for all agents

min_btw_agents = 5;             % min distance between all agents [m]
max_to_target = 50;             % max initial distance from the target [m]
min_to_target = 25;             % min initial distance from the target [m]
max_vel = 0.1;                  % max initial velocity in all directions [m/s]
nx = 4;                         % number of states
N_pairs = nchoosek(N_agents,2); % number of agent pairs

dist_min = [repmat(min_btw_agents,N_pairs,1); repmat(min_to_target, N_agents, 1)];
dist_list = zeros(N_pairs+N_agents,1);

done = 0;
while done == 0
    i = 1;
    pos = rand(nx/2,N_agents)*max_to_target*2-max_to_target;
    for agent = 1:N_agents
        pos_agent = pos(:,agent);
        for neig = agent+1:N_agents
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

x0 = [pos; rand(nx/2,N_agents)*max_vel*2-max_vel];

end