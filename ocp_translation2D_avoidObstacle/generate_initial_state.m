function x0 = generate_initial_state(params)

% DESCRIPTION
% Generates a random position within a ring around the target defined by a 
% maximum radius of max_to_target_initial and a minimum radius of 
% min_to_target_initial. Velocities are generated as random values within
% some limit max_vel_initial.

% INPUT
% - params: data structure with parameters defining the ring of possible
%           initial positions and the maximum initial velocity

% OUTPUT
% - x0: initial state, dimension (nx,1)

nx = 4;

done = 0;
while done == 0
    pos = rand(nx/2,1)*params.max_to_target_initial*2-params.max_to_target_initial;
    if norm(pos) > params.min_to_target_initial
        done = 1;
    end
end

x0 = [pos; rand(nx/2,1)*2*params.max_vel_initial-params.max_vel_initial];

end