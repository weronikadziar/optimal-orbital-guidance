function xf = generate_final_state(params)

% DESCRIPTION
% Generates a random position within a ring around the target defined by a 
% maximum radius of max_to_target_final and a minimum radius of 
% min_to_target_final.

% INPUT
% - params: data structure with parameters defining the ring of possible
%           final positions

% OUTPUT
% - xf: final state, desired final velocity is always zero, dimension (nx,1)

nx = 4;

done = 0;
while done == 0
    pos = rand(nx/2,1)*params.max_to_target_final*2-params.max_to_target_final;
    if norm(pos) > params.min_to_target_final && norm(pos) < params.max_to_target_final
        done = 1;
    end
end

xf = [pos; zeros(nx/2,1)];

end