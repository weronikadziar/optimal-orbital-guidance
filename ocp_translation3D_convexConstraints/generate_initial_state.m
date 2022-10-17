function x0 = generate_initial_state()

% Pick a random position that's at least 'distance' away from the hub
n_dim = 3;
space_size = 100;
distance = 20;

done = 0;
while done == 0
    pos = rand(n_dim,1)*space_size-space_size/2;
    if norm(pos) > distance
        done = 1;
    end
end

x0 = [pos, rand(n_dim,1)*0.2-0.1];

end