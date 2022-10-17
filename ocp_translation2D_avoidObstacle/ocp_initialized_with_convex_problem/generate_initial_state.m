function x0 = generate_initial_state(dist)

nx = 4;

done = 0;
while done == 0
    pos = rand(nx/2,1)*dist.max_to_target_initial*2-dist.max_to_target_initial;
    if norm(pos) > dist.min_to_target_initial
        done = 1;
    end
end

vel_max = 0.1;

x0 = [pos; rand(nx/2,1)*2*vel_max-vel_max];

end