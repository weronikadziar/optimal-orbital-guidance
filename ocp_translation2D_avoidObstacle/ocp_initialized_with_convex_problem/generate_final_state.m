function xf = generate_final_state(dist)

nx = 4;

done = 0;
while done == 0
    pos = rand(nx/2,1)*dist.max_to_target_final*2-dist.max_to_target_final;
    if norm(pos) > dist.min_to_target_final && norm(pos) < dist.max_to_target_final
        done = 1;
    end
end

xf = [pos; zeros(nx/2,1)];

end