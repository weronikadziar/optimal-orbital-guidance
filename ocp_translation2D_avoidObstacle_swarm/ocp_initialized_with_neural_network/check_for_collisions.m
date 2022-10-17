function [collision_count, collision_pos] = check_for_collisions(data,na,quiet)

nx = 4;
np = 2;
nc = nchoosek(na,2);
dist = 5;

data = data';
N = size(data,2);

collision_count = 0;
collision_pos = zeros(N*nc,2);

for i = 1:N
    for agent = 1:na   
        pos_agent = data(i,1+(agent-1)*nx:np+(agent-1)*nx);    
        for neig = agent+1:na
            pos_neig = data(i,1+(neig-1)*nx:np+(neig-1)*nx);
            col = norm(pos_agent-pos_neig) < dist;
            if col == 1
                collision_count = collision_count + 1;
                collision_pos(collision_count*2-1,:) = pos_agent;
                collision_pos(collision_count*2,:) = pos_neig;
            end
        end
    end

    if quiet == false
        progress = i/N*100;
        if rem(progress,10) == 0
            fprintf('%d%% complete\n', round(progress))
        end
    end
end

collision_pos = collision_pos(1:collision_count*2,:);

end
