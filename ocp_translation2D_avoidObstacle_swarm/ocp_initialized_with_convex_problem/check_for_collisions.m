na = 4;
nx = 4;
collisions = [];

data_to_verify = data_init_x;

for i = 1:length(data_to_verify)
    for agent = 1:na   
        pos_agent = data_to_verify(i,1+(agent-1)*nx:2+(agent-1)*nx);    
        for neig = 1:na
            if neig ~= agent
                pos_neig = data_to_verify(i,1+(neig-1)*nx:2+(neig-1)*nx);
                col = norm(pos_agent-pos_neig) < 3;
                collisions = [collisions; col];
            end
        end
    end
end

disp(nnz(collisions))
