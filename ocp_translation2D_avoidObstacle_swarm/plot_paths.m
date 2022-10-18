function plot_paths(data_job, data_sol, data_net, N_agents)

% DESCRIPTION
% Plots the initial guess from the network and the solutions found by the 
% OCP solver. Initial and desired final states and clearly marked. 
% Collisions that occur in the initial guess are plotted.


N = 100;
nx = 4;

i = randi(size(data_sol,1));
plt_net = reshape(data_net(i,1:nx*N_agents*N),nx*N_agents,[]);
plt_sol = reshape(data_sol(i,1:nx*N_agents*N),nx*N_agents,[]);

figure()
hold on
title('Example of a converged solution')
pbaspect([1 1 1])
xlim([-60 60])
ylim([-60 60])
set(gcf,'position',[250,120,750,500])

th = 0:pi/50:2*pi;
xunit = 5 * cos(th);
yunit = 5 * sin(th);
plot(xunit, yunit,'Color','m')

plot(data_job(i,1:nx:nx*N_agents),data_job(i,2:4:nx*N_agents),'o','LineWidth',2)
plot(data_job(i,nx*N_agents+1:2:end),data_job(i,nx*N_agents+2:2:end),'pentagram','LineWidth',2)

nc = nchoosek(N_agents,2);
dist = 5;
data = plt_net';
N = size(data,2);
collision_count = 0;
collision_pos = zeros(N*nc,2);
for i = 1:N
    for agent = 1:N_agents   
        pos_agent = data(i,1+(agent-1)*nx:nx/2+(agent-1)*nx);    
        for neig = agent+1:N_agents
            pos_neig = data(i,1+(neig-1)*nx:nx/2+(neig-1)*nx);
            col = norm(pos_agent-pos_neig) < dist;
            if col == 1
                collision_count = collision_count + 1;
                collision_pos(collision_count*2-1,:) = pos_agent;
                collision_pos(collision_count*2,:) = pos_neig;
            end
        end
    end
end
collision_pos = collision_pos(1:collision_count*2,:);
if collision_count > 0
    plot(collision_pos(:,1),collision_pos(:,2),'x','Color','r')
end

for agent = 1:N_agents
    plot(plt_net((agent-1)*nx+1,:),plt_net((agent-1)*nx+2,:),'-b')
    plot(plt_sol((agent-1)*nx+1,:),plt_sol((agent-1)*nx+2,:),'-g')
    drawnow
end

if collision_count > 0
    legend('Obstacle','Initial state','Final state','Initial guess collisions','Initial guess path','Solution path','Location', 'eastoutside')
else
    legend('Obstacle','Initial state','Final state','Initial guess path','Solution path','Location', 'eastoutside')
end

end