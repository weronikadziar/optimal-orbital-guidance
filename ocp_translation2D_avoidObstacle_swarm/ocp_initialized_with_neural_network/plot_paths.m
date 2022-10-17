N = 100;
nx = 4;
nu = 2;
na = params.na;

i = randi(size(data_net,1));
data_plot_1 = reshape(data_net(i,1:nx*na*N),nx*na,[]);
data_plot_2 = reshape(data_sol(i,1:nx*na*N),nx*na,[]);
data_plot_3 = [];
data_plot_3 = reshape(data_net_swarm(i,1:nx*na*N),nx*na,[]);

figure()
hold on
pbaspect([1 1 1])
xlim([-60 60])
ylim([-60 60])
set(gcf,'position',[250,120,750,500])

th = 0:pi/50:2*pi;
xunit = 5 * cos(th);
yunit = 5 * sin(th);
plot(xunit, yunit,'Color','m')

plot(data_job(i,1:nx:nx*na),data_job(i,2:4:nx*na),'o','LineWidth',2)
plot(data_job(i,nx*na+1:2:end),data_job(i,nx*na+2:2:end),'pentagram','LineWidth',2)

[collision_count, collision_pos] = check_for_collisions(data_plot_1,na,true);
if collision_count > 0
    plot(collision_pos(:,1),collision_pos(:,2),'x','Color','r')
end

for agent = 1:na
    plot(data_plot_1((agent-1)*nx+1,:),data_plot_1((agent-1)*nx+2,:),'-b')
    plot(data_plot_2((agent-1)*nx+1,:),data_plot_2((agent-1)*nx+2,:),'-g')
    if isempty(data_plot_3) == 0
        plot(data_plot_3((agent-1)*nx+1,:),data_plot_3((agent-1)*nx+2,:),'-c')
    end
    drawnow
end

if collision_count > 0 && isempty(data_plot_3) == 1
    legend('Obstacle','Initial state','Final state','Initial guess collisions','Initial guess path','Solution path','Location', 'eastoutside')
elseif collision_count > 0 && isempty(data_plot_3) == 0
    legend('Obstacle','Initial state','Final state','Initial guess collisions','Initial guess path','Solution path','Other data','Location', 'eastoutside')
elseif collision_count == 0 && isempty(data_plot_3) == 1
    legend('Obstacle','Initial state','Final state','Initial guess path','Solution path','Location', 'eastoutside')
elseif collision_count == 0 && isempty(data_plot_3) == 0
    legend('Obstacle','Initial state','Final state','Initial guess path','Solution path','Other data','Location', 'eastoutside')
end