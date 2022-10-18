function plot_paths(data_job, data_sol)

% DESCRIPTION
% Plots the solutions found by the OCP solver. Initial and desired final 
% states and clearly marked. 

N = 100;
nx = 4;

i = randi(size(data_sol,1));
plt_sol = reshape(data_sol(i,1:nx*N),nx,[]);

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

plot(data_job(i,1),data_job(i,2),'o','LineWidth',2)
plot(data_job(i,nx+1),data_job(i,nx+2),'pentagram','LineWidth',2)

plot(plt_sol(1,:),plt_sol(2,:),'-g')

legend('Obstacle','Initial state','Final state','Solution path','Location', 'eastoutside')

end