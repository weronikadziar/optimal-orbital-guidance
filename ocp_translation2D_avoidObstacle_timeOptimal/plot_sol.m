%% Plot solution

i = randi(size(data_job,1));
plt_1 = data_init_ocp(i,1:400);
plt_2 = reshape(data_init_net(i,1:400),4,100);
% [plt_2,window] = smoothdata(plt_2,2,'gaussian');
plt_job = data_job(i,[1:2,5:6]);
nx = 4;

figure()
hold on;

x = plt_1(1:nx:end);
y = plt_1(2:nx:end);
plot(x,y)

x = plt_2(1,:);
y = plt_2(2,:);
plot(x,y)

plot(plt_job(1),plt_job(2),'x')
plot(plt_job(3),plt_job(4),'*')

th = 0:pi/50:2*pi;
xunit = 5 * cos(th);
yunit = 5 * sin(th);
plot(xunit, yunit,'Color','m')

% legend('sol','x0','xf','obstacle')
legend('ocp','net','x0','xf','obstacle','Location', 'eastoutside')