i = randi(size(data_job,1));
N = 100;
nx = 4;
nu = 2;
job = data_job(i,:);
path = predict(net, job);
path_denorm = denormalize_data(path,net_distribution);
init_x = [data_job(i,1:nx)', reshape(path_denorm(1:N*nx),nx,N)];
[init_x_smooth, window] = smoothdata(init_x,1,'gaussian',2);

figure()
hold on;

x = init_x(1,:);
y = init_x(2,:);
plot(x,y)

x = init_x_smooth(1,:);
y = init_x_smooth(2,:);
plot(x,y)

plot(data_job(i,1),data_job(i,2),'o')
plot(data_job(i,5),data_job(i,6),'*')

th = 0:pi/50:2*pi;
xunit = 5 * cos(th);
yunit = 5 * sin(th);
plot(xunit, yunit,'Color','m')

legend('path','smooth path','x0','xf','obstacle')