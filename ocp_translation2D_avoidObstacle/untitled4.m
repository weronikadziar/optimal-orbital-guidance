% Size
N = 100;
nx = 4;
nu = 2;

% Dynamics
mu = 3.986e14;       % Earth standard gravitational parameter
r0 = 800e3;          % orbit radius
n = sqrt(mu/r0^3);   % mean motion of target satellite
f_thr = 1;           % thrust magnitude
m = 1;               % chaser mass
dt = 1;              % discretization time step
Ad = [4-3*cos(n*dt), 0, 1/n*sin(n*dt), 2/n*(1-cos(n*dt));
      6*(sin(n*dt)-n*dt), 1, -2/n*(1-cos(n*dt)), 1/n*(4*sin(n*dt)-3*n*dt);
      3*n*sin(n*dt), 0, cos(n*dt), 2*sin(n*dt);
      -6*n*(1-cos(n*dt)), 0, -2*sin(n*dt), 4*cos(n*dt)-3];
Bd = f_thr/m*[1/n^2*(1-cos(n*dt)), 2/n^2*(n*dt-sin(n*dt));
              -2/n^2*(n*dt-sin(n*dt)), 4/n^2*(1-cos(n*dt))-3/2*(dt)^2;
              1/n*sin(n*dt), 2/n*(1-cos(n*dt));
              -2/n*(1-cos(n*dt)), 4/n*sin(n*dt)-3*dt];

% Split data into network input and desired output
data_input = data(:,1:nx*1.5);
data_output = data(:,nx*1.5+1:end);

% Allocate arrays for storing data
path_u_net = zeros(2,N*nu);
path_x_prop = zeros(2,N*nx);

% j = randi(size(data,1));
x0 = data_input(j,:);
path_x_net = data_output(j,:);

% Define networks and distributions
net1 = load("C:\Users\Weronika\Documents\GitHub\optimal-orbital-guidance\neural_network_training\neural_network_objects\net_translation2D_avoidObstacle_test.mat").net;
net2 = nn_obj;
net1_distribution = distribution;
net2_distribution = distribution_onlyActions;

% Let network 1 predict the next path
path = predict(net1, x0);
path_denorm = path(1,:).*net1_distribution.sol_sd + net1_distribution.sol_mean;
path_u_net(1,:) = path_denorm(end-N*nu+1:end);

% Let network 2 predict the next path
path = predict(net2, x0);
path_denorm = path(1,:).*net2_distribution.sol_sd + net2_distribution.sol_mean;
path_u_net(2,:) = path_denorm;

% Propagate control inputs to generate the state trajectory
x1 = x0(1:nx)';
x2 = x0(1:nx)';
for i = 1:N
    x1_next = Ad*x1 + Bd*path_u_net(1,(i-1)*nu+1:i*nu)';
    path_x_prop(1,(i-1)*nx+1:i*nx) = x1_next(:)';
    x1 = x1_next;
    x2_next = Ad*x2 + Bd*path_u_net(2,(i-1)*nu+1:i*nu)';
    path_x_prop(2,(i-1)*nx+1:i*nx) = x2_next(:)';
    x2 = x2_next;
end

% Plot state trajectories
figure()
hold on
% Positions
subplot(1,2,1)
hold on
title('Position')
th = 0:pi/50:2*pi;
xunit = 5 * cos(th);
yunit = 5 * sin(th);
plot(xunit, yunit, 'Color','c')    
plot(x0(1),x0(2),'o','LineWidth',2)
plot(x0(5),x0(6),'pentagram','LineWidth',2)    
plot(path_x_net(1:nx:nx*N),path_x_net(2:nx:nx*N),'Color','m')
plot(path_x_prop(1,1:nx:end),path_x_prop(1,2:nx:end),'Color','b')
plot(path_x_prop(2,1:nx:end),path_x_prop(2,2:nx:end),'Color','g')
legend('Obstacle','Initial state','Final state','Predicted state','Predicted propagated state 1','Predicted propagated state 2')
% Velocities
subplot(1,2,2)
hold on
title('Velocity')  
plot(x0(3),x0(4),'o','LineWidth',2)
plot(path_x_net(3:nx:nx*N),'Color','m')
plot(path_x_prop(1,3:nx:end),'Color','b')
plot(path_x_prop(2,3:nx:end),'Color','g')
plot(path_x_net(4:nx:nx*N),'Color','m')
plot(path_x_prop(1,4:nx:end),'Color','b')
plot(path_x_prop(2,4:nx:end),'Color','g')
legend('Initial state','Predicted state','Predicted propagated state 1','Predicted propagated state 2')
