% Load network
% net = load("C:\Users\Weronika\Documents\GitHub\optimal-orbital-guidance\ocp_translation2D_avoidObstacle_swarm\net_translation2D_avoidObstacle.mat").net;
% net_distribution = load("C:\Users\Weronika\Documents\GitHub\optimal-orbital-guidance\ocp_translation2D_avoidObstacle_swarm\data_distribution_translation2D_avoidObstacle.mat").net_distribution;

% Load test data
data = load("C:\Users\Weronika\Documents\GitHub\optimal-orbital-guidance\neural_network_training\data\data_test_network.mat").data;

%% Choose testing mode: 
% 0 - run and plot 3 jobs for visualization
% 1 - run all jobs for calculating RMSE
mode = 0;

% net = load("C:\Users\Weronika\Documents\GitHub\optimal-orbital-guidance\neural_network_training\neural_network_objects\net_translation2D_avoidObstacle_test.mat").net;
% net_distribution = get_distribution(train_data);
net = nn_obj;
net_distribution = distribution_onlyActions;
net_distribution.sd = distribution_onlyActions.sol_sd;
net_distribution.mean = distribution_onlyActions.sol_mean;

% Test network
data = data(1:50000,:);
[path_u_net, path_x_prop] = propagate_net_actions(net, net_distribution, data, mode);

% Calculate RMSE
N = 100;
nx = 4;
nu = 2;
if mode == 1
    distribution = load("C:\Users\Weronika\Documents\GitHub\optimal-orbital-guidance\ocp_translation2D_avoidObstacle_swarm\data_distribution_translation2D_avoidObstacle.mat").net_distribution;
    data_norm = (data - distribution.sol_mean)./distribution.sol_sd;
    RMSE_u_net = sqrt(sum(mean((data_norm(:,N*nx+1:end) - path_u_net).^2)));
    RMSE_x_prop = sqrt(sum(mean((data_norm(:,N*nx) - path_x_prop).^2)));
    fprintf('RMSE of networks control trajectory: %.2f\n', RMSE_u_net)
    fprintf('RMSE of networks propagated state trajectory : %.2f\n', RMSE_x_prop)
end

function [path_u_net, path_x_prop] = propagate_net_actions(net, net_distribution, data, mode)

% Number of jobs depends on the chosen mode
if mode == 0
    N_jobs = 3;
elseif mode == 1
    N_jobs = size(data,1);
end

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
path_u_net = zeros(N_jobs,N*nu);
path_x_prop = zeros(N_jobs,N*nx);

disp('Generating paths to test the network...')
tic

for job = 1:N_jobs

    % Let the network predict the next path
    j = mode*job+abs(mode-1)*randi(size(data,1));
    x0 = data_input(j,:);
    path = predict(net, x0);
    path_denorm = path(1,:).*net_distribution.sd + net_distribution.mean;
    path_u_net(job,:) = path_denorm(end-N*nu+1:end);

    % Propagate control inputs to generate the state trajectory
    x = x0(1:nx)';
    for i = 1:N
        x_next = Ad*x + Bd*path_u_net(job,(i-1)*nu+1:i*nu)';
        path_x_prop(job,(i-1)*nx+1:i*nx) = x_next(:)';
        x = x_next;
    end

    if mode == 0
        % Plot state trajectories
        figure(job)
        hold on
        path_x_net = data_output(j,:);
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
        plot(path_x_prop(job,1:nx:end),path_x_prop(job,2:nx:end),'Color','b')
        legend('Obstacle','Initial state','Final state','Predicted state','Predicted propagated state')
        % Velocities
        subplot(1,2,2)
        hold on
        title('Velocity')  
        plot(x0(3),x0(4),'o','LineWidth',2)
        plot(path_x_net(3:nx:nx*N),'Color','m')
        plot(path_x_prop(job,3:nx:end),'Color','b')
        plot(path_x_net(4:nx:nx*N),'Color','m')
        plot(path_x_prop(job,4:nx:end),'Color','b')
        legend('Initial state','Predicted state','Predicted propagated state')
    end

    % Print progress
    progress = job/N_jobs*100;
    if rem(progress,1) == 0
        elapsedTime = toc;
        fprintf('%d%% path generation complete, %.2f s elapsed\n', round(progress), elapsedTime)
    end

end

end

