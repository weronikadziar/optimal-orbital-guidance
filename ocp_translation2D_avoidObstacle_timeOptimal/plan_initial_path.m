function [init_x, init_u] = plan_initial_path(x0,xf,net_plan,N,nx,nu,distribution)

job = [x0; xf(1:nx/2)]';
path = predict(net_plan, job);
path_denorm = denormalize_data(path,distribution);
init_x = [x0, reshape(path_denorm(1:N*nx),nx,N)];
init_u = reshape(path_denorm(N*nx+1:end),nu,N);

end