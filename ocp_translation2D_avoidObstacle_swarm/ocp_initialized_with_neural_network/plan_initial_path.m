function [init_x, init_u] = plan_initial_path(x0,xf,net_plan,N,nx,nu,na,distribution)

init_x = zeros(nx*na,N+1);
init_u = zeros(nu*na,N);

for agent = 1:na
    job = [x0(:,agent); xf(1:nx/2,agent)]';
    path = predict(net_plan, job);
    path_denorm = denormalize_data(path,distribution);
    path_x = [x0(:,agent), reshape(path_denorm(1:N*nx),nx,N)];
    path_u = reshape(path_denorm(N*nx+1:end),nu,N);
    init_x(1+(agent-1)*nx:agent*nx,:) = path_x;
    init_u(1+(agent-1)*nu:agent*nu,:) = path_u;
end

end