function x0 = generate_initial_state()

q = rand(1,4);
q = q/norm(q);
R = quat2dcm(q);
x0 = [R(:); randn(3,1)*deg2rad(3)];

end