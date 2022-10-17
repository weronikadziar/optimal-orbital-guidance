nx = 4;
N = 100;
collisions = [];

data_to_verify = data_flat;

for i = 1:length(data_to_verify)  
    pos_x = data_to_verify(i,1:nx:(N+1)*nx); 
    pos_y = data_to_verify(i,2:nx:(N+1)*nx);    
    col = norm([pos_x;pos_y]) < 5;
    collisions = [collisions; col];

end

disp(nnz(collisions))
