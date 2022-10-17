pos = data(1:100*16,1:2);

figure()
hold on

for i = 1:16
    plot(pos(100*(i-1)+1:100*i,1),pos(100*(i-1)+1:100*i,2),'-')
    hold on
    drawnow

end