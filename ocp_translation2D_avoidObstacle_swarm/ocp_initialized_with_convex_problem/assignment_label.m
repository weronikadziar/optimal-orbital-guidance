function [one_hot, class] = assignment_label(docking_order)

na = 4;
v = 1:na;
perm = perms(v);

one_hot = zeros(1,factorial(na));

for i = 1:24
    if isequal(docking_order, perm(i,:))
        class = i;
        one_hot(i) = 1;
        break
    end
end


end