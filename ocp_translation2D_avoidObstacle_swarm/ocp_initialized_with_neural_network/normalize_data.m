function [data_normalized, distribution] = normalize_data(data,distribution,na)

if isempty(distribution)
    distribution = get_distribution(data,na);
end

data_normalized = data;
for row = 1:size(data,1)
    data_normalized(row,:) = (data(row,:) - distribution.sol_mean)./distribution.sol_sd;
end

end