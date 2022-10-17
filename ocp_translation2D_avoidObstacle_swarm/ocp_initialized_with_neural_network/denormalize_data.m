function data_denormalized = denormalize_data(data,distribution)

data_denormalized = data;
for row = 1:size(data,1)
    data_denormalized(row,:) = data(row,:).*distribution.sol_sd + distribution.sol_mean;
end

end