function trainInfo = prepare_data(dataStruct)

% DESCRIPTION
% Outputs a data struct with shuffled data split into training, validation
% and test dataset.

disp('Preparing data...')

% Extract the dataStruct variables
data = dataStruct.data;
outputIndex = dataStruct.outputIndex;
inputIndex = dataStruct.inputIndex;
validationSplitPercent = dataStruct.validationSplitPercent;
testSplitPercent = dataStruct.testSplitPercent;
distribution = dataStruct.distribution;

% Normalize data
data_to_normalize = data(:,outputIndex);
data_normalized = zeros(size(data_to_normalize));
for row = 1:size(data_to_normalize,1)
    data_normalized(row,:) = (data_to_normalize(row,:) - distribution.sol_mean)./distribution.sol_sd;
end
data = [data(:,inputIndex), data_normalized];

% Create training, validation, and test data
totalRows = size(data,1);
numValidationDataRows = floor(validationSplitPercent*totalRows);
numTestDataRows = floor(testSplitPercent*totalRows);
randomIdx = randperm(totalRows,numValidationDataRows + numTestDataRows);
randomData = data(randomIdx,:);    
trainDataIdx = setdiff(1:totalRows,randomIdx);
newValidationData = randomData(1:numValidationDataRows,:);
newValidationInput = newValidationData(:,inputIndex);
newValidationOutput = newValidationData(:,outputIndex);
newTestData = randomData(numValidationDataRows + 1:end,:);
newTrainData = data(trainDataIdx,:);

% Populate trainInfo structure
trainInfo.newTrainInput = newTrainData(:,inputIndex);
trainInfo.newTrainOutput = newTrainData(:,outputIndex);
trainInfo.newValidationCellArray = {newValidationInput, newValidationOutput};
trainInfo.newTestDataInput = newTestData(:,inputIndex);
trainInfo.newTestDataOutput = newTestData(:,outputIndex);

end