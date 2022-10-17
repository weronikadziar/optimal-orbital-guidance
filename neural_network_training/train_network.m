function net = train_network(net, trainInfo, trainOptions, filename)

disp('Initializing network training...')

trainInput = trainInfo.newTrainInput;
trainOutput = trainInfo.newTrainOutput;
validationCellArray = trainInfo.newValidationCellArray;
testDataInput = trainInfo.newTestDataInput;
testDataOutput = trainInfo.newTestDataOutput;

% Options for training neural net
trainOptions.ValidationData = validationCellArray;

% Train Network
net = trainNetwork(trainInput,trainOutput,net,trainOptions);

% Test Network
predictedTestDataOutput = predict(net,testDataInput);

% RMSE calculation
testRMSE = sqrt(sum(mean((testDataOutput - predictedTestDataOutput).^2)));
fprintf('Test Data RMSE = %.2f\n', testRMSE);

% Save neural network object
save(filename,'net')

end