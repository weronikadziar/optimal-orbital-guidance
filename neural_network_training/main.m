% Create a neural network architecture and train it using the provided
% dataset. Meant for behavioral cloning of the OCP policies. All the
% hyperparameters can be changed in this file. The dataset should contain
% flattened state and input trajectories. Entries 1:numInputs includes the
% initial states and the desired final positions. The remaining entries are
% the full state and input trajectories from the OCP solver.

% Define neural network architecture
numInputs = 6;
numOutputs = 600;
hiddenLayerSize = 500;
net = [featureInputLayer(numInputs,'Normalization','none','Name','observation')
       fullyConnectedLayer(hiddenLayerSize,'Name','fc1')
       reluLayer('Name','relu1')
       fullyConnectedLayer(hiddenLayerSize,'Name','fc2')
       reluLayer('Name','relu2')
       fullyConnectedLayer(hiddenLayerSize,'Name','fc3')
       reluLayer('Name','relu3')
       fullyConnectedLayer(hiddenLayerSize,'Name','fc4')
       reluLayer('Name','relu4')  
       fullyConnectedLayer(numOutputs,'Name','fcLast')
       regressionLayer('Name','routput')];

% Define training options
trainOptions = trainingOptions('adam', ...
                               'Plots', 'training-progress', ...
                               'OutputNetwork', 'best-validation-loss', ...
                               'ValidationFrequency', 5000, ...
                               'MaxEpochs', 10, ...
                               'MiniBatchSize', 64, ...
                               'InitialLearnRate', 0.0001);

% Define data and data split 
dataStruct = struct();
dataStruct.data = data; 
dataStruct.distribution = distribution;
dataStruct.validationSplitPercent = 0.2;
dataStruct.testSplitPercent = 0.1;
dataStruct.inputIndex = 1:numInputs;
dataStruct.outputIndex = numInputs+1:numInputs+numOutputs;
trainInfo = prepare_data(dataStruct);

% Train and test
filename = 'net.mat';
net = train_network(net, trainInfo, trainOptions, filename);



