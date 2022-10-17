%% Create OCP object
model = model_translation_3D_convex_constraints();
ocp = acados_ocp_obj(model);

%% Solve
xf = [10; 0; 0; 0; 0; 0];  % SET desired endpoint
N_jobs = 10;               % SET number of paths to generate
N_aug = 10;                % SET number of augmented paths, set to 0 for NO augmentation
[data, data_augmented] = acados_solve(ocp, xf, N_jobs, N_aug);

%% Save datasets
filename = 'data_translation_3D_convex_constraints';      % SET the file name
folder = fileparts(fileparts(cd));
% save(strcat(folder,'/data/',filename,'.mat'), 'data')
% save(strcat(folder,'/data/',filename,'_augmented.mat'), "data_augmented")