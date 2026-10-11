% RUN_MODEL  Load and run the original single-phase grid-connected MMC model.
% Requirements: MATLAB/Simulink R2024a and Specialized Power Systems.
% Model initialization is embedded in PreLoadFcn and InitFcn.
% This script has not been executed in MATLAB as part of repository packaging.

thisScript = mfilename('fullpath');
repoRoot = fileparts(fileparts(thisScript));
modelName = 'MMC_1ph_Grid_ArmLeaders_EnergyBalancing_v3_1A';
modelFile = fullfile(repoRoot, 'model', [modelName '.slx']);

assert(isfile(modelFile), 'Cannot find model: %s', modelFile);
load_system(modelFile);
fprintf('Model: %s\n', modelFile);
fprintf('Fixed step: %s s | Stop time: %s s\n', ...
    get_param(modelName, 'FixedStep'), get_param(modelName, 'StopTime'));

open_system(modelName);
% The original model specifies StopTime = 1 second.
simOut = sim(modelName);
fprintf('Simulation completed. Result stored in variable simOut.\n');
% Inspect the model Scopes and/or the logging contained in simOut.
