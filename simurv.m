% SIMURV
%
% Based on 
%
% @BOOK{Antonelli_springer13,
%  author = {G.~Antonelli},
%  title = {Underwater robots. {M}otion and force control of vehicle-manipulator systems},
%  edition = {3rd},
%  publisher = {Springer Tracts in Advanced Robotics, Springer-Verlag},
%  address = {Heidelberg, D},
%  month ={},
%  year = {2013}
%  }
% 
% Gianluca Antonelli, ver 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

clc

fprintf('\n ----------------')
fprintf('\n ----------------')
fprintf('\n -- SIMURV 4.0 --')
fprintf('\n ----------------')
fprintf('\n ----------------')

% take path and "now" to store data
mypath = pwd;
prefix = datestr(now,30);

% ---------------------------------------------------
% load the model and copy the m-file in the folder output
% ---------------------------------------------------
cd data
if (exist('model_name','var')==0)
    model_name = uigetfile('data*.m', 'select a model to simulate');
end
model_copy = strcat(mypath,filesep,'output',filesep,prefix,model_name);
copyfile(model_name,model_copy);
eval(strrep(model_name,'.m',''));
fprintf('\nmodel copied in          %s',[filesep,'output',filesep,prefix,model_name]);
cd ..

% ---------------------------------------------------
% output name to be copied in the folder output
% ---------------------------------------------------
% output_name = strcat(mypath,filesep,'output',filesep,prefix,'out.mat'); 
% fprintf('\noutput will be copied in %s',[filesep,'output',filesep,prefix,'out.mat']);
output_name = strcat('demo',prefix,'out.mat');  % 2020.2.6
fprintf('\noutput will be copied in %s',[filesep,'output',filesep,prefix,'out.mat']);


% ---------------------------------------------------
% run the "core" simulation file and copy the m-file in the folder output
% ---------------------------------------------------
cd func
if (exist('core_simulator_name','var')==0)
    core_simulator_name = uigetfile('core*.m', 'select the core simulation file');
end
core_simulator_copy = strcat(mypath,filesep,'output',filesep,prefix,core_simulator_name);
copyfile(core_simulator_name,core_simulator_copy);
fprintf('\ncore_simulator in        %s',[filesep,'output',filesep,prefix,core_simulator_name]);
eval(strrep(core_simulator_name,'.m',''));
cd ..

% ---------------------------------------------------
% save simulation output in a mat file
% ---------------------------------------------------
cd output
eval(['save ',output_name]);
cd ..

% ---------------------------------------------------
% warning for too much files in the output dir
% ---------------------------------------------------
file_number = size(dir('output'),1)-2;
if file_number>30
    fprintf('\n %d files are currently stored in the output folder, consider erase some',file_number);
    fprintf('\n to erase all type: delete output/2*');
end
    

% ---------------------------------------------------
% exiting stuff
% ---------------------------------------------------
fprintf('\n\n to run another simulation with same model and controller type: simurv[enter]');
fprintf('\n to modify the model type: clear model_name, simurv[enter]');
fprintf('\n to modify the controller type: clear core_simulator_name, simurv[enter]');
fprintf('\n\n');
clear mypath prefix model_copy core_simulator_copy


