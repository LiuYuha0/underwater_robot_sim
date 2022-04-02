function GenerateMovie(flag)
%
% Generate movie or numbered frames according to the flag
%
% GenerateMovie(flag)
%
% input:
%       flag   dim 1       ==1 (to be fixed)uses avifile command from 
%                              matlab and show animation (slow) 
%                          ==0 generate frames/frame%0d.jpg files
%                              in hidden figures
%                              to be used with other sw (fast, default)
%
% output:
%       look under frames/ directory
%
% Yuhao Liu   2019/12/30

if nargin==0
    flag = 0;
end
k = 1;

clc

% ---------------------------------------------------
% load the data
% ---------------------------------------------------
cd ../output
data_name = uigetfile('*.mat', 'select data to play');
if (data_name==0)
    fprintf('\n GenerateMovie.m: no file selected\n');
    cd ../func
    return;
end
eval(['load ' data_name]);
cd ../func

if Ts==0.001;
    my_dec = 100;
else
    my_dec = 20;
end

% ---------------------------------------------------
% some info depending on the input
% ---------------------------------------------------
fprintf('\n\n GenerateMovie \n')
if flag==1
    fprintf('\n now creating avifile for the selected data in frames/simurv_movie.avi')
    fprintf('\n for a faster run try with other flag \n')
    FileName  = 'simurv_movie.avi';
    FrameRate = 10; 
%     myvideo = avifile(FileName,'fps', FrameRate, 'quality', 100, 'compression', 'None');
else
    delete frames/frame*.jpg
    fprintf('\n now creating frames for the selected data in frames/*frame%%04d.jpg')
    fprintf('\n number of points: %d', npti)
    fprintf('\n decimation      : %d', my_dec)
    fprintf('\n number of frames: %d', ceil(npti/my_dec))
    fprintf('\n')
end

% ---------------------------------------------------
% main loop
% ---------------------------------------------------
t_start_sim = clock();
if (flag==1)
    hf = figure;
    set(hf,'Position',[50 30 1000 750]);
    hold on
end
for i = 1:my_dec:npti
    if (flag==0)
        hf = figure('Visible','off','Position',[50 30 1000 750]);
        hold on
    end
    
%     DrawSnap(eta(:,i),[DH(:,1:3) q(:,i)],PARAM,2.1);
    DrawSnap(eta(:,i),[DH(:,1:3) q(1:3,i)],[DH_r(:,1:3) q(4:6,i)],PARAM,2.1,0); 
%     plot3(eta_ee1_d(1,1:i),eta_ee1_d(2,1:i),eta_ee1_d(3,1:i),'g')
%     plot3(eta_ee1(1,1:i),eta_ee1(2,1:i),eta_ee1(3,1:i),'k')
    if (exist('obj_pos','var')==1)
%         DrawCube(obj_pos,obj_size);
    end

    DrawCylinder3([-0.3  0.8 1.192], [-0.3 -0.8 1.192], 0.1, 20,[0.6 0.6 0.6],1,0);
        
    axis tight;
    if (flag==1)
%         myvideo = addframe(myvideo, getframe(gcf)); 
        myvideo = VideoWriter(myvideo, getframe(gcf));
        clf(hf);
    else
        drawnow;
        saveas(gcf,sprintf('../output/fig/frame%04d',k),'jpg');
        hold off
        clf(hf);
    end
    k = k + 1;
    EstimateEndSim(t_start_sim, k, ceil(npti/my_dec));
end
close(hf);
if (flag==1)
    myvideo = close(myvideo); 
else
    fprintf('\n to record a video under Linux, from shell:');
    fprintf('\n avconv -y -r 25 -i frame%%04d.jpg -s 640x480 -b 5000k foo.avi');
end

fprintf('\n');


% ---------------------------------------------------
% generate movie
% ---------------------------------------------------
cd ../output/fig
allnames = struct2cell(dir('*.jpg'));
[~,len]=size(allnames);
aviobj = VideoWriter('example.avi');
aviobj.FrameRate = 30;
open(aviobj)

for i = 1:len
    name = allnames{1,i};
    frame = imread(name);
    writeVideo(aviobj,frame);
end
close(aviobj)

