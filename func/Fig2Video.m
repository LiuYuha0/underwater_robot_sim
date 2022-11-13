
clear all;
clc;
srcDic = uigetdir('../output/fig');
cd(srcDic);
allnames = struct2cell(dir('*.jpg'));
[k,len]=size(allnames);
aviobj = VideoWriter('example.avi');
aviobj.FrameRate = 30;
open(aviobj)
for i = 1:len
    name = allnames{1,i};
    frame = imread(name);
    writeVideo(aviobj,frame);
end
close(aviobj)

