clear all;
clc;

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

s = serial('COM3','BaudRate',115200);

fopen(s);
s.Status

%%  Run Sim
%sim('MSE312ProjectGainSch');