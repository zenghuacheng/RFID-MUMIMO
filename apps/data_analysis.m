clear;
clc;
close all;

fileID0 = fopen('ant0.bin');
fileID1 = fopen('ant1.bin');
tmp0 = fread(fileID0,  'float');
tmp1 = fread(fileID1,  'float');
% tmp0 = fread(fileID0, 1000000, 'float');
% tmp1 = fread(fileID1, 1000000, 'float');

d = [tmp0(1:2:end) + 1i*tmp0(2:2:end)    tmp1(1:2:end) + 1i*tmp1(2:2:end)];


figure; plot(2*abs(d(:,1)),'r*-');
hold on; plot(abs(d(:,2)),'b*-');


a = 1;