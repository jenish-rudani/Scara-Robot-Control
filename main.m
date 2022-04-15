close all;

filename = "X_vs_Y.csv"

data = csvread(filename,1,0);

plot(data(:,1),data(:,2))
%% 