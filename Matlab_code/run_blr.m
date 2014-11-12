close all;
clear; clc;

% toggle between 'm' or 'n' to switc between datasets
datachar = 'm';

% read in original data
my_filepath = '../src/classifiers/data/';
A = importdata([my_filepath 'oakland_part3_a' datachar '_rf.node_features'],' ',3);

% this assumes that generate_additional_features.m has already been run
B = importdata([my_filepath 'random_features_' datachar '.node_features'],' ');
C = importdata([my_filepath 'noised_features_' datachar '.node_features'],' ');

combos = {'Veg','Wire','Pole','Ground','Facade'};

% sigma value for initial covariance of theta weight vector
sigma = 1;
frac_train_pts = 0.5;

data = A.data;
data(:,end) = [];

%% train & test on regular dataset
resmat = [];
for ii = 1:numel(combos)
    res = blr(data,combos{ii},sigma,frac_train_pts,datachar,my_filepath);
    resmat = [resmat; res];
end

%% train & test on dataset with appended random features
fprintf('\n Random Features \n');
rdata = [data B];
resrmat = [];
for ii = 1:numel(combos)
    res = blr(rdata,combos{ii},sigma,frac_train_pts,datachar,'');
    resrmat = [resrmat; res];
end

%% train & test on dataset with appended noised features
fprintf('\n Noised Features \n');
ndata = [data C];
resnmat = [];
for ii = 1:numel(combos)
    res = blr(ndata,combos{ii},sigma,frac_train_pts,datachar,'');
    resnmat = [resnmat; res];
end