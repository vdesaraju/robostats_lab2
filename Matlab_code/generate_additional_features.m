%% make additional features
clear all; clc;

%% bring in datasets

% set out path
cd ..
cd src/classifiers/data
addpath(genpath(pwd));
cd ../../../Matlab_code

% bring in dataset N
data_file_N = '../src/classifiers/data/oakland_part3_an_rf.node_features';
A_n = importdata(data_file_N,' ',3);

% for ease of use, make A a structure with fields according to:
% x y z node_id node_label [features]
    A_n.x = A_n.data(:,1);
    A_n.y = A_n.data(:,2);
    A_n.z = A_n.data(:,3);
    A_n.node_id = A_n.data(:,4);
    A_n.node_label = A_n.data(:,5);
    A_n.features = A_n.data(:,6:end);

% bring in dataset N
data_file_M = '../src/classifiers/data/oakland_part3_am_rf.node_features';
A_m = importdata(data_file_M,' ',3);

% for ease of use, make A a structure with fields according to:
% x y z node_id node_label [features]
    A_m.x = A_m.data(:,1);
    A_m.y = A_m.data(:,2);
    A_m.z = A_m.data(:,3);
    A_m.node_id = A_m.data(:,4);
    A_m.node_label = A_m.data(:,5);
    A_m.features = A_m.data(:,6:end);

%% make "noised" features

noised_features_n = A_n.features + randn(size(A_n.features));
noised_features_m = A_m.features + randn(size(A_m.features));

%% make random features

% the features are all positive, so they range from 0 to max_val
max_n = max(max(A_n.features));
max_m = max(max(A_m.features));

% now create features randomly from 0 to max_val
random_features_n = rand(size(A_n.features)) * max_n;
random_features_m = rand(size(A_m.features)) * max_m;

%% store the data

% save file to data folder for visualization
my_filepath = '../src/classifiers/data/';

% DONT RUN THE BELOW CODE UNLESS YOU WANT TO OVERWRITE EXISTING DATA. SO
% CHANGE THE FILENAMES OR SOMETHING.

feature_sets = { noised_features_n, noised_features_m, ...
                 random_features_n, random_features_m };
feature_set_names = { 'noised_features_n', 'noised_features_m', ...
                      'random_features_n', 'random_features_m' };
             
% for i = 1:length(feature_sets)
%     dlmwrite( [my_filepath feature_set_names{i} '.node_features'], ...
%                feature_sets{i},'delimiter',' ')
% end











