%% these are stored weights for data_set n  
% uncomment & run one of these blocks in order to generate a dataset /
% classify nodes, using the code below

% weights_4_veg_n = [...   
%    -0.0084   -0.0109    0.0125    0.0005    0.0004 ...   
%    0.0006   -0.0001     0.0000   -0.0006    0.0004 ]';
% weights = weights_4_veg_n;
% LABEL_OF_INTEREST = 1004;

% weights_4_gound_n = [ ... 
%    0.0058407   -0.011174  -0.0016333    -0.01756    0.068126     ...
%      0.17661    -0.15282    -0.99342      0.2162   0.0078439 ]';
% weights = weights_4_ground_n;
% LABEL_OF_INTEREST = 1200;

% weights_4_facade_n = [ ... 
%       0.0026544   0.0031918  -0.0026067 -0.00074072  0.00032909   ...
%       -0.020126   0.0068431 -6.4919e-05   0.0081587  -0.0061362 ]';
% weights = weights_4_facade_n;
% LABEL_OF_INTEREST = 1400;

% weights_4_wire_n = [ ... 
%     -0.002243   0.0089188  -0.0054574   -0.020674    0.049566   ...
%     -0.051931  -0.0033609 -0.00095364   -0.057061   -0.024386 ]';
% weights = weights_4_wire_n;
% LABEL_OF_INTEREST = 1100;

% weights_4_pole_n = [ ... 
%     -0.00046431  0.00037426  0.00036395   0.0051763  -0.0060921  ...
%     -0.0067542   0.0013873 -0.00028661  -0.0040351  -0.0021636 ]';
% weights = weights_4_pole_n;
% LABEL_OF_INTEREST = 1103;

%% these are stored weights for data_set m  
% uncomment & run one of these blocks in order to generate a dataset /
% classify nodes, using the code below

% weights_4_veg_m = [...   
%     -0.0051943  -0.0042574   0.0049415   0.0033862   0.0049476...
%     0.011813 -3.4956e-05  -0.0018364   -0.029733   0.0096467 ]';
% weights = weights_4_veg_m;
% LABEL_OF_INTEREST = 1004;

% weights_4_wire_m = [...   
%     -0.0038543   0.0027207  -0.0012254   -0.031053    0.057132 ...
%     -0.1017    0.019203  -0.0019528   -0.060957   -0.033442 ]';
% weights = weights_4_wire_m;
% LABEL_OF_INTEREST = 1100;

% weights_4_pole_m = [...   
%     -0.003485   0.0029371  2.2444e-05    0.028879   -0.015071 ...
%     -0.030668   -0.019829  -0.0014537   -0.020771  -0.0050373  ]';
% weights = weights_4_pole_m;
% LABEL_OF_INTEREST = 1103;

% weights_4_ground_m = [...   
%     0.0296  -0.0045056 -0.00072522   -0.090393   -0.022587  ...
%     0.60618    -0.32049     -4.3717     0.52811     0.25588  ]';
% weights = weights_4_ground_m;
% LABEL_OF_INTEREST = 1200;

% weights_4_facade_m = [...   
%     0.001038   0.0015623  -0.0026151  -0.0011077  -0.0030825 ...
%     -0.051124    0.023968   0.0014334    0.013699   -0.018797  ]';
% weights = weights_4_facade_m;
% LABEL_OF_INTEREST = 1400;

% label_strings = {'Veg', 'Wire', 'Pole', 'Ground', 'Facade'};
% label_values = [  1004,   1100,   1103,     1200,    1400];

%% bring in dataset to test on

% set out path
cd ..
cd src/classifiers/data
addpath(genpath(pwd));
cd ../../../Matlab_code

% bring in data-- CHANGE THIS TO THE CORRECT FILE FOR WHAT YOU WANT TO TEST
data_file = '../src/classifiers/data/oakland_part3_am_rf.node_features';
A = importdata(data_file,' ',3);

% for ease of use, make A a structure with fields according to:
% x y z node_id node_label [features]
    A.x = A.data(:,1);
    A.y = A.data(:,2);
    A.z = A.data(:,3);
    A.node_id = A.data(:,4);
    A.node_label = A.data(:,5);
    A.features = A.data(:,6:end);

%% test our weights on the data, & record our classifications

% B is the new data file we are going to be making / recording our
% classifications in, and storing for Vishnu's visualizer code to display
B = A.data;

running_tally = zeros(1,4);
for i = 1:length(A.node_label)

    features = A.features(i,:)';
    veggie = weights'*features;
    
guess_veg_and_it_is    = veggie >= 0 && LABEL_OF_INTEREST==A.node_label(i);
guess_veg_and_its_not  = veggie >= 0 && LABEL_OF_INTEREST~=A.node_label(i);
guess_not_and_it_is    = veggie <  0 && LABEL_OF_INTEREST==A.node_label(i);
guess_not_and_its_not  = veggie <  0 && LABEL_OF_INTEREST~=A.node_label(i);

        results = [ guess_veg_and_it_is, guess_veg_and_its_not, ...
                    guess_not_and_it_is, guess_not_and_its_not ];
        running_tally = running_tally + results;   
    
    if veggie >= 0
        B(i,5) = LABEL_OF_INTEREST;
    else
        B(i,5) = 9999;
    end
    
end

accuracy = 100*(running_tally(1)+running_tally(4))/sum(running_tally);
display(['accuracy of ' num2str(accuracy)])

% ok, now "shrink" dataset B of all the nodes we didn't think were the
% "label of interest," so that we only visualize the points we classified
C = zeros(sum(B(:,5) == LABEL_OF_INTEREST),size(B,2));
idx = 1;
for i = 1:size(B,1)
   
    if B(i,5) == LABEL_OF_INTEREST
        C(idx,:) = B(i,:);
        idx = idx+1;
    end

end

% save file to data folder for visualization
my_filepath = '../src/classifiers/data/';

% UNCOMMENT THE BELOW LINE TO SAVE THE FILE TO THE DATA FOLDER--
% MAKE SURE YOU CHANGE THE FILE NAME BEFORE SAVING

% dlmwrite([my_filepath 'svm_facade_m.node_features'],C,'delimiter',' ')
