function [ train_acc, test_acc, test_recall, test_prec, weights] = ...
   SVM_fn( data_file, LABEL_OF_INTEREST, noised_features, random_features )
% set out path
cd ..
cd src/classifiers/data
addpath(genpath(pwd));
cd ../../../Matlab_code

% data_file = '../src/classifiers/data/oakland_part3_am_rf.node_features';
A = importdata(data_file,' ',3);

% noised_features = '../src/classifiers/data/noised_features_m.node_features';
% if "noised_features" or "random_features," supplement feature set
if ~isempty(noised_features)
    noised_features = importdata(noised_features,' ');
    A.data = [ A.data noised_features ];    
end
if ~isempty(random_features)
    random_features = importdata(random_features,' ');
    A.data = [ A.data random_features ];    
end

% for ease of use, make A a structure with fields according to:
% x y z node_id node_label [features]
    A.x = A.data(:,1);
    A.y = A.data(:,2);
    A.z = A.data(:,3);
    A.node_id = A.data(:,4);
    A.node_label = A.data(:,5);
    A.features = A.data(:,6:end);

% next, separate (randomly!) into training and testing data
num_total_data_pts = size(A.data,1);
numtest = floor(num_total_data_pts/10);
idx_rand = randperm(num_total_data_pts);
idx_test = idx_rand(1:numtest);
idx_train = idx_rand(numtest+1:end);
numtrain = length(idx_train);
     
    A_test.data = A.data(idx_test);
    A_test.x = A.x(idx_test);
    A_test.y = A.y(idx_test);
    A_test.z = A.z(idx_test);
    A_test.node_id = A.node_id(idx_test);
    A_test.node_label = A.node_label(idx_test);
    A_test.features = A.features(idx_test,:);
    
    A_train.data = A.data(idx_train);
    A_train.x = A.x(idx_train);
    A_train.y = A.y(idx_train);
    A_train.z = A.z(idx_train);
    A_train.node_id = A.node_id(idx_train);
    A_train.node_label = A.node_label(idx_train);
    A_train.features = A.features(idx_train,:);    
    
%% okay, now lets train

% LABEL_OF_INTEREST = 1004; 
label_strings = {'Veg', 'Wire', 'Pole', 'Ground', 'Facade'};
label_values = [ 1004,   1100,   1103,     1200,    1400];

display(' ')
display( [ 'LABEL_OF_INTEREST = ' ...
           label_strings{label_values == LABEL_OF_INTEREST} ] );

weights = zeros(size(A_train.features,2),1);
mymargin = 1;    
maxrepeat = 5;
mylimit = 0;
T = numtrain;

% the outer loop, using index counter "repeat," passes our training data
% through multiple times, in order to make sure our weights converge /
% learn the training samples. 
for repeat = 1:maxrepeat
    
    myindex = randperm(T); % set up a randomized index of our training data
    running_tally = zeros(1,4);    
    
    % now, train weights on randomized training data
    for t = 1:T
        
        features = A_train.features(myindex(t),:)';

        % predict:
        score = weights'*features;
        guess = score > mylimit;

        % check-- find out what the real label is
        check = A_train.node_label(myindex(t));

        % did we get it right (and how), or wrong (false + or false -?)
        tp =  guess  &&  LABEL_OF_INTEREST==check; % guess_veg_and_it_is 
        fp =  guess  &&  LABEL_OF_INTEREST~=check; % guess_veg_and_its_not
        fn = ~guess  &&  LABEL_OF_INTEREST==check; % guess_not_and_it_is
        tn = ~guess  &&  LABEL_OF_INTEREST~=check; % guess_not_and_its_not
    
        % store a record of our guess & how we were right/wrong 
        results = [ tp, fp, fn, tn ];
        running_tally = running_tally + results;

        lambda = 1;
        alpha_0 = 1;
        % alpha is exaclty 1/(lambda*t), but if we are repeating the
        % training data set, this is calculating the "true" t, i.e., the
        % number of datapoints over all that we've trained on
        alpha = alpha_0*1/(lambda*(t+(repeat-1)*T));  
            
        % if we were wrong, update our weights!
        if fp || fn

            if check==LABEL_OF_INTEREST
                y = +1;
            else
                y = -1;
            end             
             
            if y*weights'*features < mymargin
               weights = weights - alpha*lambda*weights + alpha*y*features;
            else
               weights = weights - alpha*lambda*weights;
            end

        end

    end % end t = 1:T, going through our training set
    
end

tp = running_tally(1);
fp = running_tally(2);
fn = running_tally(3);
tn = running_tally(4);
% print our results for training to the screen
train_acc = (tp+tn)/sum(running_tally);
display(['training accuracy: ' num2str(train_acc) ]) 
    
%% testing on testing set
running_tally = zeros(1,4);

for i = 1:length(A_test.node_label)

    features = A_test.features(i,:)';
    veggie = weights'*features;

    guess = veggie >= mylimit;
    check = A_test.node_label(i);
    
    tp =  guess  &&  LABEL_OF_INTEREST==check; % guess_veg_and_it_is 
    fp =  guess  &&  LABEL_OF_INTEREST~=check; % guess_veg_and_its_not
    fn = ~guess  &&  LABEL_OF_INTEREST==check; % guess_not_and_it_is
    tn = ~guess  &&  LABEL_OF_INTEREST~=check; % guess_not_and_its_not
    
    results = [ tp, fp, fn, tn ];
    running_tally = running_tally + results;
    
end

tp = running_tally(1);
fp = running_tally(2);
fn = running_tally(3);
tn = running_tally(4);

test_acc = (tp+tn)/sum(running_tally);
test_recall = tp/(tp+fn);
test_prec = tp/(tp+fp);

display(['testing accuracy: ' num2str(test_acc) ]) 
display(['testing recall: ' num2str(test_recall) ]) 
display(['testing precision: ' num2str(test_prec) ]) 

display(num2str(weights'))
display(' ')

weights = weights';

end

