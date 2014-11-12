%% lab 2 svm test -- setup data section
clear all; clc; % close all; 

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

label_strings = {'Veg', 'Wire', 'Pole', 'Ground', 'Facade'};
label_values = [  1004,   1100,   1103,     1200,    1400];
LABEL_OF_INTEREST = 1004; 

weights = zeros(10,1);
mymargin = 1;    
maxrepeat = 5;
mylimit = 0;
T = numtrain;

% this is just to plot the weights early on, to see convergence-- mostly
% for debugging
plot_weights = zeros(T,10); 

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
        guess_veg_and_it_is     =  guess  &&  LABEL_OF_INTEREST==check;
        guess_veg_and_its_not   =  guess  &&  LABEL_OF_INTEREST~=check;
        guess_not_and_it_is     = ~guess  &&  LABEL_OF_INTEREST==check;
        guess_not_and_its_not   = ~guess  &&  LABEL_OF_INTEREST~=check;

        % store a record of our guess & how we were right/wrong 
        results = [ guess_veg_and_it_is, guess_veg_and_its_not, ...
                    guess_not_and_it_is, guess_not_and_its_not ];
        running_tally = running_tally + results;

        lambda = 1;
        alpha_0 = 1;
        % alpha is exaclty 1/(lambda*t), but if we are repeating the
        % training data set, this is calculating the "true" t, i.e., the
        % number of datapoints over all that we've trained on
        alpha = alpha_0*1/(lambda*(t+(repeat-1)*T));  
            
        % if we were wrong, update our weights!
        if guess_veg_and_its_not || guess_not_and_it_is

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

        % this only stores our weights for the first time through our
        % training data; it's used for plotting the weights, so we can see
        % the convergence, and is mostly for debugging
        if (t+(repeat-1)*T) <= T 
            plot_weights(t,:) = weights';
        end
        
    end % end t = 1:T, going through our training set
    
    % print our results for this round of training data to the screen
    display(['on training data: ' num2str(running_tally) '    % = ' ...
     num2str(100*(running_tally(1)+running_tally(4))/sum(running_tally)) ]) 
end

% just for fun, let's look at our weights 
figure()
    plot(plot_weights); 
    title('weights as a function of time for 1st data training set')
    xlabel('time (i.e., training data points)');
    ylabel('weights')
    hold on

%% testing on whole data set
%{
running_tally = zeros(1,4);

for i = 1:length(A.node_label)

    features = A.features(i,:)';
    veggie = weights'*features;
    
    guess = veggie >= mylimit;
    check = A.node_label(i);
    
        guess_veg_and_it_is     =  guess  &&  LABEL_OF_INTEREST==check;
        guess_veg_and_its_not   =  guess  &&  LABEL_OF_INTEREST~=check;
        guess_not_and_it_is     = ~guess  &&  LABEL_OF_INTEREST==check;
        guess_not_and_its_not   = ~guess  &&  LABEL_OF_INTEREST~=check;
    
    results = [ guess_veg_and_it_is, guess_veg_and_its_not, ...
                guess_not_and_it_is, guess_not_and_its_not ];
    running_tally = running_tally + results;
    
end

display(['on all data:     ' num2str(running_tally) '    % = ' ...
     num2str(100*(running_tally(1)+running_tally(4))/sum(running_tally)) ])    
%}
%% testing on testing set
running_tally = zeros(1,4);

for i = 1:length(A_test.node_label)

    features = A_test.features(i,:)';
    veggie = weights'*features;

    guess = veggie >= mylimit;
    check = A_test.node_label(i);
    
        guess_veg_and_it_is     =  guess  &&  LABEL_OF_INTEREST==check;
        guess_veg_and_its_not   =  guess  &&  LABEL_OF_INTEREST~=check;
        guess_not_and_it_is     = ~guess  &&  LABEL_OF_INTEREST==check;
        guess_not_and_its_not   = ~guess  &&  LABEL_OF_INTEREST~=check;
    
    results = [ guess_veg_and_it_is, guess_veg_and_its_not, ...
                guess_not_and_it_is, guess_not_and_its_not ];
    running_tally = running_tally + results;
    
end

display(['on test data:     ' num2str(running_tally) '    % = ' ...
     num2str(100*(running_tally(1)+running_tally(4))/sum(running_tally)) ]) 
display(num2str(weights'))

% put results on the plot, too, for reference 
text(.5*10^4,1.2,['on test data:     ' num2str(running_tally) '    % = '...
     num2str(100*(running_tally(1)+running_tally(4))/sum(running_tally)) ]) 
axis([0 3.5*10^4 -1 1.5])


