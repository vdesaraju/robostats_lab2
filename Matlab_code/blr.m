function results = blr(data, A_str, sigma, frac_train_pts, datachar, writepath)
% data is a matrix of values read in from the provided files
% A_str is one of 'Veg','Wire','Pole','Ground' or 'Facade'
% sigma is the prior on the covariance for the algorithm
% frac_train_pts is the fraction of the total data set used for training
% datachar = {m,n} denotes which dataset we are working with
% writepath is the path where the output text file is dumped
% results = [training accuracy  test accuracy recall precision]

%% Extract data from the two classes we are concerned with
str_label_map = {1004,'Veg'; 1100, 'Wire'; 1103, 'Pole'; ...
                 1200,'Ground'; 1400, 'Facade'};

A_label = str_label_map{strcmp(str_label_map(:,2),A_str),1};

A_ind = find(data(:,5) == A_label);
B_ind = find(data(:,5) ~= A_label);

A_data = data(A_ind,:);
B_data = data(B_ind,:);

% assign label y = +1 to cateogry A and y = -1 to category B
subdata = [ones(numel(A_ind),1) A_data(:,6:end);
          -ones(numel(B_ind),1) B_data(:,6:end)];
      
predata = [A_data(:,1:4); B_data(:,1:4)];      

% random permutation of the data
rpind = randperm(size(subdata,1));

subdata = subdata(rpind,:);
predata = predata(rpind,:);

dim_theta = size(subdata,2)-1;
total_pts = numel(rpind);
num_train_pts = round(frac_train_pts*total_pts);

%% Bayes Linear Regression (Training)
Jvec = zeros(dim_theta,num_train_pts);
Pmat = zeros(dim_theta,dim_theta,num_train_pts);
Pmat(:,:,1) = 1/sigma^2*eye(dim_theta);
muvec = zeros(dim_theta,num_train_pts);
froPvec = zeros(1,num_train_pts);
online_pred = nan(num_train_pts,1);

for kk = 1:num_train_pts
   x = subdata(kk,2:end)';
   y = subdata(kk,1);
   Jvec(:,kk) = Jvec(:,max(1,kk-1)) + y*x/sigma^2;
   Pmat(:,:,kk) = Pmat(:,:,max(1,kk-1)) + x*x'/sigma^2;
   if kk > dim_theta
        froPvec(kk) = norm(Pmat(:,:,kk),'fro');
        muvec(:,kk) = Pmat(:,:,kk) \ Jvec(:,kk);
        online_pred(kk) = sign(muvec(:,kk-1)'*x);
   end
end

% final parameters after training
theta = muvec(:,end);

% vector with 1 for correct prediction and -1 for incorrect prediction
train_results = subdata(dim_theta+1:num_train_pts,1).*online_pred(dim_theta+1:end);
train_acc = sum((train_results+1)/2)/(numel(dim_theta+1:num_train_pts));

fprintf('\n %s vs all \n',A_str);
fprintf('Accuracy on training examples is %f %% \n', train_acc*100);

%% Testing
testdata = subdata(num_train_pts+1:end,:);
poslabel_testdata = predata(num_train_pts+1:end,:);

num_test = size(testdata,1);
testpred = nan(num_test,1);

for kk = 1:num_test
    x = testdata(kk,2:end)';
    testpred(kk) = sign(theta'*x);
end

% vector with 1 for correct prediction and -1 for incorrect prediction
test_results = testdata(:,1).*testpred;
test_acc = sum((test_results + 1)/2)/num_test;

num_true_positives = sum((testpred == 1) & (testdata(:,1) == 1));

% recall = (# true positives) / (# actual positives in dataset)
recall = num_true_positives/sum(testdata(:,1) == 1);

% precision = (# true positives) / (# predicted positives)
precision = num_true_positives/sum(testpred == 1);

fprintf('Accuracy on %d test points is %f %% \n', num_test, test_acc*100);
results = [train_acc test_acc recall precision];

%% write file of results
if ~isempty(writepath)
testpredlabels = testpred;
testpredlabels(testpredlabels == 1,:) = A_label*ones(sum(testpredlabels == 1),1);

M = [poslabel_testdata testpredlabels zeros(numel(testpredlabels),10)];
M((testpredlabels~=A_label),:) = [];
dlmwrite([writepath 'blr_a' datachar '_' A_str '_test_results.txt'],...
          M,'delimiter',' ');
end
end