% call "multi_run" a bunch of times, in order to run all classes multiple
% times and calculate mean & std_dev of results for the base set of
% features, added features with noise, and added random features

clear all; clc; close all; tic
 
results_0 = zeros(5,8,10);
count = 1;
while count <= 10   
    
    use_noise = 0;
    use_random = 0;
    run multi_run.m
    
    if ~isnan(sum(sum(results)))
        results_0(:,:,count) = results;    
        count = count+1;
    end
    
end
RESULTS_0 = mean(results_0,3);
STD_DEV_0 = std(results_0,0,3);

%%
results_noise = zeros(5,8,10);
count = 1;
while count <= 10   
    
    use_noise = 1;
    use_random = 0;
    run multi_run.m
    
    if ~isnan(sum(sum(results)))
        results_noise(:,:,count) = results;    
        count = count+1;
    end
    
end
RESULTS_NOISE = mean(results_noise,3);
STD_DEV_NOISE = std(results_noise,0,3);

%%
results_random = zeros(5,8,10);
count = 1;
while count <= 10   
    
    use_noise = 0;
    use_random = 1;
    run multi_run.m
    
    if ~isnan(sum(sum(results)))
        results_random(:,:,count) = results;    
        count = count+1;
    end
    
end
RESULTS_RANDOM = mean(results_random,3);
STD_DEV_RANDOM = std(results_random,0,3);

%%
clc

% save('results_SVM.mat', 'RESULTS_0','STD_DEV_0',...
%                         'RESULTS_NOISE','STD_DEV_NOISE',...
%                         'RESULTS_RANDOM','STD_DEV_RANDOM');

toc()        

RESULTS_NOISE(:,[2,6]) - RESULTS_0(:,[2,6])
RESULTS_RANDOM(:,[2,6]) - RESULTS_0(:,[2,6])

max_results_dif_noise_mag   = max(max(RESULTS_NOISE(:,[2,6]) - RESULTS_0(:,[2,6])))
min_results_dif_noise_mag   = min(min(RESULTS_NOISE(:,[2,6]) - RESULTS_0(:,[2,6])))

max_results_dif_random_mag  = max(max(RESULTS_RANDOM(:,[2,6]) - RESULTS_0(:,[2,6])))
min_results_dif_random_mag  = min(min(RESULTS_RANDOM(:,[2,6]) - RESULTS_0(:,[2,6])))



% RESULTS_0
%     0.9596    0.9608    0.7784    0.7938    0.8876    0.8944    0.8859    0.8514
%     0.9877    0.9881    0.3006    0.3474    0.9488    0.9473    0.6094    0.5983
%     0.9912    0.9909    0.7269    0.7304    0.9717    0.9715    0.4941    0.5162
%     0.9886    0.9889    0.9993    0.9861    0.9881    0.9870    0.9858    0.9806
%     0.9618    0.9637    0.8611    0.8667    0.9266    0.9256    0.7082    0.7084
