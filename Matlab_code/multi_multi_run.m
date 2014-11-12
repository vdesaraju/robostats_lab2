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

save('results_SVM.mat', 'RESULTS_0','STD_DEV_0',...
                        'RESULTS_NOISE','STD_DEV_NOISE',...
                        'RESULTS_RANDOM','STD_DEV_RANDOM');

toc()                    

