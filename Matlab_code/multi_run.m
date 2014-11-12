% clear all; clc; close all;
 
label_values = [ 1004,   1100,   1103,     1200,    1400];
 
% use_noise = 0;
% use_random = 0;

results = nan(length(label_values),8);
weights_results = nan(2*length(label_values),10*(use_noise+use_random+1));

%%
data_file = '../src/classifiers/data/oakland_part3_am_rf.node_features';

if use_noise
    noised_features = '../src/classifiers/data/noised_features_m.node_features';
else
    noised_features = [];
end

if use_random
    random_features = '../src/classifiers/data/random_features_m.node_features';
else
    random_features = [];
end

display('data_file m')
for i = 1:length(label_values)
    LABEL_OF_INTEREST = label_values(i);
    [ train_acc, test_acc, test_recall, test_prec, weights] = ...
   SVM_fn( data_file, LABEL_OF_INTEREST, noised_features, random_features);
    weights_results(i,:) = weights;
    results(i,1:4) = [ train_acc test_acc test_recall test_prec ];
end

%%
data_file = '../src/classifiers/data/oakland_part3_an_rf.node_features';

if use_noise
    noised_features = '../src/classifiers/data/noised_features_n.node_features';
else
    noised_features = [];
end

if use_random
    random_features = '../src/classifiers/data/random_features_n.node_features';
else
    random_features = [];
end

display('data_file n')
for j = 1:length(label_values)
    LABEL_OF_INTEREST = label_values(j);
    [ train_acc, test_acc, test_recall, test_prec, weights] = ...
   SVM_fn( data_file, LABEL_OF_INTEREST, noised_features, random_features);
    weights_results(i+j,:) = weights;
    results(j,5:8) = [ train_acc test_acc test_recall test_prec ];
end

printme = {'Veg', 'Wire', 'Pole', 'Ground', 'Facade'}';
printme(:,2:9) = num2cell(double(vpa(sym(results),4)));
display(printme)












%{

data_file m
 
LABEL_OF_INTEREST = Veg
training accuracy: 0.96029
testing accruacy: 0.96304
testing recall: 0.76654
testing precision: 0.79542
-0.0049486  -0.0044446   0.0042501    0.015467    0.019241    0.029441   0.0046875  -0.0037392   -0.062206    0.006323
 
LABEL_OF_INTEREST = Wire
training accuracy: 0.98826
testing accruacy: 0.98909
testing recall: 0.24324
testing precision: 0.3
-0.00029224  0.00052035 -0.00043197  0.00023119   0.0037072  -0.0044119  0.00064351 -0.00018264  -0.0055509  -0.0017815
 
LABEL_OF_INTEREST = Pole
training accuracy: 0.99276
testing accruacy: 0.99299
testing recall: 0.75806
testing precision: 0.74016
-0.0013451   0.0010079 -0.00030381    0.014621   -0.008584  -0.0080225   -0.012119 -0.00045753  -0.0033434  -0.0055235
 
LABEL_OF_INTEREST = Ground
training accuracy: 0.99587
testing accruacy: 0.99588
testing recall: 0.9997
testing precision: 0.99477
0.013324  0.00041754  -0.0089453    -0.41001     0.14389     0.45841    -0.28739     -1.4971     0.31345   -0.037784
 
LABEL_OF_INTEREST = Facade
training accuracy: 0.9646
testing accruacy: 0.96538
testing recall: 0.87715
testing precision: 0.86932
0.0012218  0.00085714   -0.002875  -0.0066294  -0.0043363   -0.054397    0.034324   0.0020176    0.014387    -0.02481
data_file n
 
LABEL_OF_INTEREST = Veg
training accuracy: 0.88061
testing accruacy: 0.8942
testing recall: 0.91724
testing precision: 0.83386
-0.0053046  -0.0043794   0.0064352   0.0010788  0.00049481   0.0063729 -0.00025652 -0.00033802   -0.012294   0.0036429
 
LABEL_OF_INTEREST = Wire
training accuracy: 0.94823
testing accruacy: 0.94477
testing recall: 0.4856
testing precision: 0.60825
0.00091136   0.0062431  -0.0035979   -0.029487   0.0082742   -0.012775   0.0011385 -0.00033858   -0.020423   0.0011994
 
LABEL_OF_INTEREST = Pole
training accuracy: 0.97362
testing accruacy: 0.97417
testing recall: 0.57522
testing precision: 0.58559
-0.0056193 -0.00095208   0.0033604    0.055657   -0.019567   -0.044782   -0.010186  -0.0020979   -0.032252   -0.026673
 
LABEL_OF_INTEREST = Ground
training accuracy: 0.98953
testing accruacy: 0.98736
testing recall: 0.98573
testing precision: 0.98153
0.0089222  -0.0044053  -0.0033402   -0.036045    -0.10552      0.1264   -0.059561    -0.41585     0.10955    0.012893
 
LABEL_OF_INTEREST = Facade
training accuracy: 0.92744
testing accruacy: 0.9258
testing recall: 0.71242
testing precision: 0.70323
0.0029593   0.0022117 -0.00087968   -0.021128 -0.00030807   -0.075357    0.048398  -0.0041085  -0.0098148  -0.0058773   

%}

 
% printme = 
% 
% 
%     'Veg'       [0.9458]    [0.9441]    [0.6646]    [0.7009]
%     'Wire'      [0.9910]    [0.9901]    [     0]    [   NaN]
%     'Pole'      [0.9929]    [0.9922]    [0.8311]    [0.7321]
%     'Ground'    [0.9802]    [0.9805]    [     1]    [0.9748]
%     'Facade'    [0.9590]    [0.9624]    [0.8644]    [0.8617]
% 
% 
%     'Veg'       [0.8809]    [0.8772]    [0.8426]    [0.8420]
%     'Wire'      [0.9454]    [0.9472]    [0.5868]    [0.6068]
%     'Pole'      [0.9717]    [0.9681]    [0.5965]    [0.4928]
%     'Ground'    [0.9842]    [0.9841]    [0.9789]    [0.9803]
%     'Facade'    [0.9217]    [0.9286]    [0.6909]    [0.7229]
%     
% weights_results
%    -0.0146   -0.0020    0.0112    0.0170    0.0429    0.0054    0.0223   -0.0060   -0.1007    0.0062
%     0.0030   -0.0096    0.0009   -0.0766   -0.1090   -0.1721    0.1833   -0.4622   -0.1452   -0.0067
%    -0.0027    0.0029   -0.0010    0.0413   -0.0417   -0.0215   -0.0279   -0.0013   -0.0046   -0.0151
%     0.0031    0.0230   -0.0024   -0.0388    0.3403    0.1340   -0.1573   -0.5831    0.3291    0.2401
%     0.0014    0.0016   -0.0037    0.0157    0.0060   -0.0745    0.0580    0.0004   -0.0290   -0.0357
% 
%    -0.0056   -0.0075    0.0066    0.0042    0.0032    0.0115    0.0011   -0.0005   -0.0167    0.0012
%    -0.0041    0.0059   -0.0062   -0.0819    0.0745   -0.0420    0.0184    0.0004   -0.0766   -0.0514
%    -0.0038    0.0049    0.0002    0.0695   -0.0436   -0.0645   -0.0128   -0.0041   -0.0506   -0.0168
%     0.0026    0.0069   -0.0014   -0.0160   -0.0055    0.0447   -0.0830   -0.7498    0.1102    0.0273
%     0.0029    0.0016   -0.0029   -0.0029   -0.0055   -0.0913    0.0685   -0.0052   -0.0111   -0.0198

