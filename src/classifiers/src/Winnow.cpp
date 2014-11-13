#include <iostream>
#include <ctime>
#include <math.h>
#include <unistd.h>
#include <numeric>

#include <boost/tokenizer.hpp>

#include <classifiers/Winnow.h>


namespace pu = parameter_utils;
using namespace std;


Winnow::Winnow()
{
  weights.ones();
  maxwtf.ones();
  minwtf.ones();
}

Winnow::~Winnow() {}

bool Winnow::initialize(const ros::NodeHandle& n)
{
  std::string name = ros::names::append(n.getNamespace(), "Winnow");

  if (!loadParameters(n))
  {
    ROS_ERROR("%s: failed to load parameters", name.c_str());
    return false;
  }
  
  return true;
}

bool Winnow::loadParameters(const ros::NodeHandle& n)
{
  if (!parameter_utils::get("winnow/train_ratio", train_ratio)) return false;
  if (!parameter_utils::get("winnow/feature_thresh", feature_thresh)) return false;
  if (!parameter_utils::get("winnow/pred_thresh", pred_thresh)) return false;
  if (!parameter_utils::get("winnow/alpha", alpha)) return false;
  
  return true;
}


void Winnow::setData(vector<DataIO::pc_data_t>& data_in)
{
  data = data_in;
  computeMaxf(data);
  std::random_shuffle (data.begin(), data.end());
  if (train_ratio > 0)
    train_end_idx = (unsigned int)round(train_ratio*data.size());
}

void Winnow::train()
{
  vector<DataIO::pc_data_t> d;
  if (train_ratio > 0)
    d = getSubvec(data,0,train_end_idx);
  else
    d = data;

  arma::mat55 confusion;
  confusion.zeros();
  
//   for (unsigned int outer = 0; outer < 2; outer++)
  for (unsigned int idx = 0; idx < d.size(); idx++)
  {
    // Normalized features
    arma::vec::fixed<NUM_FEATURES> f = (d[idx].features-minf)/(maxf-minf);
    // Binary features
    arma::vec::fixed<NUM_FEATURES> binf = arma::zeros<arma::vec>(NUM_FEATURES);
    arma::uvec bindx = find(f>feature_thresh);
    binf(bindx).ones();

    // Get predictions for each class
    arma::vec5 wtf = weights*binf;
    arma::vec::fixed<5> pred = arma::zeros<arma::vec>(5);
    pred(find(wtf>pred_thresh)).ones();

    unsigned int max_pred = as_scalar(arma::find(wtf==arma::max(wtf)));
    unsigned int label_idx;
    
    arma::vec::fixed<5> truth = arma::zeros<arma::vec>(5);
    switch (d[idx].node_label)
    {
      case 1004:
        label_idx = 0;
        break;
      case 1100:
        label_idx = 1;
        break;
      case 1103:
        label_idx = 2;
        break;
      case 1200:
        label_idx = 3;
        break;
      case 1400:
        label_idx = 4;
        break;
    }
    truth(label_idx)=1;


    arma::vec::fixed<5> err = truth - pred;
    for (unsigned int i = 0; i<5; i++)
    {
      arma::uvec::fixed<1> ii;
      ii << i;
      weights(ii,bindx) *= pow(alpha,err(i));

      if (wtf(i) > maxwtf(i) && pred(i)==1)
        maxwtf(i) = wtf(i);

      if (wtf(i) < maxwtf(i) && pred(i)==1)
        minwtf(i) = wtf(i);

    }
    confusion(label_idx,max_pred)++;
  }
  cout <<"maxwtf" << endl << maxwtf.t() << endl;
  cout <<"minwtf" << endl << minwtf.t() << endl;
    
  cout << "Number of training points: " << d.size() << endl << "Final weights [w0 ... w4]:" << endl << weights.t() << endl;
  printf("Confusion matrix (%3.3f accuracy):\n", 100.0*arma::trace(confusion)/d.size());
  cout << confusion;
  arma::vec5 precision = diagvec(confusion)/sum(confusion);
  cout << "Precision per category:" << endl << precision.t();
  arma::vec5 recall = diagvec(confusion)/sum(confusion.t());
  cout << "Recall per category:" << endl << recall.t();
}

void Winnow::test()
{
  vector<DataIO::pc_data_t> d;
  if (train_ratio > 0)
     d = getSubvec(data,train_end_idx+1,data.size());
  else
    d = data;
  
  arma::uvec pred;
  pred.resize(d.size());

  arma::uvec truth;
  truth.resize(d.size());

  arma::uvec err;
  err.resize(d.size());

  arma::vec5 labels;
  labels << 1004 << 1100 << 1103 << 1200 << 1400;

  arma::mat55 confusion;
  confusion.zeros();
  
  for (unsigned int idx = 0; idx < d.size(); idx++)
  {
    // Normalized features
    arma::vec::fixed<NUM_FEATURES> f = (d[idx].features-minf)/(maxf-minf);
    // Binary features
    arma::vec::fixed<NUM_FEATURES> binf = arma::zeros<arma::vec>(NUM_FEATURES);
    arma::uvec bindx = find(f>feature_thresh);
    binf(bindx).ones();

    // Get predictions for each class
    arma::vec5 wtf = weights*binf;
    unsigned int max_pred = as_scalar(arma::find(wtf==arma::max(wtf)));
    pred(idx) = labels(max_pred);
    d[idx].predicted_label = pred(idx);
    truth(idx) = d[idx].node_label;
    err(idx) = (pred(idx) != truth(idx));

    unsigned int label_idx;
    switch (d[idx].node_label)
    {
      case 1004:
        label_idx = 0;
        break;
      case 1100:
        label_idx = 1;
        break;
      case 1103:
        label_idx = 2;
        break;
      case 1200:
        label_idx = 3;
        break;
      case 1400:
        label_idx = 4;
        break;
    }

    confusion(label_idx,max_pred)++;

  }
  data_tested = d;
  printf("Total number of errors: %u\nTotal number tested: %lu\n",sum(err),d.size());
  printf("Accuracy = %3.3f%%\n", (double)(d.size()-sum(err))/(double)d.size()*100.0);

  printf("Confusion matrix (%3.3f accuracy):\n", 100.0*arma::trace(confusion)/d.size());
  cout << confusion;
  arma::vec5 precision = diagvec(confusion)/sum(confusion);
  cout << "Precision per category:" << endl << precision.t();
  arma::vec5 recall = diagvec(confusion)/sum(confusion.t());
  cout << "Recall per category:" << endl << recall.t();
}

vector<DataIO::pc_data_t>& Winnow::getDataTested()
{
  return data_tested;
}

void Winnow::computeMaxf(vector<DataIO::pc_data_t>& d)
{
  for (unsigned int i = 0; i < d.size(); i++)
  {
    for (unsigned int j = 0; j < NUM_FEATURES; j++)
    {
      maxf(j) = fmax(maxf(j), d[i].features(j));
      minf(j) = fmin(minf(j), d[i].features(j));
    }
  }
}


vector<DataIO::pc_data_t> Winnow::getSubvec(const vector<DataIO::pc_data_t>& v,
                                            unsigned int start, unsigned int end)
{
  vector<DataIO::pc_data_t> newVec(v.begin()+start, v.begin()+end);
  return newVec;
}

