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
  
  for (unsigned int idx = 0; idx < d.size(); idx++)
  {
    // Normalized features
    arma::vec::fixed<10> f = d[idx].features%maxfinv;
    // Binary features
    arma::vec::fixed<10> binf = arma::zeros<arma::vec>(10);
    arma::uvec bindx = find(f>0.5);
    binf(bindx).ones();

    // Get predictions for each class
    arma::vec5 wtf = weights*binf;
    arma::vec::fixed<5> pred = arma::zeros<arma::vec>(5);
    pred(find(wtf>5)).ones();

    arma::vec::fixed<5> truth = arma::zeros<arma::vec>(5);
    switch (d[idx].node_label)
    {
      case 1004:
        truth(0)=1;
        break;
      case 1100:
        truth(1)=1;
        break;
      case 1103:
        truth(2)=1;
        break;
      case 1200:
        truth(3)=1;
        break;
      case 1400:
        truth(4)=1;
        break;
    }

    arma::vec::fixed<5> err = truth - pred;
    for (unsigned int i = 0; i<5; i++)
    {
      arma::uvec::fixed<1> ii;
      ii << i;
      weights(ii,bindx) *= pow(2,err(i));
    }    
  }
  cout << "Number of training points: " << d.size() << endl << "Final weights [w0 ... w4]:" << endl << weights.t() << endl;
    
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
  
  for (unsigned int idx = 0; idx < d.size(); idx++)
  {
    // Normalized features
    arma::vec::fixed<10> f = d[idx].features%maxfinv;
    // Binary features
    arma::vec::fixed<10> binf = arma::zeros<arma::vec>(10);
    arma::uvec bindx = find(f>0.5);
    binf(bindx).ones();

    // Get predictions for each class
    arma::vec5 wtf = weights*binf;
    pred(idx) = labels(as_scalar(arma::find(wtf==arma::max(wtf))));
    d[idx].predicted_label = pred(idx);
    truth(idx) = d[idx].node_label;
    err(idx) = (pred(idx) != truth(idx));
  }
  data_tested = d;
  printf("Total number of errors: %u\nTotal number tested: %lu\n",sum(err),d.size());
  printf("Accuracy = %3.3f%%\n", (double)(d.size()-sum(err))/(double)d.size()*100.0);
}

vector<DataIO::pc_data_t>& Winnow::getDataTested()
{
  return data_tested;
}

void Winnow::computeMaxf(vector<DataIO::pc_data_t>& d)
{
  arma::vec::fixed<10> maxf;
  for (unsigned int i = 0; i < d.size(); i++)
  {
    for (unsigned int j = 0; j < 10; j++)
    {
      maxf(j) = fmax(maxf(j), d[i].features(j));
    }
  }
  maxfinv = 1.0/maxf;
}


vector<DataIO::pc_data_t> Winnow::getSubvec(const vector<DataIO::pc_data_t>& v,
                                            unsigned int start, unsigned int end)
{
  vector<DataIO::pc_data_t> newVec(v.begin()+start, v.begin()+end);
  return newVec;
}

