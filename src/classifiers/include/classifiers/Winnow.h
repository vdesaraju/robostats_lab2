/*!
 * \file Winnow.h
 */

#ifndef _WINNOW_H_
#define _WINNOW_H_

#include <list>
#include <vector>
#include <armadillo>
#include <fstream>
#include <string>

#include <classifiers/DataIO.h>

class Winnow
{
public:

  Winnow();
  ~Winnow();
  
  bool initialize(const ros::NodeHandle& n);

  void setData(std::vector<DataIO::pc_data_t>& d);
  void train();
  void test();
  std::vector<DataIO::pc_data_t>& getDataTested();

private:

  double train_ratio;
  unsigned int train_end_idx;
  double feature_thresh;
  double pred_thresh;
  double alpha;
  
  std::vector<DataIO::pc_data_t> data;
  std::vector<DataIO::pc_data_t> data_tested;
  arma::mat::fixed<5,NUM_FEATURES> weights;
  arma::vec::fixed<NUM_FEATURES> maxf;
  arma::vec::fixed<NUM_FEATURES> minf;

  arma::vec::fixed<5> maxwtf;
  arma::vec::fixed<5> minwtf;
  
  bool loadParameters(const ros::NodeHandle& n);

  void computeMaxf(std::vector<DataIO::pc_data_t>& d);

  std::vector<DataIO::pc_data_t> getSubvec(const std::vector<DataIO::pc_data_t>& v,
                                           unsigned int start, unsigned int end);
};

#endif