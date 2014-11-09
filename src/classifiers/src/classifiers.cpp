
#include <iostream>
#include <fstream>
#include <ctime>

#include <classifiers/DataIO.h>

using namespace std;
using namespace arma;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "classifiers");
  ros::NodeHandle n("~");

  DataIO dataio;
  dataio.initialize(n);

  std::vector<DataIO::pc_data_t> d;
  dataio.loadData(d);
  // TODO: call classifier on d
  dataio.visualize(d, true);  // Change to false to show predicted labels instead
    
  ros::spin();

  return 0;
}
