
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
  dataio.loadData("/home/vishnu/Documents/RoboStats/robostats_lab2/data/oakland_part3_am_rf.node_features",d);

//   for (unsigned int i = 0; i<d.size(); i++)
//     d[i].print();

  dataio.visualize(d);
  
  ros::spin();

  return 0;
}
