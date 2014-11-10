
#include <iostream>
#include <fstream>
#include <ctime>

#include <classifiers/DataIO.h>
#include <classifiers/Winnow.h>

using namespace std;
using namespace arma;

int main (int argc, char **argv)
{
  // Start up ros
  ros::init(argc, argv, "classifiers");
  ros::NodeHandle n("~");

  // Load data
  DataIO dataio;
  dataio.initialize(n);
  std::vector<DataIO::pc_data_t> d;
  dataio.loadData(d);

  // Example of how to load from predicted_datafilename (just call loadData with 'false')
  // std::vector<DataIO::pc_data_t> d_pred;
  // dataio.loadData(d_pred, false);
  
  // Run the classifier
  string algorithm;
  if (!parameter_utils::get("algorithm", algorithm)) return false;

  if (algorithm=="winnow")
  {
    cout << "Winnowinnowinnow" << endl;
    Winnow winnow;
    winnow.initialize(n);
    winnow.setData(d);
    winnow.train();
    winnow.test();
    std::vector<DataIO::pc_data_t> d_tested = winnow.getDataTested();
    dataio.visualize(d, d_tested);
  }
  else if (algorithm=="blr")
  {
    cout << "Add blr code here" << endl;

    // Visualization options
    // dataio.visualize(d); // Visualize truth data
    // dataio.visualize(d_pred, false); // Visualize prediction data
    // dataio.visualize(d, d_tested);  // Visualize truth and prediction data
  }
  else if (algorithm=="svm")
  {
    cout << "add svm code here" << endl;
  }
  else
    cout << "Invalid algorithm: " << algorithm << endl;

  ros::spin();

  return 0;
}
