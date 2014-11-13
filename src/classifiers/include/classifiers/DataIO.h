/*!
 * \file DataIO.h
 */

#ifndef _DATA_IO_H_
#define _DATA_IO_H_

#include <list>
#include <vector>
#include <armadillo>
#include <fstream>
#include <string>

#include <ros/ros.h>

#include <classifiers/ParameterUtils.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#define NUM_FEATURES 20

class DataIO
{
public:

  typedef struct pc_data
  {
    arma::vec3 position;
    unsigned int node_id;
    unsigned int node_label;
    arma::vec::fixed<NUM_FEATURES> features;
    unsigned int predicted_label;
    void print() const
    {
       printf("(x,y,z) = (%3.3f, %3.3f, %3.3f),\tid = %u,\tlabel = %u\tprediction =%u\tfeatures = [%3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f]\n",
              position(0),position(1),position(2), node_id, node_label, predicted_label,
              features(0), features(1), features(2), features(3), features(4), features(5), features(6), features(7), features(8), features(9));
    }
  } pc_data_t;


  
  DataIO(){};
  ~DataIO(){};

  bool initialize(const ros::NodeHandle& n)
  {
    std::string name = ros::names::append(n.getNamespace(), "DataIO");

    if (!loadParameters(n))
    {
      ROS_ERROR("%s: failed to load parameters", name.c_str());
      return false;
    }

    return true;
  }


  bool loadParameters(const ros::NodeHandle& n)
  {
    if (!parameter_utils::get("frame_id/fixed", fixed_frame_id)) return false;

    if (!parameter_utils::get("datafilename", data_file_name)) return false;
    if (!parameter_utils::get("predicted_datafilename", pred_data_file_name)) return false;

    return true;
  }


  bool loadData(std::vector<pc_data_t>& d, bool true_labels=true)
  {
    std::string line;
    std::string file_name;
    if (true_labels)
      file_name = data_file_name;
    else
      file_name = pred_data_file_name;
    
    std::ifstream datafile (file_name.c_str());
    if (datafile.is_open())
    {
      unsigned int datafile_line_idx = 0;
      while ( getline (datafile,line))
      {
        if (datafile_line_idx<2)
        {
          datafile_line_idx++;
          continue;
        }

        pc_data_t d_new;
        datafile >> d_new.position(0) >> d_new.position(1) >> d_new.position(2);
        datafile >> d_new.node_id;
        if (true_labels)
          datafile >> d_new.node_label;
        else
          datafile >> d_new.predicted_label;
        
        for (unsigned int i = 0; i < NUM_FEATURES; i++)
          datafile >> d_new.features(i);

        d.push_back(d_new);

        datafile_line_idx++;
      }
      return true;
    }
    else
    {
      ROS_ERROR("%s: Unable to open data file",name.c_str());
      return false;
    }
    ROS_INFO("%s: Loaded %s",name.c_str(), data_file_name.c_str());
  }


  void visualizationManager()
  {
    viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();

    std::size_t found = data_file_name.find("_am_");
    if (found!=std::string::npos)
    {
      // Camera view for _am_ dataset
      viewer->setCameraPosition(43.0757,255.961,25.1505,
                                179.566,219.226,14.8565,
                                0.0494566,-0.0948605,0.994261);
    }

    found = data_file_name.find("_an_");
    if (found!=std::string::npos)
    {
      // Camera view for _an_ dataset
      viewer->setCameraPosition(53.6417,172.444,22.2762,
                                148.357,108.08,-2.31281,
                                0.166275,-0.128802,0.977631);
    }
    viewer->setBackgroundColor (0, 0, 0);
  }

  
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr
  getCloudVis(const std::vector<pc_data_t>& d, bool true_labels)
  {
    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;

    for (unsigned int i = 0; i < d.size(); i++)
    {
      pcl::PointXYZRGB p;
      p.x = d[i].position(0);
      p.y = d[i].position(1);
      p.z = d[i].position(2);

      unsigned int label;
      if (true_labels)
        label = d[i].node_label;
      else
        label = d[i].predicted_label;

      switch(label)
      {
        case 1004:  // Veg
          p.r = 0;
          p.g = 255;
          p.b = 0;
          break;
        case 1100:  // Wire
          p.r = 255;
          p.g = 255;
          p.b = 255;
          break;
        case 1103:  // Pole
          p.r = 255;
          p.g = 0;
          p.b = 0;
          break;
        case 1200:  // Ground
          p.r = 255;
          p.g = 128;
          p.b = 0;
          break;
        case 1400:  // Facade
          p.r = 0;
          p.g = 0;
          p.b = 255;
          break;
        default:
          ROS_ERROR("%s: Invalid label %u",name.c_str(), label);
          exit(-1);
      }
      point_cloud.push_back(p);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>(point_cloud));
    return point_cloud_ptr;
  }

  // Call with one argument to visualize the data passed in
  void visualize(const std::vector<pc_data_t>& d, bool true_labels=true)
  {
    visualizationManager();

    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_ptr = getCloudVis(d, true_labels);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_ptr);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_ptr, rgb, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    viewer->addCoordinateSystem (1.0);

    while (ros::ok())
      viewer->spinOnce(100);
  }

  // Call with two arguments to visualize the data passed as truth vs predicted
  void visualize(const std::vector<pc_data_t>& d_truth,
                 const std::vector<pc_data_t>& d_pred)
  {
    visualizationManager();
    
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr truth_cloud_ptr = getCloudVis(d_truth, true);
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pred_cloud_ptr = getCloudVis(d_pred, false);
    
    int v1(0);
    viewer->createViewPort(0.0,0.0,0.5,1.0,v1);
    viewer->addText("Truth",10,10,"v1 text",v1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_truth(truth_cloud_ptr);
    viewer->addPointCloud<pcl::PointXYZRGB> (truth_cloud_ptr, rgb_truth, "true cloud", v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "true cloud");
    viewer->addCoordinateSystem (1.0);

    int v2(0);
    viewer->createViewPort(0.5,0.0,1.0,1.0,v2);
    viewer->addText("Prediction",10,10,"v2 text",v2);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_pred(pred_cloud_ptr);
    viewer->addPointCloud<pcl::PointXYZRGB> (pred_cloud_ptr, rgb_pred, "predicted cloud", v2);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "predicted cloud");
    viewer->addCoordinateSystem (1.0);

    while (ros::ok())
      viewer->spinOnce(100);
  }
  
private:

  // string parameters
  std::string name;
  std::string fixed_frame_id;

  std::string data_file_name, pred_data_file_name;

  ros::Publisher pointcloud_pub;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

};

#endif
