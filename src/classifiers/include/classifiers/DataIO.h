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


class DataIO
{
public:

  typedef struct pc_data
  {
    arma::vec3 position;
    unsigned int node_id;
    unsigned int node_label;
    arma::vec::fixed<10> features;
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

    return true;
  }


  bool loadData(std::vector<pc_data_t>& d)
  {
    std::string line;
    std::ifstream datafile (data_file_name.c_str());
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
        datafile >> d_new.node_id >> d_new.node_label;
        for (unsigned int i = 0; i < 10; i++)
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


  void visualize(const std::vector<pc_data_t>& d, bool true_labels)
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
        case 1400:  // Facade
          p.r = 0;
          p.g = 0;
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
        case 1100:  // Wire
          p.r = 255;
          p.g = 255;
          p.b = 255;
          break;
        default:
          ROS_ERROR("%s: Invalid label %u",name.c_str(), label);
      }
      point_cloud.push_back(p);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>(point_cloud));
    pcl::visualization::CloudViewer viewer("CloudViewer");

    viewer.showCloud(point_cloud_ptr);

    // Block to allow interaction with viewer, exit if ros does
    while (!viewer.wasStopped () && ros::ok());

  }

private:
  
  // string parameters
  std::string name;
  std::string fixed_frame_id;

  std::string data_file_name;

  ros::Publisher pointcloud_pub;

};

#endif
