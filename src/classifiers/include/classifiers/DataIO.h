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
// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/point_cloud_conversion.h>


class DataIO
{
public:
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

    if (!registerCallbacks(n))
    {
      ROS_ERROR("%s: failed to register callbacks", name.c_str());
      return false;
    }

    return true;
  }

  
  typedef struct pc_data
  {
    arma::vec3 position;
    unsigned int node_id;
    unsigned int node_label;
    arma::vec::fixed<10> features;
    void print() const
    {
       printf("(x,y,z) = (%3.3f, %3.3f, %3.3f),\tid = %u,\tlabel = %u\tfeatures = [%3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f]\n",
              position(0),position(1),position(2), node_id, node_label,
              features(0), features(1), features(2), features(3), features(4), features(5), features(6), features(7), features(8), features(9));
    }
  } pc_data_t;

  bool loadParameters(const ros::NodeHandle& n)
  {
    if (!parameter_utils::get("frame_id/fixed", fixed_frame_id)) return false;

//     if (!pu::get("files/data", data_file_name)) return false;
//     if (!pu::get("files/path", base_path)) return false;

//     loadData(base_path + "data/log/" + data_file_name);
//     std::cout << "Loaded " << stampedData.size() << " data entries" << std::endl;

    return true;
  }
  
  bool registerCallbacks(const ros::NodeHandle& n)
  {
    ros::NodeHandle nl(n, "");

    pointcloud_pub =
    nl.advertise<sensor_msgs::PointCloud>("pointcloud", 10, true);

    return true;
  }
  

  bool loadData(const std::string& data_path, std::vector<pc_data_t>& d)
  {
    std::string line;
    std::ifstream datafile (data_path.c_str());
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

        if (datafile_line_idx<20000)
        {
          datafile_line_idx++;
          continue;
        }
        
        if (datafile_line_idx>60000)
          break;

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
      std::cout << "Unable to open data file" << std::endl;
      return false;
    }

  }

  
  void visualize(std::vector<pc_data_t>& d)
  {
    sensor_msgs::PointCloud pcld;
    pcld.header.frame_id = fixed_frame_id;
    pcld.points.resize(d.size());
    pcld.channels.resize(d.size());
    std::cout << "size of d " << d.size() << std::endl;
    for (unsigned int i = 0; i < d.size(); i++)
    {
      pcld.points[i].x = d[i].position(0);
      pcld.points[i].y = d[i].position(1);
      pcld.points[i].z = d[i].position(2);
      pcld.channels[i].name = "rgb";
//       std::cout << "val[" << i << "] = " << d[i].node_label/2000.0 << std::endl;
      unsigned int color;
      switch(d[i].node_label)
      {
        case 1004:
          color = 0x00aa00;
          break;
        case 1400:
          color = 0x0000ff;
          break;
        case 1103:
          color = 0xff0000;
          break;
        case 1200:
          color = 0xff8800;
          break;
        case 1100:
          color = 0x000000;
          break;
      }
      pcld.channels[i].values.push_back(color);
    }

//     sensor_msgs::PointCloud2 pcld2;
//     sensor_msgs::convertPointCloudToPointCloud2(pcld,pcld2);
    pointcloud_pub.publish(pcld);
  }

private:

  // string parameters
  std::string name;
  std::string fixed_frame_id;
  std::string odom_frame_id;
  std::string laser_frame_id;
  std::string base_frame_id;

  std::string data_file_name;
  std::string base_path;

  ros::Publisher pointcloud_pub;

};

#endif
