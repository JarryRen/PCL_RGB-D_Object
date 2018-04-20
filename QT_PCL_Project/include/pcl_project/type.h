/**
 * 
 * 
 */
#pragma once
//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//rapidjson
#include "3rdParty/rapidjson/document.h"
#include "3rdParty/rapidjson/istreamwrapper.h"
#include <fstream>

//Graduation Project
namespace gp{
  typedef pcl::PointXYZRGB PointT;
  
  //KinectV1相机内参
  const double camera_factor = 1000;
  const double camera_cx = 325.5;
  const double camera_cy = 253.5;
  const double camera_fx = 518.0;
  const double camera_fy = 519.0;
}