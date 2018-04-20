/**
 * 
 * 
 */
#pragma once

//pcl
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
//gp
#include "pcl_project/type.h"

namespace gp{
  
  class registration{
  public:
    registration();
    
    /** \brief 全局配准
     * \param[in] cloud_data 点云对象指针容器
     * \param[out] cloud 配准完成的点云对象指针
     */ 
    void global_registration(std::vector<pcl::PointCloud<PointT>::Ptr > &cloud_data,pcl::PointCloud<PointT>::Ptr global_alian_cloud);
  
  private:
    float leaf_size;
    double ne_radius;//法线计算半径
    double fpfh_radius;//fpfh特征计算半径
    
    double max_distance;//MaxCorrespondenceDistance
    int max_iterations;//MaximumIterations
    double trans_epsilon;//TransformationEpsilon
    double ef_epsilon;//EuclideanFitnessEpsilon
    
    /** \brief sac_预处理，过滤，提取特征
     * \param[in] input 待处理的源点云
     * \param[out] output 处理后点云
     * \param[out] fpfh 快速特征点直方图描述
     */ 
    void pre_process(pcl::PointCloud<PointT>::Ptr input,pcl::PointCloud<PointT>::Ptr output,
		     pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs);
    
    /** \brief sac_ia 粗匹配
     * \param[in] source 待转换的源点云
     * \param[in] target 目标点云
     * \param[out] source_out 处理后的源点云
     * \param[out] sac_trans 粗配转换矩阵
     */ 
    void sac_ia(pcl::PointCloud<PointT>::Ptr source,pcl::PointCloud<PointT>::Ptr target, pcl::PointCloud< PointT >::Ptr source_out,
		Eigen::Matrix4f &sac_trans);
    
    /** \brief 迭代最近点
     * \param[in] source 待转换的源点云
     * \param[in] target 目标点云
     * \param[in] sac_trans 粗配转换矩阵
     * \param[out] icp_trans 转换矩阵
     */ 
    void iterative_closest_point(pcl::PointCloud<PointT>::Ptr source, pcl::PointCloud<PointT>::Ptr target, 
				 Eigen::Matrix4f &sac_trans,Eigen::Matrix4f &icp_trans);
    
    /** \brief 取得配置信息，文件为当前目录
     * \in 文件 project_config.json
     */ 
    void get_config();
  };
  
}
