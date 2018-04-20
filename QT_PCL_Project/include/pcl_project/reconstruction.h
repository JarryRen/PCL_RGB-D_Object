/**
 * 
 * 
 */
#pragma once
//pcl
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
//gp
#include "pcl_project/type.h"

namespace gp{ 
  class reconsruction{
  public:
    reconsruction();
    
    /** \brief 点云转换到模型 模板函数
     * \param[in] cloud 输入点云
     * \param[out] mesh 重建后的网格模型
     * \param[in] method 使用方法 \1 泊松  \2 三角化
     */ 
    void pcd_to_mesh(pcl::PointCloud< PointT >::Ptr cloud, pcl::PolygonMesh& mesh, int method);
    
  private:
    double mls_radius;
    //poisson param
    int depth;
    int solver_divide;
    int iso_divide;
    
    /** \brief 移动最小二乘 平滑处理
     * \param[in] input RGB点云
     * \param[out] output 平滑后RGB点云
     */ 
    void moving_least_sauares(pcl::PointCloud<PointT>::Ptr input,pcl::PointCloud<PointT>::Ptr output);
    
    /** \brief 法向量一致化
     * \param[in] input 
     * \param[out] output 
     */     
    void normlas();
    
    /** \brief 泊松重建
     * \param[in] cloud 输入点云
     * \param[out] mesh 重建后的网格模型
     */ 
    void poisson(pcl::PointCloud<PointT>::Ptr cloud,pcl::PolygonMesh &mesh);
    
    /** \brief 贪婪三角投影
     * 
     * 
     */ 
    void greedy_projection_triangulation(pcl::PointCloud<PointT>::Ptr cloud,pcl::PolygonMesh &mesh);
    
    /** \brief 模型染色
     * \param[in] mesh 重建后模型
     * \param[in] cloud 处理前点云数据，用于颜色提取
     * \out mesh 染色完成的模型
     */ 
    void mesh_coloring(pcl::PolygonMesh &mesh,pcl::PointCloud<PointT>::Ptr cloud);
    
    /** \brief 取得配置信息，文件为当前目录
     * \in 文件 project_config.json
     */ 
    void get_config();
  };
}