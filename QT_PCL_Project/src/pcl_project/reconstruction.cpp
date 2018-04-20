#include "pcl_project/reconstruction.h"

gp::reconsruction::reconsruction()
{
  get_config();
}

void gp::reconsruction::pcd_to_mesh(pcl::PointCloud< gp::PointT >::Ptr cloud, pcl::PolygonMesh& mesh, int method)
{
  pcl::PointCloud< gp::PointT >::Ptr cloud_mls(new pcl::PointCloud< gp::PointT >);
  moving_least_sauares(cloud,cloud_mls);
  if(method == 1){
    poisson(cloud_mls,mesh);
    mesh_coloring(mesh,cloud);
  }
  if(method == 2){

  }
}

void gp::reconsruction::moving_least_sauares(pcl::PointCloud< gp::PointT >::Ptr input, pcl::PointCloud< gp::PointT >::Ptr output)
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(input);
  
  pcl::MovingLeastSquares<pcl::PointXYZRGB,pcl::PointXYZRGB> mls;
  mls.setInputCloud(input);
  mls.setComputeNormals(true);
  mls.setPolynomialFit(true);//启用多项式你拟合
  mls.setPolynomialOrder(3);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(mls_radius);//值越大，平滑范围越大，点越多
  /*
  mls.setUpsamplingMethod(pcl::MovingLeastSquares< pcl::PointXYZRGB, pcl::PointXYZRGB >::VOXEL_GRID_DILATION);
  mls.setDilationVoxelSize(1);
  mls.setDilationIterations(2);
  */
  mls.process(*output);
}


void gp::reconsruction::poisson(pcl::PointCloud< gp::PointT >::Ptr cloud, pcl::PolygonMesh& mesh)
{
  
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud,centroid);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<PointT,pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setRadiusSearch(0.01);
  ne.setViewPoint(centroid[0],centroid[1],centroid[2]);
  ne.compute(*cloud_normals);
  
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields(*cloud,*cloud_normals,*cloud_with_normals);
  
  pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
  poisson.setInputCloud(cloud_with_normals);

  poisson.setDepth(depth);//
  poisson.setSolverDivide(solver_divide);
  poisson.setIsoDivide(iso_divide);
  
  poisson.setConfidence(false);
  poisson.setManifold(false);
  poisson.setOutputPolygons(false);
  
  poisson.reconstruct(mesh);
}

void gp::reconsruction::mesh_coloring(pcl::PolygonMesh& mesh, pcl::PointCloud< gp::PointT >::Ptr cloud)
{
  pcl::PointCloud<PointT> cloud_color_mesh; 
  pcl::fromPCLPointCloud2(mesh.cloud, cloud_color_mesh); 

  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud (cloud);
  // K 搜索的近邻点数
  int K = 5;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  for(int i=0;i<cloud_color_mesh.points.size();++i)
  {
      uint8_t r = 0;
      uint8_t g = 0;
      uint8_t b = 0;
      int red = 0;
      int green = 0;
      int blue = 0;
      
      if ( kdtree.nearestKSearch (cloud_color_mesh.points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
      {
	for (int j = 0; j < pointIdxNKNSearch.size (); ++j) 
	{ 
	    r = cloud->points[ pointIdxNKNSearch[j] ].r;
	    g = cloud->points[ pointIdxNKNSearch[j] ].g;
	    b = cloud->points[ pointIdxNKNSearch[j] ].b;

	    red += int(r);
	    green += int(g);
	    blue += int(b);
	}  
      }
      cloud_color_mesh.points[i].r = int(red/pointIdxNKNSearch.size ()+0.5); 
      cloud_color_mesh.points[i].g = int(green/pointIdxNKNSearch.size ()+0.5); 
      cloud_color_mesh.points[i].b = int(blue/pointIdxNKNSearch.size ()+0.5);
  }
      toPCLPointCloud2(cloud_color_mesh, mesh.cloud);
}

void gp::reconsruction::get_config()
{
    std::ifstream fin("project_config.json");
    rapidjson::IStreamWrapper isw(fin);
    
    rapidjson::Document document;
    document.ParseStream(isw);
    
    rapidjson::Value &rec = document["reconsruction"]; 
    mls_radius = rec["mls_radius"].GetFloat();
    
    depth= rec["depth"].GetInt();
    solver_divide = rec["solver_divide"].GetInt();
    iso_divide = rec["iso_divide"].GetInt();
}
