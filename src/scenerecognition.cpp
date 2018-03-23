#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "boost/bind.hpp"
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<iostream>

//#include "scene_recognition/config.h"
//struct LocalConfig :Config
//{
//  static int DistanceThreshold;
//  static float p_inliers;
//  LocalConfig() :
//  Config(){
//    params.push_back(new Parameter<int> ("DistanceThreshold", &DistanceThreshold, ""));
//    params.push_back(new Parameter<float> ("p_inliers", &p_inliers, ""));
//  }
//};
//int LocalConfig::DistanceThreshold = 1;
//float LocalConfig::p_inliers = 0.3;

float DistanceThreshold = 0.2;
float p_inliers = 0.1;

class Segmentation
{
public:
  ros::Publisher seg_pub;
  ros::Subscriber cloud_sub;

  Segmentation(ros::NodeHandle& nh):
    seg_pub(nh.advertise<sensor_msgs::PointCloud2> ("seg_output", 1))
    { }
  void init(ros::NodeHandle& nh)
  {
    cloud_sub=nh.subscribe("ring_buffer/cloud2", 1, &Segmentation::cloudCallback, this);
  }
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    sensor_msgs::PointCloud2 output;
    //pcl::visualization::CloudViewer viewer("pcd viewer");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input, *cloud);
    //pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    //compute normal
    //cloud_with_normals=Segmentation::NormalCalculation(cloud);
    //std::cerr << "normals:"<<cloud_with_normals->points[1].normal[0]<<std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.1f, 0.1f, 0.1f);
    vg.filter (*cloud_filtered);
    //normal computation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    //建立kdtree来进行近邻点集搜索
    //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    //为kdtree添加点云数据
    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
    //tree->setInputCloud (cloud_filtered);
    ne.setInputCloud (cloud_filtered);
    ne.setSearchMethod (tree);
    //点云法向计算时，需要搜索的近邻点大小
    ne.setRadiusSearch (0.02) ;
    //开始进行法向计算
    ne.compute (*normals);
    //* normals should not contain the point normals + surface curvatures
    // Concatenate the XYZ and normal fields*
    //将点云数据与法向信息拼接
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //一个点云团队列，用于存放聚类结果
    std::vector <pcl::PointIndices> clusters;
    //区域生长分割器
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    //输入分割目标
    reg.setSearchMethod (tree);
    //reg.setNumberOfNeighbours (100);
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(100000);
    reg.setNumberOfNeighbours (8);
    reg.setInputCloud (cloud_filtered);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    //设置限制条件及先验知识
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);
    reg.extract (clusters);
    std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
    for(int i=0;i<clusters.size();i++)
    {
      std::cout << "The"<<i<<" cluster has " << clusters[i].indices.size() << " points." << endl;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
    std::vector<unsigned int> colors(3,1);
    colored_cloud->is_dense = cloud_filtered->is_dense;
    for(size_t i_cluster = 0; i_cluster < clusters.size(); i_cluster++)
    {
      colors[0] = rand() % 256;
      colors[1] = rand() % 256;
      colors[2] = rand() % 256;
      for(size_t i_point = 0; i_point <clusters[i_cluster].indices.size();i_point++)
      {

        //colors.push_back(static_cast<unsigned int> (rand() % 256));
        //colors.push_back(static_cast<unsigned int> (rand() % 256));
        pcl::PointXYZRGB point;
        point.x = (cloud_filtered->points[clusters[i_cluster].indices[i_point]].x);
        point.y = (cloud_filtered->points[clusters[i_cluster].indices[i_point]].y);
        point.z = (cloud_filtered->points[clusters[i_cluster].indices[i_point]].z);
        point.r = colors[0];
        point.g = colors[1];
        point.b = colors[2];
        colored_cloud->points.push_back(point);
      }
    }
    pcl::toROSMsg(*colored_cloud, output);
    output.header.frame_id = "world";
    seg_pub.publish(output);
    std::cout << "Finished" <<std::endl;
  }
};
    //if(cloud_plane->points.size () < p_inliers * nr_points)
    //{
    //  break;
    //}


    //for (size_t i_point = 0; i_point < cloud_plane->points.size(); i_point++)
        //  {
          //    pcl::PointXYZRGB point;
          //    point.x = *(cloud_plane->points[i_point].data);
        //      point.y = *(cloud_plane->points[i_point].data + 1);
        //      point.z = *(cloud_plane->points[i_point].data + 2);
        //      point.r = colors[0];
        //      point.g = colors[1];
        //      point.b = colors[2];
        //      colored_cloud->points.push_back(point);
      //    }

    //color cloud
    //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colored_normal_cloud;
    //colored_normal_cloud = (new pcl::PointCloud<pcl::PointXYZRGBNormal>)->makeShared();
    //std::vector<unsigned int> colors;
    //colors.push_back(static_cast<unsigned int> (rand() % 256));
    //colors.push_back(static_cast<unsigned int> (rand() % 256));
    //colors.push_back(static_cast<unsigned int> (rand() % 256));

    //kd-tree search
    //pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    //kdtree.setInputCloud (cloud_filtered);
    //pcl::PointXYZ searchPoint;
    //std::vector<int> pointIdxRadiusSearch;
    //std::vector<float> pointRadiusSquaredDistance;
    //float radius = 0.4;
    //for (size_t i_point = 0; i_point < cloud_filtered->points.size(); i_point++)
    //{
      //searchPoint.x = cloud_filtered->points[i_point].x;
      //searchPoint.y = cloud_filtered->points[i_point].y;
      //searchPoint.z = cloud_filtered->points[i_point].z;
      //if(kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )


//    }






    // Create the segmentation object for the planar model and set all the parameters
    //pcl::SACSegmentation<pcl::PointXYZ> seg;
    //pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    //pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    //seg.setOptimizeCoefficients (true);
    //seg.setModelType (pcl::SACMODEL_PLANE);
    //seg.setMethodType (pcl::SAC_RANSAC);
    //seg.setMaxIterations (100);
    //seg.setDistanceThreshold (LocalConfig::DistanceThreshold);
    //seg.setDistanceThreshold (DistanceThreshold);
    //std::cerr << "LocalConfig::DistanceThreshold:"<<LocalConfig::DistanceThreshold<<std::endl;
    //segmentation iterativly
    //int i=0, nr_points = (int) cloud_filtered->points.size ();
    //while (ros::ok())
    //{
      // Segment the largest planar component from the remaining cloud
      //seg.setInputCloud (cloud_filtered);
      //seg.segment (*inliers, *coefficients);
      //std::cerr << "inliers:"<<inliers->indices.size()<<std::endl;
      //std::cerr << "totalpoints"<<nr_points<<std::endl;
      //std::cerr << "Model coefficients: " <<i<<" "<< coefficients->values[0] << " "
        //                                  << coefficients->values[1] << " "
          //                                << coefficients->values[2] << " "
            //                              << coefficients->values[3] << std::endl;
      //if (inliers->indices.size () == 0)
      //{
        //std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        //break;
      //}

      // Extract the planar inliers from the input cloud
      //pcl::ExtractIndices<pcl::PointXYZ> extract;
      //extract.setInputCloud (cloud_filtered);
      //extract.setIndices (inliers);
      //extract.setNegative (false);

      // Get the points associated with the planar surface
      //extract.filter (*cloud_plane);
      //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
      // Remove the planar inliers, extract the rest
      //extract.setNegative (true);
      //extract.filter (*cloud_f);
      //*cloud_filtered = *cloud_f;
      //i++;

      //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colored_normal_cloud;
      //colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
      //std::vector<unsigned int> colors;
      //colors.push_back(static_cast<unsigned int> (rand() % 256));
      //colors.push_back(static_cast<unsigned int> (rand() % 256));
      //colors.push_back(static_cast<unsigned int> (rand() % 256));
      //colored_cloud->width = cloud_plane->width;
      //colored_cloud->height = cloud_plane->height;


//pcl::PointCloud<pcl::PointNormal>::Ptr NormalCalculation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//{
  //pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  //pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  //建立kdtree来进行近邻点集搜索
  //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  //为kdtree添加点云数据
  //tree->setInputCloud (cloud);
  //ne.setInputCloud (cloud);
  //ne.setSearchMethod (tree);
  //点云法向计算时，需要搜索的近邻点大小
  //ne.setKSearch (0.5);
  //开始进行法向计算
  //ne.compute (*normals);
  //* normals should not contain the point normals + surface curvatures
  // Concatenate the XYZ and normal fields*
  //将点云数据与法向信息拼接
  //pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  //pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //return cloud_with_normals;
//}

int main(int argc, char **argv)
{
//  Parser parser;
//  parser.addGroup(LocalConfig());
//  parser.addGroup(GeneralConfig());
//  parser.read(argc, argv);
  ros::init(argc, argv, "recognition");
  ros::NodeHandle nh;
  Segmentation segment(nh);
  segment.init(nh);
  ros::spin();
  return 0;
}
