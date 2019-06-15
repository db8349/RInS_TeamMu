#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/point_cloud.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PointStamped.h"

#include "robot/PCLCylinder.h"
#include <vector>

ros::Publisher pubx;
ros::Publisher puby;
ros::Publisher pubm;
ros::Publisher cylinder_pub;

tf2_ros::Buffer tf2_buffer;

typedef pcl::PointXYZRGB PointT;


void 
cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_blob)
{
  // All the objects needed

  ros::Time time_rec, time_test;
  time_rec = ros::Time::now();
  
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  Eigen::Vector4f centroid;

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
  
  // Read in the cloud data
  pcl::fromPCLPointCloud2 (*cloud_blob, *cloud);
  //std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.5);
  pass.filter (*cloud_filtered);

  //std::cerr << "PointCloud after z filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-5, 0.3);
  pass.filter (*cloud_filtered);

  if (cloud_filtered->points.size () == 0) {
    return;
  }

  //std::cerr << "PointCloud after y filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  //pcl::PCLPointCloud2 outcloud_cylinder;
  //pcl::toPCLPointCloud2 (*cloud_filtered, outcloud_cylinder);
  //puby.publish (outcloud_cylinder);

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  //std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_plane);
  //std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
  
  /*pcl::PCLPointCloud2 outcloud_plane;
  pcl::toPCLPointCloud2 (*cloud_plane, outcloud_plane);
  pubx.publish (outcloud_plane);*/

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  if (cloud_filtered2->points.size () == 0) {
    return;
  }

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0.06, 0.2);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  //std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  int temp_lol = 0;
  if (cloud_cylinder->points.empty ()) 
    //std::cerr << "Can't find the cylindrical component." << std::endl;
    temp_lol = 1;
  else
  {
    if (cloud_cylinder->points.size () < 23000) {
      return;
    }
	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;

    std::vector<int> rV;
    std::vector<int> gV;
    std::vector<int> bV;
    long total = 0;
    for(pcl::PointCloud<PointT>::iterator it = cloud_cylinder->begin(); it != cloud_cylinder->end(); it++){
        uint32_t rgb = *reinterpret_cast<int*>(&it->rgb);
        //std::cerr << rgb << std::endl;
        uint8_t r = (rgb >> 16) & 0x0000ff;
        uint8_t g = (rgb >> 8)  & 0x0000ff;
        uint8_t b = (rgb)     & 0x0000ff;
        if ((int)r > 0 && (int)g > 0 && (int)b > 0) {
          rV.push_back((int)r);
          gV.push_back((int)g);
          bV.push_back((int)b);
          total++;
        }
    }
  
          pcl::compute3DCentroid (*cloud_cylinder, centroid);
          //std::cerr << "centroid of the cylindrical component: " << centroid[0] << " " <<  centroid[1] << " " <<   centroid[2] << " " <<   centroid[3] << std::endl;

	  //Create a point in the "camera_rgb_optical_frame"
          geometry_msgs::PointStamped point_camera;
          geometry_msgs::PointStamped point_map;
	      visualization_msgs::Marker marker;
          geometry_msgs::TransformStamped tss;
          
          point_camera.header.frame_id = "camera_rgb_optical_frame";
          point_camera.header.stamp = ros::Time::now();

	  	  point_map.header.frame_id = "map";
          point_map.header.stamp = ros::Time::now();

		  point_camera.point.x = centroid[0];
		  point_camera.point.y = centroid[1];
		  point_camera.point.z = centroid[2];

	  try{
		  time_test = ros::Time::now();

		  //std::cerr << time_rec << std::endl;
		  //std::cerr << time_test << std::endl;
  	      tss = tf2_buffer.lookupTransform("map","camera_rgb_optical_frame", time_rec);
          //tf2_buffer.transform(point_camera, point_map, "map", ros::Duration(2));
	  }
          catch (tf2::TransformException &ex)
	  {
	       ROS_WARN("Transform warning: %s\n", ex.what());
	  }

          //std::cerr << tss ;

          tf2::doTransform(point_camera, point_map, tss);

	      //std::cerr << "point_camera: " << point_camera.point.x << " " <<  point_camera.point.y << " " <<  point_camera.point.z << std::endl;

	      //std::cerr << "point_map: " << point_map.point.x << " " <<  point_map.point.y << " " <<  point_map.point.z << std::endl;
        robot::PCLCylinder cylinder;
        cylinder.point = point_map;
        cylinder.r = rV;
        cylinder.g = gV;
        cylinder.b = bV;
        cylinder_pub.publish(cylinder);

	  	  marker.header.frame_id = "map";
          marker.header.stamp = ros::Time::now();

          marker.ns = "cylinder";
          marker.id = 0;

          marker.type = visualization_msgs::Marker::CYLINDER;
          marker.action = visualization_msgs::Marker::ADD;

          marker.pose.position.x = point_map.point.x;
          marker.pose.position.y = point_map.point.y;
          marker.pose.position.z = point_map.point.z;
          marker.pose.orientation.x = 0.0;
	      marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;

          marker.scale.x = 0.1;
	      marker.scale.y = 0.1;
	      marker.scale.z = 0.1;

          marker.color.r=0.0f;
          marker.color.g=1.0f;
          marker.color.b=0.0f;
          marker.color.a=1.0f;

	      marker.lifetime = ros::Duration();

	      pubm.publish (marker);

	      /*pcl::PCLPointCloud2 outcloud_cylinder;
          pcl::toPCLPointCloud2 (*cloud_cylinder, outcloud_cylinder);
          puby.publish (outcloud_cylinder);*/
          std::cerr << "Publishing cylinder pose: " << point_map.point.x << " " <<  point_map.point.y << " " <<  point_map.point.z << std::endl;

  }
  
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cylinder_segment");
  ros::NodeHandle nh;

  // For transforming between coordinate frames
  tf2_ros::TransformListener tf2_listener(tf2_buffer);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pubx = nh.advertise<pcl::PCLPointCloud2> ("planes", 1);
  puby = nh.advertise<pcl::PCLPointCloud2> ("cylinder", 1);

  pubm = nh.advertise<visualization_msgs::Marker>("detected_cylinder",1);
  cylinder_pub = nh.advertise<robot::PCLCylinder>("cylinder_detect/cylinder", 1);

  // Spin
  ros::spin ();
}
