/*
 * ircp_loop.cpp
 *
 *  Created on: Sep 4, 2014
 *      Author: baris
 */

#include <cluster_processing.hpp>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/openni_grabber.h>

#include <utils.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <feature_extraction/PcFeatures.h>
#include <feature_extraction/PcFeatureArray.h>
#include <segmentation/SegmentedClusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <utils_pcl_ros.hpp>

#include <k2g.h>

#include <signal.h>

pcl::PointCloud<PointT>::ConstPtr prev_cloud;
boost::mutex cloud_mutex;
boost::mutex imageName_mutex;
bool writePCD2File = false;
char imageName[100];
bool gotCluster = false;
bool gotNormal = false;
bool gotPlane = false;
bool interrupt = false;
pcl::PointCloud<PointT>::CloudVectorType clusters;
std::vector<pcl::PointCloud<pcl::Normal> > normals;
Eigen::Vector4f plane (0,0,0,0);
const char *clusterRostopic = "/beliefs/clusters";
const char *normalRostopic = "/beliefs/normals";
const char *planeRostopic = "/beliefs/plane";
const char *outRostopic = "/beliefs/features";
const char *transformRostopic = "/beliefs/tfs";

void interruptFn(int sig) {
  interrupt = true;
}

/*void
cluster_cb (const sensor_msgs::PointCloud2ConstPtr& cluster)
{
    pcl::PointCloud<PointT>::Ptr pclCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cluster, *pclCloud);

    clusters.clear();
    clusters.push_back(*pclCloud);
    gotCluster = true;
}*/

void
cluster_cb (const segmentation::SegmentedClusters& msg)
{
    clusters.clear();
    for(int i = 0; i < msg.clusters.size(); i++) {
        pcl::PointCloud<PointT>::Ptr pclCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(msg.clusters[i], *pclCloud);
        clusters.push_back(*pclCloud);
    }

    normals.clear();
    for(int i = 0; i < msg.normals.size(); i++) {
        pcl::PointCloud<pcl::Normal>::Ptr pclCloud (new pcl::PointCloud<pcl::Normal>);
        pcl::fromROSMsg(msg.normals[i], *pclCloud);
        normals.push_back(*pclCloud);
    }

    std::vector<float>::const_iterator it = msg.plane.data.begin();
    for(int i = 0; i < 4; i++) {
	plane[i] = *it;
	++it;
    }

    gotCluster = true;
}

void
normal_cb (const pcl::PointCloud<pcl::Normal>::ConstPtr& normal)
{
    normals.clear();
    normals.push_back(*normal);
    gotNormal = true;
}

void
plane_cb (const std_msgs::Float32MultiArray::ConstPtr& arr)
{
    std::vector<float>::const_iterator it = arr->data.begin();
    for(int i = 0; i < 4; i++) {
	plane[i] = *it;
	++it;
    }
    gotPlane = true;
}

int
main (int argc, char **argv)
{

  parsedArguments pA;
  if(parseArguments(argc, argv, pA) < 0)
    return 0;

  ridiculous_global_variables::ignore_low_sat       = pA.saturation_hack;
  ridiculous_global_variables::saturation_threshold = pA.saturation_hack_value;
  ridiculous_global_variables::saturation_mapped_value = pA.saturation_mapped_value;

  OpenNIOrganizedMultiPlaneSegmentation multi_plane_app;

  pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr ncloud_ptr (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Label>::Ptr label_ptr (new pcl::PointCloud<pcl::Label>);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  if(pA.viz) {
    viewer = cloudViewer(cloud_ptr);
    multi_plane_app.setViewer(viewer);
  }

  float workSpace[] = {-0.3,0.4,-0.25,0.35,0.3,2.0};//Simon on the other side:{-0.1,0.6,-0.4,0.15,0.7,1.1};//{-0.5,0.6,-0.4,0.4,0.4,1.1};
  multi_plane_app.setWorkingVolumeThresholds(workSpace);

  pcl::Grabber* interface;
  ros::NodeHandle *nh;
  ros::Subscriber normalSub, clusterSub, planeSub;
  ros::Publisher pub,transformPub;

  bool spawnObject = true;

  K2G *k2g;
  processor freenectprocessor = OPENGL;

  std::cout << "ros node initialized" << std::endl;
  ros::init(argc, argv, "feature_extraction",ros::init_options::NoSigintHandler);
  nh = new ros::NodeHandle();

  std::cout << "Publishing ros topic: " << outRostopic << std::endl;
  //pub = nh->advertise<pc_segmentation::PcFeatures>(outRostopic,5);
  pub = nh->advertise<feature_extraction::PcFeatureArray>(outRostopic,5);
  //transformPub = nh->advertise<geometry_msgs::Transform>(transformRostopic,5);


  std::cout << "Using ros topic as input" << std::endl;
  //clusterSub = nh->subscribe<pcl::PointCloud<pcl::PointXYZRGB>>(clusterRostopic, 1, cluster_cb);
  clusterSub = nh->subscribe(clusterRostopic, 1, cluster_cb);
 // normalSub = nh->subscribe<pcl::PointCloud<pcl::Normal>>(normalRostopic, 1, normal_cb);
 // planeSub = nh->subscribe(planeRostopic, 1, plane_cb);

  //if(!pA.viz)
   signal(SIGINT, interruptFn);
  while (!interrupt && (!pA.viz || !viewer->wasStopped ()))
  {
    //if(pA.ros_node)
    //{
      ros::spinOnce();
    //}
    if(pA.viz)
      viewer->spinOnce(20);
    if(!gotCluster)
      continue;
    std::vector<pc_cluster_features> feats;
    int selected_cluster_index = -1;
    if(cloud_mutex.try_lock ())
    {
      selected_cluster_index = multi_plane_app.processOnce(clusters,normals,feats,plane,pA.hue_val,pA.merge_clusters,pA.hue_thresh, pA.z_thresh, pA.euc_thresh);
     // pub.publish(feats);
      cloud_mutex.unlock();
    }

    if(selected_cluster_index < 0)
      continue;

    float angle = feats[selected_cluster_index].aligned_bounding_box.angle;
    std::cout << "Selected cluster angle (rad, deg): " << angle << " " << angle*180.0/3.14159 << std::endl;
    std::cout << "Selected cluster hue: " << feats[selected_cluster_index].hue << std::endl;

    feature_extraction::PcFeatureArray rosMsg;
    rosMsg.header.stamp = ros::Time::now();

    for(int i = 0; i < feats.size(); i++) {
      feature_extraction::PcFeatures ft;
      fillRosMessage(ft, feats[i]);
      rosMsg.objects.push_back(ft);
      rosMsg.transforms.push_back(ft.transform);
    }
    pub.publish(rosMsg);
    objectPoseTF(rosMsg.transforms[0]);
    /*if(spawnObject)
    {
      visualization_msgs::Marker marker;
      getObjectMarker(marker, rosMsg);
      objMarkerPub.publish(marker);
    }*/
  }
  if(pA.pc_source == 1)
  {
    interface->stop ();
    delete interface;
  }
    delete nh;

    ros::shutdown();
  return 0;
}
