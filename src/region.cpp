#include <string>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <math.h>
#include <list>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

#include <ltu_actor_route_obstacle/Region.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include <ltu_actor_route_obstacle/RegionConfig.h>

using actor_cloud_t = pcl::PointCloud<pcl::PointXYZ>;
using actor_cloud_ptr_t = actor_cloud_t::Ptr;

#define REGION_ALPHA 0.4f
#define DEFAULT_CLOSEST 100.0

struct region{
    float originX;
    float originY;
    float width;
    float length;
    float z_min;
    float z_max;
};

class obstacle_loc{

    private:
        void TransformToBase(actor_cloud_ptr_t& out, actor_cloud_ptr_t& in);
        bool WithinRegion(const float& x, const float& y, const float& z, region reg);
        void configCallback(ltu_actor_route_obstacle::RegionConfig &config, uint32_t level);

        // Callback
        void CloudCallback(const sensor_msgs::PointCloud2ConstPtr&);

        //Region Publishing
        void PublishPoints(ros::Publisher pub, int points);
        void PublishClosest(ros::Publisher pub, double dist);
        void PublishRegions();
        visualization_msgs::Marker RegiontoMarker(region region, float red, float green, float blue, float alpha, int id);

        ros::NodeHandle nh_;
        ros::Publisher  cloud_pub;
        ros::Publisher vis_pub;
        ros::Subscriber lidar_sub;

        ros::Publisher pub_front_far;
        ros::Publisher pub_front_close;
        ros::Publisher pub_front_closest;
        ros::Publisher pub_left_far;
        ros::Publisher pub_left_close;
        ros::Publisher pub_left_closest;
        ros::Publisher pub_right_far;
        ros::Publisher pub_right_close;
        ros::Publisher pub_right_closest;

        region front_close;
        region front_far;
        region left_close;
        region left_far; 
        region right_close;
        region right_far; 

        double front_dist;
        double left_dist;
        double right_dist;

        std::string sub_topic;

        dynamic_reconfigure::Server<ltu_actor_route_obstacle::RegionConfig> server_;
        ltu_actor_route_obstacle::RegionConfig config_;

        bool enabled;
    public:
        obstacle_loc();

        bool isEnabled() {return enabled;}
        void shutdown() {
            lidar_sub = ros::Subscriber();
            enabled =  false;
        }
        void startup(){
            lidar_sub = nh_.subscribe<sensor_msgs::PointCloud2>(sub_topic, 100, &obstacle_loc::CloudCallback, this);
            enabled = true;
        }
        bool hasSub(){
            return (pub_front_far.getNumSubscribers() || pub_front_close.getNumSubscribers() || pub_front_closest.getNumSubscribers() ||
                    pub_left_far.getNumSubscribers() || pub_left_close.getNumSubscribers() || pub_left_closest.getNumSubscribers() ||
                    pub_right_far.getNumSubscribers() || pub_right_close.getNumSubscribers() || pub_right_closest.getNumSubscribers() || vis_pub.getNumSubscribers()) > 0;
        }
};

// constructor
// set up publisher and subscriber
obstacle_loc::obstacle_loc()
{
    enabled = false;

    nh_ = ros::NodeHandle("~");

    if (!nh_.getParam("sub_topic", sub_topic))
    {
        ROS_ERROR_STREAM("No lidar topic passed to " + sub_topic);
        throw std::invalid_argument("Bad lidar topic.");
    }

    vis_pub = nh_.advertise<visualization_msgs::MarkerArray>( "/regions_visualization", 10);

    pub_front_far = nh_.advertise<std_msgs::Int32>("front_far", 10);
    pub_front_close = nh_.advertise<std_msgs::Int32>("front_close", 10);
    pub_front_closest = nh_.advertise<std_msgs::Float64>("front_closest", 10);
    pub_left_far = nh_.advertise<std_msgs::Int32>("left_far", 10);
    pub_left_close = nh_.advertise<std_msgs::Int32>("left_close", 10);
    pub_left_closest = nh_.advertise<std_msgs::Float64>("left_closest", 10);
    pub_right_far = nh_.advertise<std_msgs::Int32>("right_far", 10);
    pub_right_close = nh_.advertise<std_msgs::Int32>("right_close", 10);
    pub_right_closest = nh_.advertise<std_msgs::Float64>("right_closest", 10);

    // Dynamic Reconfigure
    dynamic_reconfigure::Server<ltu_actor_route_obstacle::RegionConfig>::CallbackType cb;
    cb = boost::bind(&obstacle_loc::configCallback, this, _1, _2);
    server_.setCallback(cb);
    server_.getConfigDefault(config_);
}


void obstacle_loc::configCallback(ltu_actor_route_obstacle::RegionConfig &config, uint32_t level)
{
    config_ = config;

    //Front Close Config Settings
    front_close.originX = config_.front_close_originX;
    front_close.originY = config_.front_close_originY;
    front_close.width = config_.front_close_width;
    front_close.length = config_.front_close_length;
    front_close.z_min = config_.front_close_z_min;
    front_close.z_max = config_.front_close_z_max;

    //Front Far Config Settings
    front_far.originX = config_.front_far_originX;
    front_far.originY = config_.front_far_originY;
    front_far.width = config_.front_far_width;
    front_far.length = config_.front_far_length;
    front_far.z_min = config_.front_far_z_min;
    front_far.z_max = config_.front_far_z_max;

    //Left Close Config Settings
    left_close.originX = config_.left_close_originX;
    left_close.originY = config_.left_close_originY;
    left_close.width = config_.left_close_width;
    left_close.length = config_.left_close_length;
    left_close.z_min = config_.left_close_z_min;
    left_close.z_max = config_.left_close_z_max;

    //Left Far Config Settings
    left_far.originX = config_.left_far_originX;
    left_far.originY = config_.left_far_originY;
    left_far.width = config_.left_far_width;
    left_far.length = config_.left_far_length;
    left_far.z_min = config_.left_far_z_min;
    left_far.z_max = config_.left_far_z_max;

    //Right Close Config Settings
    right_close.originX = config_.right_close_originX;
    right_close.originY = config_.right_close_originY;
    right_close.width = config_.right_close_width;
    right_close.length = config_.right_close_length;
    right_close.z_min = config_.right_close_z_min;
    right_close.z_max = config_.right_close_z_max;

    //Right Far Config Settings
    right_far.originX = config_.right_far_originX;
    right_far.originY = config_.right_far_originY;
    right_far.width = config_.right_far_width;
    right_far.length = config_.right_far_length;
    right_far.z_min = config_.right_far_z_min;
    right_far.z_max = config_.right_far_z_max;
}


bool obstacle_loc::WithinRegion(const float& x, const float& y, const float& z, region reg){

    if (z >= reg.z_min && z <= reg.z_max){
        if (x >= reg.originX && x <= reg.originX + reg.length){
            if (y >= reg.originY && y <= reg.originY + reg.width){
                return true;
            }
        }
    }

    return false;
}

void obstacle_loc::CloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    int front_far_points = 0;
    int front_close_points = 0;
    int left_far_points = 0;
    int left_close_points = 0;
    int right_far_points = 0;
    int right_close_points = 0;

    front_dist = DEFAULT_CLOSEST;
    left_dist = DEFAULT_CLOSEST;
    right_dist = DEFAULT_CLOSEST;


    pcl::PCLPointCloud2 pcl_pc2;
    //convert from msg to pcl
    pcl_conversions::toPCL(*msg,pcl_pc2);

    //create input cloud
    actor_cloud_ptr_t cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //convert from pcl to ros
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

    //create cloud for transforming
    actor_cloud_ptr_t centeredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    //The ground plane and rotation may not be needed.
    //It seems like the puck is able to do it for us.
    TransformToBase(centeredCloud,cloud);

    //cloud_pub.publish(*centeredCloud);

    //Set Regions
    for (auto p : *centeredCloud){
        if (WithinRegion(p.x, p.y, p.z, front_close)){
            front_close_points++;
            if(front_dist > p.x){ front_dist = p.x; }
        }
        else if (WithinRegion(p.x, p.y, p.z, front_far)){
            front_far_points++;
        }
        else if (WithinRegion(p.x, p.y, p.z, left_close)){
            left_close_points++;
            if(left_dist > p.x){ left_dist = p.x; }
        }
        else if (WithinRegion(p.x, p.y, p.z, left_far)){
            left_far_points++;
        }
        else if (WithinRegion(p.x, p.y, p.z, right_close)){
            right_close_points++;
            if(right_dist > p.x){ right_dist = p.x; }
        }
        else if (WithinRegion(p.x, p.y, p.z, right_far)){
            right_far_points++;
        }
    }
    
    //Publish number of points in each region
    PublishPoints(pub_front_far, front_far_points);
    PublishPoints(pub_front_close, front_close_points);
    PublishPoints(pub_left_far, left_far_points);
    PublishPoints(pub_left_close, left_close_points);
    PublishPoints(pub_right_far, right_far_points);
    PublishPoints(pub_right_close, right_close_points);

    //Publish closest point in front regions 
    PublishClosest(pub_front_closest, front_dist);
    PublishClosest(pub_left_closest, left_dist);
    PublishClosest(pub_right_closest, right_dist);

    //Publish regions visualization
    PublishRegions();
}

void obstacle_loc::PublishPoints(ros::Publisher pub, int points){
    std_msgs::Int32 pts;
    pts.data = points;
    pub.publish(pts); 
}

void obstacle_loc::PublishClosest(ros::Publisher pub, double dist){
    std_msgs::Float64 closest;
    if(dist != DEFAULT_CLOSEST){
        closest.data = dist;
    }
    else{
        closest.data = -1.0f;
    }
    pub.publish(closest);
}

void obstacle_loc::PublishRegions(){

    visualization_msgs::MarkerArray regionsArray;

    regionsArray.markers.push_back(RegiontoMarker(front_close, 1, 0, 0, REGION_ALPHA, 0)); //Red Region
    regionsArray.markers.push_back(RegiontoMarker(front_far, 0, 1, 0, REGION_ALPHA, 1));   //Green Region
    regionsArray.markers.push_back(RegiontoMarker(left_close, 0, 0, 1, REGION_ALPHA, 2));  //Blue Region
    regionsArray.markers.push_back(RegiontoMarker(left_far, 1, 0, 1, REGION_ALPHA, 3));  //Blue Region
    regionsArray.markers.push_back(RegiontoMarker(right_close, 1, 1, 0, REGION_ALPHA, 4)); //Purple Region
    regionsArray.markers.push_back(RegiontoMarker(right_far, 0, 1, 1, REGION_ALPHA, 5));  //Blue Region

    vis_pub.publish(regionsArray);
}

visualization_msgs::Marker obstacle_loc::RegiontoMarker(region region, float red, float green, float blue, float alpha, int id){

    
    visualization_msgs::Marker marker;

    marker.header.frame_id = "near_field";
    marker.header.stamp = ros::Time();
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = region.originX + (region.length / 2.0f);
    marker.pose.position.y = region.originY + (region.width / 2.0f);
    marker.pose.position.z = (region.z_max + region.z_min) / 2.0f;
    marker.scale.x = region.length;
    marker.scale.y = region.width;
    marker.scale.z = region.z_max - region.z_min;
    marker.color.a = alpha;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.lifetime = ros::Duration(0.5);

    return marker;
}


void obstacle_loc::TransformToBase(actor_cloud_ptr_t& out, actor_cloud_ptr_t& in)
{
    if (in->points.size() == 0){
        return;
    }

    Eigen::Matrix<float, 1, 3> floor_plane_normal_vector, xy_plane_normal_vector;


    floor_plane_normal_vector[0] = config_.norm_x;
    floor_plane_normal_vector[1] = config_.norm_y;
    floor_plane_normal_vector[2] = config_.norm_z;

    //NORMAL of the XY plane <0,0,1> (aka the Z axis)
    xy_plane_normal_vector[0] = 0.0;
    xy_plane_normal_vector[1] = 0.0;
    xy_plane_normal_vector[2] = 1.0;


    Eigen::Vector3f rotation_vector = xy_plane_normal_vector.cross(floor_plane_normal_vector);
    float theta = config_.theta;

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.rotate (Eigen::AngleAxisf (theta, rotation_vector));
    pcl::transformPointCloud (*in, *out, transform_2);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "region");
    obstacle_loc obstacle_loc;

    ros::Rate r(10); 

    while (ros::ok()){
        if (obstacle_loc.hasSub()){
            if (!obstacle_loc.isEnabled()){
                obstacle_loc.startup();
            }
        } else {
            if (obstacle_loc.isEnabled()){
                obstacle_loc.shutdown();
            }
        }
        ros::spinOnce();
        r.sleep();
    }
}
