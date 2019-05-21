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

#include <ltu_actor_route_obstacle/Region.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include <ltu_actor_route_obstacle/RegionConfig.h>

using actor_cloud_t = pcl::PointCloud<pcl::PointXYZ>;
using actor_cloud_ptr_t = actor_cloud_t::Ptr;

#define REGION_ALPHA 0.4f

struct region{
    float originX;
    float originY;
    float width;
    float length;
    float z_min;
    float z_max;
    float z_depreciation;

    //std::list<geometry_msgs::Point> points;
    int threshold;
    int points = 0;
};

class obstacle_loc{

    private:
        void TransformToBase(actor_cloud_ptr_t& out, actor_cloud_ptr_t& in);
        bool WithinRegion(const float& x, const float& y, const float& z, region reg);
        void configCallback(ltu_actor_route_obstacle::RegionConfig &config, uint32_t level);

        // Callback
        void CloudCallback(const sensor_msgs::PointCloud2ConstPtr&);

        //Region Publishing
        void PublishRegions();
        visualization_msgs::Marker RegiontoMarker(region region, float red, float green, float blue, float alpha, int id);

        ros::NodeHandle nh_;
        ros::Publisher  pub_;
        ros::Publisher  cloud_pub;
        ros::Publisher vis_pub;
        ros::Subscriber sub_;
        ros::Publisher pub_front_far;
        ros::Publisher pub_front_close;

        region front_close;
        region front_far;
        region left_close;
        region left_far; //not used
        region right_close ;
        region right_far; //not used;

        std::string sub_topic;

        dynamic_reconfigure::Server<ltu_actor_route_obstacle::RegionConfig> server_;
        ltu_actor_route_obstacle::RegionConfig config_;

        bool enabled;
    public:
        obstacle_loc();

        bool isEnabled() {return enabled;}
        void shutdown() {
            sub_ = ros::Subscriber();
            enabled =  false;
        }
        void startup(){
            sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(sub_topic, 100, &obstacle_loc::CloudCallback, this);
            enabled = true;
        }
        bool hasSub(){
            return true;//(pub_.getNumSubscribers() || cloud_pub.getNumSubscribers() || vis_pub.getNumSubscribers()) > 0;
        }
};

// constructor
// set up publisher and subscriber
obstacle_loc::obstacle_loc()
{
    enabled = false;

    front_close.originX = 1.0f;
    front_close.originY = -0.5f;
    front_close.width = 1.0f;
    front_close.length = 4.0f;
    front_close.z_min = -1.25f;
    front_close.z_max = 0.0f;
    front_close.z_depreciation = 0.0f;

    front_far.originX = 4.0f;
    front_far.originY = -0.5f;
    front_far.width = 1.0f;
    front_far.length = 4.0f;
    front_far.z_min = -1.25f;
    front_far.z_max = 0.0f;
    front_far.z_depreciation = 0.5f;

    /*left_close.originX = 1.0f;
    left_close.originY = 0.5f;
    left_close.width = 1.0f;
    left_close.length = 4.0f;
    left_close.z_min = -1.25f;
    left_close.z_max = 0.0f;
    left_close.z_depreciation = 0.0f;

    right_close.originX = 1.0f;
    right_close.originY = -1.5f;
    right_close.width = 1.0f;
    right_close.length = 4.0f;
    right_close.z_min = -1.25f;
    right_close.z_max = 0.0f;
    right_close.z_depreciation = 0.0f;*/

    front_close.threshold = 10;
    front_far.threshold = 10;
    //left_close.threshold = 10;
    //right_close.threshold = 10;

    nh_ = ros::NodeHandle("~");

    if (!nh_.getParam("input", sub_topic))
    {
        ROS_ERROR_STREAM("No lidar topic passed to " + sub_topic);
        throw std::invalid_argument("Bad lidar topic.");
    }

    //pub_ = nh_.advertise<ltu_actor_route_obstacle::Region>("regions", 10);
    //cloud_pub = nh_.advertise<actor_cloud_t>("centered_cloud",10);
    vis_pub = nh_.advertise<visualization_msgs::MarkerArray>( "/regions_visualization", 10);
    sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(sub_topic, 100, &obstacle_loc::CloudCallback, this);

    pub_front_far = nh_.advertise<std_msgs::Bool>("front_far", 10);
    pub_front_close = nh_.advertise<std_msgs::Bool>("front_close", 10);

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
    front_close.z_depreciation = config_.front_close_z_depreciation;
    front_close.threshold = config_.front_close_threshold;

    //Front Far Config Settings
    front_far.originX = config_.front_far_originX;
    front_far.originY = config_.front_far_originY;
    front_far.width = config_.front_far_width;
    front_far.length = config_.front_far_length;
    front_far.z_min = config_.front_far_z_min;
    front_far.z_max = config_.front_far_z_max;
    front_far.z_depreciation = config_.front_far_z_depreciation;
    front_far.threshold = config_.front_far_threshold;

    //Left Close Config Settings
    left_close.originX = config_.left_close_originX;
    left_close.originY = config_.left_close_originY;
    left_close.width = config_.left_close_width;
    left_close.length = config_.left_close_length;
    left_close.z_min = config_.left_close_z_min;
    left_close.z_max = config_.left_close_z_max;
    left_close.z_depreciation = config_.left_close_z_depreciation;
    left_close.threshold = config_.left_close_threshold;

    //Right Close Config Settings
    right_close.originX = config_.right_close_originX;
    right_close.originY = config_.right_close_originY;
    right_close.width = config_.right_close_width;
    right_close.length = config_.right_close_length;
    right_close.z_min = config_.right_close_z_min;
    right_close.z_max = config_.right_close_z_max;
    right_close.z_depreciation = config_.right_close_z_depreciation;
    right_close.threshold = config_.right_close_threshold;
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

    cloud_pub.publish(*centeredCloud);

    //Set Regions
    for (auto p : *centeredCloud){
        if (WithinRegion(p.x, p.y, p.z, front_close)){
            front_close.points++;
        }
        else if (WithinRegion(p.x, p.y, p.z, front_far)){
            front_far.points++;
        }
        else if (WithinRegion(p.x, p.y, p.z, left_close)){
            left_close.points++;
        }
        else if (WithinRegion(p.x, p.y, p.z, right_close)){
            right_close.points++;
        }
    }

    ltu_actor_route_obstacle::Region regions;
    regions.header = msg->header;

    regions.front_close_region_points = front_close.points;
    regions.front_far_region_points = front_far.points;
    regions.left_close_region_points = left_close.points;
    regions.right_close_region_points = right_close.points;


    if(front_close.points > front_close.threshold){ 
        regions.front_close_region = true;
        std_msgs::Bool bReg;
        bReg.data = true;
        pub_front_close.publish(bReg);
    }
    else{ 
        regions.front_close_region = false; 
        std_msgs::Bool bReg;
        bReg.data = false;
        pub_front_close.publish(bReg);
    }
    
    if(front_far.points > front_far.threshold){ 
        regions.front_far_region = true; 
        std_msgs::Bool bReg;
        bReg.data = true;
        pub_front_far.publish(bReg);
    }
    else{ 
        regions.front_far_region = false; 
        std_msgs::Bool bReg;
        bReg.data = false;
        pub_front_far.publish(bReg);
    }
    
    if(left_close.points > left_close.threshold){ regions.left_close_region = true; }
    else{ regions.left_close_region = false; }
    
    if(right_close.points > right_close.threshold){ regions.right_close_region = true; }
    else{ regions.right_close_region = false; }

    front_close.points = 0;
    front_far.points = 0;
    left_close.points = 0;
    right_close.points = 0;

    PublishRegions();
}

void obstacle_loc::PublishRegions(){

    visualization_msgs::MarkerArray regionsArray;

    regionsArray.markers.push_back(RegiontoMarker(front_close, 1, 0, 0, REGION_ALPHA, 0)); //Red Region
    regionsArray.markers.push_back(RegiontoMarker(front_far, 0, 1, 0, REGION_ALPHA, 1));   //Green Region
    regionsArray.markers.push_back(RegiontoMarker(left_close, 0, 0, 1, REGION_ALPHA, 2));  //Blue Region
    regionsArray.markers.push_back(RegiontoMarker(right_close, 1, 0, 1, REGION_ALPHA, 3)); //Purple Region

    vis_pub.publish(regionsArray);
}

visualization_msgs::Marker obstacle_loc::RegiontoMarker(region region, float red, float green, float blue, float alpha, int id){

    
    visualization_msgs::Marker marker;

    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time();
    //marker.ns = "my_namespace";
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
