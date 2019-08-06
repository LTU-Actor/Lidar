# Obstacle Detector

Uses sensor information to determine where objects are located around the vehicle. Regions allow for square areas to be defined around the vehicle to detect objects in specific locations.

## Required ROS Params
`~sub_topic` sets the topic the subscriber receives the lidar data on (sensor_msgs::PointCloud2).

**Note:** Our implementation only makes use of a 3D Velodyne lidar to detect objects.

## Example Launch File
```
<node name="region" pkg="ltu_actor_route_obstacle" type="region" >
  <param name="sub_topic" value="/lidar/vh_wm_points" />
</node>
```
