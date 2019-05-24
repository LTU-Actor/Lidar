# Actor Obstacle

## Launching 

```
<node name="region" pkg="ltu_actor_route_obstacle" type="region" >
  <param name="sub_topic" value="/lidar/vh_wm_points" />
</node>
```

or

```
<include file="$(find ltu_actor_route_obstacle)/launch/regions.launch" />
```
