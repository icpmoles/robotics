## cmds

```bash
rosbag play -l src/bags/robotics.bag -r 10
rosrun first_project gps_to_odom
rosrun plotjuggler plotjuggler
roslaunch first_project launch.launch


catkin_make && roslaunch first_project launch.launch


LD_PRELOAD=/home/polimi/apitrace-latest-Linux/lib/apitrace/wrappers/glxtrace.so rviz -d  /home/polimi/robotics/src/first_project/lidar.rviz
```

## First Node

It receives the packets from the GPS receiver on the `/fix` topic with a `sensor_msgs/NavSatFix` type.

It then uses this data to convert into a local reference frame fixed on the starting position (or another arbitrary position provided by parameters) to provide odometric data on the `/gps_odom` topic with type `nav_msgs/Odometry` 

### Legend
```mermaid
flowchart TD
id99([topics])
exn(nodes)
LLA_paramsid1{{Configuration}}
```
### Flowchart
```mermaid
flowchart TD
LLA_paramsid{{Latitude\nLongitude\nAltitude}}-.-ogps2odom;
gps([/fix])--->gps2odom(gps_to_odom);
gps2odom-->id2([/gps_odom]);
id2-->gps_o2t(odom_to_tf);
gps_o2t-->gpstf([/gps_tf]);
id3([/wheel_odom])-->wheel_o2t(odom_to_tf);
wheel_o2t-->2ndtf([/wheel_tf]);
2ndtf-->3rdnode{lidar_select}
gpstf-->3rdnode;
rqt{{Dynamic\nSelector}}-.-o3rdnode;
3rdnode==>Rviz
```

In theory the NED should have |N|<90m and |E|<60m

![overlay of GPS track and satellite photo](map_estimate.png)