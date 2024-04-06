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
gps([/fix])--->gps2odom(gps2odom);
gps2odom-->id2([/gps_odom]);
id2-->gps_o2t(odom2tf);
gps_o2t-->gpstf([/gps_tf]);
id3([/wheel_odom])-->wheel_o2t(odom2tf);
wheel_o2t-->2ndtf([/wheel_tf]);
2ndtf-->3rdnode{TF\nbroadcaster}
gpstf-->3rdnode;
rqt{{Dynamic\nSelector}}-.-o3rdnode;
3rdnode==>Rviz
```