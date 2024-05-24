In case of windows freezing and acting up:

cd ~/.config/ros.org/


23/05/2024 fix, mo more dual vm


# L10

cd src/stage
rosrun stage_ros stageros maze.world
 roslaunch demo_mapping slam_toolbox_localization.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

-> panels -> slam toolbox panel -> serialize $path
/ugv/rslidar_points
#