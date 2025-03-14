define kinect ranger
(
  sensor
  (
    pose [ 0.0 0.0 -0.11 0.0 ]
    size [0.1 0.1 0.1 ]
    range  [0 12]
    fov 360.0
    samples 360
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define scout position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  
  localization "gps"

# variance of noise
  odom_error [0.05 0.05 0.0 0.02 ]

  size [ 0.612 0.580 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "omni"
  color "red"
  
  kinect()
)
