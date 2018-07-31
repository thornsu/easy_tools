# easy_tools
camera_model_calibration, imu and pic extract from rosbag ,creat imu rosbag

camera_calibration: 
1.opencv calibration example
2.part of [camodocal](https://github.com/hengli/camodocal),used in vins-mono


extract:
extract pic and imu data from rosbag, file format is look like euroc dataset
change your bag name and topic

imu_rosbag_creat:
change frome [kalibr_bagcerater](https://github.com/ethz-asl/kalibr)

euroc_to_tum:
if you want use [evo](https://github.com/MichaelGrupp/evo) to anaylis your slam trajectory, you need change to TUM rajectory file format, it's a tool to change vins-mono trajectory to tum
