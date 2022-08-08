std::string vehicle;
std::string bag_time;
int sensor_num;
int frame_num;
std::vector<std::string> sensor_vec_;
std::vector<geometry_msgs::Pose> poses_vec_;
std::vector<pg_lib::Transform> ECEF_transforms_vec_;
std::vector<pg_lib::Transform> IMU0_transforms_vec_;

std::string root_dirname_;
std::string config_filename_;
std::string data_dir_;

std::string imu_file_name_;
pointcloud_tools::SensorDataID init_id;
std::vector<std::string> field_names{"x","y","z","intensity"};



