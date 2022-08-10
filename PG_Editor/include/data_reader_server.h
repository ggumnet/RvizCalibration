std::vector<geometry_msgs::Pose> poses_vec_;
std::vector<pg_lib::Transform> ECEF_transforms_vec_;
std::vector<pg_lib::Transform> IMU0_transforms_vec_;

std::string data_dirname_;

std::string imu_file_name_;
std::vector<std::string> field_names{"x","y","z","intensity"};

bool configuration_set_done = false;