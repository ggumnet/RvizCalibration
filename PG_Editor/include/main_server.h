boost::shared_ptr<InteractiveMarkerServer> server;

ros::Publisher relative_frame_pub;
ros::Subscriber relative_pose_sub;

// marker and edges publisher
ros::Publisher edge_pub;
ros::Publisher pose_pub;
ros::Publisher pose_pc_pub;
ros::Publisher transforminfo_pub;

ros::ServiceClient pointcloud_client;
ros::ServiceClient matching_result_client, imu_pose_result_client;
pg_editor::GetNDTMatchingResult matching_result_service;

std::vector<sensor_msgs::PointCloud2> pointcloud_vec_;
std::vector<ros::Publisher> pc_publisher_vec_;
std::vector<Transform> transform_vec_;
std::vector<geometry_msgs::Pose> IMU_pose_vec_;

std::map<int, pointcloud_tools::SensorDataID> time_step_to_sensorDataID_map;
std::map<int, sensor_msgs::PointCloud2> time_step_to_pointcloud_map;
std::map<int, Transform> time_step_to_init_transform_map;

Graph *graph_ptr;

std::string vehicle;
std::string bag_time;
int sensor_num;
int frame_num;
std::vector<std::string> sensor_vec_;

std::vector<bool> pc_publish_or_not;

std::string root_dirname_;
std::string config_filename_;
pointcloud_tools::SensorDataID init_id;


bool do_optimize = false;
int add_or_remove;
