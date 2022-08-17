boost::shared_ptr<InteractiveMarkerServer> server;

ros::Publisher relative_frame_pub;
ros::Subscriber relative_pose_sub;

// marker and edges publisher
ros::Publisher edge_pub;
ros::Publisher pose_pub;
ros::Publisher pose_pc_pub;
ros::Publisher transforminfo_pub;
ros::Publisher arrow_edge_pub;

ros::ServiceClient pointcloud_client;
ros::ServiceClient matching_result_client, ECEF_pose_result_client, send_configuration_client;
pg_editor::GetNDTMatchingResult matching_result_service;

ros::Publisher pc_in_pub;
ros::Publisher pc_ref_pub;
ros::Publisher map_pc_pub;

std::vector<sensor_msgs::PointCloud2> pointcloud_vec_;
std::vector<ros::Publisher> pc_publisher_vec_;
std::vector<Transform> IMU_transform_vec_;
std::vector<geometry_msgs::Pose> IMU_pose_vec_;

std::map<int, pointcloud_tools::SensorDataID> time_step_to_sensorDataID_map;
std::map<int, sensor_msgs::PointCloud2> time_step_to_pointcloud_map;
std::map<int, Transform> time_step_to_init_transform_map;

Graph *graph_ptr;

std::vector<bool> pc_publish_or_not;

bool do_optimize = false;
bool configuration_set_done = false;
int add_or_remove;
int max_iteration;
int edge_distance_threshold;

int index_of_ref_pc = -1, index_of_in_pc = -1;

bool setFrameNum();