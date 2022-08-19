boost::shared_ptr<InteractiveMarkerServer> marker_server_;

ros::Publisher relative_frame_pub;
ros::Subscriber relative_pose_sub;

ros::Publisher edge_pub;
ros::Publisher pose_pub;
ros::Publisher pose_pc_pub;
ros::Publisher arrow_edge_pub;

ros::ServiceClient pointcloud_client;
ros::ServiceClient matching_result_client, ECEF_pose_result_client, send_configuration_client, Tf_broadcast_info_client;
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

geometry_msgs::Pose ref_pose, dest_pose;
int index_of_ref_pc = -1, index_of_in_pc = -1;

pg_editor::InitialConfigurationConfig global_config_;
dynamic_reconfigure::Server<pg_editor::InitialConfigurationConfig> *server_ptr_;

bool setFrameNum();
int findClosestPoint(tf::Vector3 new_vector);
void setRqtConfigByIndex();
void setMarkerColorByIndex();
void afterConfigurationSet();
void sendConfiguration();