// Contain main graph
// optimize graph and visualize it(publish pc and markers of graph)

#include <ros/ros.h>
#include <pg_lib/graph.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <pg_editor/TransformInfo.h>
#include <pg_editor/RelativeFramesInfo.h>
#include <pg_editor/RelativePoseInfo.h>
#include <pg_editor/GetPointcloud.h>
#include <pg_editor/GetNDTMatchingResult.h>
#include <pg_editor/GetImuPoseResult.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32MultiArray.h>
#include <boost/tokenizer.hpp>


using namespace visualization_msgs;
using namespace interactive_markers;
using namespace pg_lib;

#include <rot2quat.h>
#include <transform_pose_conversion.h>
#include <print_tool.h>

#define ADD 0
#define REMOVE 1
#define COST_TYPE Cost::TYPE::SQUARED

pointcloud_tools::SensorDataID init_id;
boost::shared_ptr<InteractiveMarkerServer> server;


ros::Publisher first_pub;
ros::Publisher second_pub;

ros::Publisher relative_frame_pub;
ros::Subscriber relative_pose_sub;

// marker and edges publisher
ros::Publisher edge_pub;
ros::Publisher pose_pub;
ros::Publisher pose_pc_pub;
ros::Publisher transforminfo_pub;

std::vector<sensor_msgs::PointCloud2> pointcloud_vec_;
std::vector<ros::Publisher> pc_publisher_vec_;
std::vector<Transform> transform_vec_;
std::vector<geometry_msgs::Pose> IMU_pose_vec_;

std::map<int, pointcloud_tools::SensorDataID> time_step_to_sensorDataID_map;
std::map<int, sensor_msgs::PointCloud2> time_step_to_pointcloud_map;
std::map<int, Transform> time_step_to_init_transform_map;

ros::ServiceClient pointcloud_client;
ros::ServiceClient matching_result_client, imu_pose_result_client;

pg_editor::GetNDTMatchingResult matching_result_service;

Graph *graph_ptr;

//Reference frame: "map" frame

//TOSET
namespace initconfiguration
{
    const int frame_num = 12;
    std::string root_dirname_ = "/home/rideflux/v5_1_sample_data_1/";
    std::string config_filename_ = root_dirname_+"configuration.txt";
    // TOSET
    void initSensorInfo()
    {
        init_id.vehicle = "solati_v5_1";
        init_id.bag_time = "2022-07-14-11-46-25";
        init_id.sensor = "pandar64_0";
    }
    void parse_config_data(const std::string &str, const std::string delimiters)
    {
        boost::char_separator<char> sep(delimiters.c_str());
        boost::tokenizer<boost::char_separator<char>> tok(str, sep);
        boost::tokenizer<boost::char_separator<char>>::iterator itr = tok.begin();
        
        init_id.vehicle = *(itr++);
        init_id.bag_time = *(itr++);
        for(;itr!=tok.end(); itr++){
            init_id.sensor = *itr;
        }
        ROS_WARN("config read done");
    }
    void readConfiguration()
    {
        std::string config_read_string = "";
        std::ifstream config_file(config_filename_);
        if (config_file.is_open())
        {
            while (config_file)
            {
                std::string s;
                getline(config_file, s);
                config_read_string += s + "\n";
            }
            config_file.close();
        }
        else
        {
            std::cout << "file open failed" << std::endl;
        }
        std::string delimiters = " \n\t";
        parse_config_data(config_read_string, delimiters);
    }
    void initPointclouds()
    {
        pg_editor::GetPointcloud pointcloud_service;
        rideflux_msgs::SensorDataID data_id;
        data_id.vehicle = init_id.vehicle;
        data_id.sensor = init_id.sensor;
        data_id.bag_time = init_id.bag_time;
        for (int i = 0; i < frame_num; i++)
        {
            data_id.time_step = i;
            pointcloud_service.request.data_id = data_id;
            if (pointcloud_client.call(pointcloud_service))
            {
                auto pc = pointcloud_service.response.pointcloud;
                ROS_INFO("frame_id, width: %s, %d", pc.header.frame_id.c_str(), pc.width);
                pointcloud_vec_.push_back(pc);
            }
        }
    }
    void initPointcloudPublisherList(ros::NodeHandle &nh)
    {
        ros::Publisher publisher;
        for (int i = 0; i < initconfiguration::frame_num; i++)
        {
            publisher = nh.advertise<sensor_msgs::PointCloud2>("/pc_" + std::to_string(i), 1);
            pc_publisher_vec_.push_back(publisher);
        }
    }
    void initSensorDataID()
    {
        pointcloud_tools::SensorDataID temp_id = init_id;
        for(int i=0; i<frame_num; i++){
            temp_id.time_step = i;
            time_step_to_sensorDataID_map.insert(std::make_pair(i, temp_id));
        }
    }
}
bool do_optimize = false;
int add_or_remove;
bool pc_publish_or_not[initconfiguration::frame_num] = {true, true, true, true, true, true, true, true, true, true, true, true};

namespace markerhandling
{
    // make marker at the certain position
    InteractiveMarker makeEmptyMarker(geometry_msgs::Pose pose, std::string frame_id)
    {
        InteractiveMarker int_marker;
        int_marker.header.frame_id = frame_id;
        int_marker.header.stamp = ros::Time::now();
        int_marker.pose.position.x = pose.position.x;
        int_marker.pose.position.y = pose.position.y;
        int_marker.pose.position.z = pose.position.z;
        int_marker.pose.orientation = pose.orientation;
        int_marker.scale = 1;
        return int_marker;
    }
    // make box according to the size of InteractiveMarker
    Marker makeBox(InteractiveMarker &msg)
    {
        Marker marker;
        marker.type = Marker::SPHERE;
        marker.scale.x = msg.scale * 0.45;
        marker.scale.y = msg.scale * 0.45;
        marker.scale.z = msg.scale * 0.45;
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
        marker.color.a = 1.0;
        return marker;
    }
    // make menu marker and push it to server
    void makeMenuMarker(std::string name, geometry_msgs::Pose pose, std::string frame_id)
    {
        InteractiveMarker int_marker = markerhandling::makeEmptyMarker(pose, frame_id);
        int_marker.name = name;

        InteractiveMarkerControl control;

        control.interaction_mode = InteractiveMarkerControl::BUTTON;
        control.always_visible = true;

        control.markers.push_back(markerhandling::makeBox(int_marker));
        int_marker.controls.push_back(control);

        server->insert(int_marker);
        server->applyChanges();
    }
    MenuHandler initMenu(geometry_msgs::Pose pose)
    {
        MenuHandler menu_handler;
        MenuHandler::EntryHandle pose_menu, set_menu;
        pose_menu = menu_handler.insert("pose");
        menu_handler.insert(pose_menu, "tx " + std::to_string(pose.position.x));
        menu_handler.insert(pose_menu, "ty " + std::to_string(pose.position.y));
        menu_handler.insert(pose_menu, "tz " + std::to_string(pose.position.z));
        menu_handler.insert(pose_menu, "qx " + std::to_string(pose.orientation.x));
        menu_handler.insert(pose_menu, "qy " + std::to_string(pose.orientation.y));
        menu_handler.insert(pose_menu, "qz " + std::to_string(pose.orientation.z));
        menu_handler.insert(pose_menu, "qw " + std::to_string(pose.orientation.w));
        return menu_handler;
    }
}
namespace graphhandling{
    geometry_msgs::Pose pose_array[initconfiguration::frame_num];
}

geometry_msgs::Pose ref_pose, dest_pose;

void setRefFrame(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ref_pose = feedback.get()->pose;
}
void setDestFrame(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    dest_pose = feedback.get()->pose;
}
void optimizeGraph(Graph &graph)
{
    (*graph_ptr).optimize(false);
}
void visualizeGraph(const Graph &graph, ros::Publisher &edge_pub, ros::Publisher &poses_pub, ros::Publisher &pose_pc_pub, std::string frame_id)
{
    visualization_msgs::Marker marker;
    geometry_msgs::PoseArray pose_array;
    std::vector<std::size_t> indices;
    graph.createMsgToVisualize(marker, pose_array, indices);

    if (!indices.empty())
    {
        marker.header.frame_id = pose_array.header.frame_id = frame_id;
        marker.header.stamp = pose_array.header.stamp = ros::Time::now();
        // marker.type = visualization_msgs::Marker::ARROW;

        marker.id = 0;
        marker.ns = "edges";
        marker.scale.x = 0.2;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;

        edge_pub.publish(marker);
        poses_pub.publish(pose_array);
        int i = 0;
        for (auto pose : pose_array.poses)
        {
            MenuHandler menu_handler;
            i++;
            markerhandling::makeMenuMarker("marker" + std::to_string(i), pose, frame_id); // make menu marker and add to server
            // ROS_INFO("done");
            menu_handler = markerhandling::initMenu(pose);
            menu_handler.apply(*server, "marker" + std::to_string(i)); // apply menu entry to menu marker
        }
    }
    server->applyChanges();
}

bool addAbsFactor(Graph &graph, pointcloud_tools::SensorDataID &id, Transform &T, ParamMatrix &H)
{
    auto pose = graph.getVariable<Pose>(id, true);
    
    Factor::Ptr factor_abs = std::make_shared<AbsolutePoseFactor>(pose, T, H);
    factor_abs->setIsReliable(true);
    factor_abs->setCost(COST_TYPE);

    if(!graph.addFactor(factor_abs))
    {
        ROS_ERROR("Failed to add factor_abs. %s", toString(id).c_str());
        return false;
    }
    return true;
}


bool addAbsFactorWithSensor(Graph &graph, pointcloud_tools::SensorDataID &id, pointcloud_tools::SensorFrameID &sensor_id, Transform &T, ParamMatrix &H)
{
    Pose::Ptr pose = graph.getVariable<Pose>(id, true);
    Pose::Ptr sensor = graph.getSensorVariable(sensor_id, true);


    Factor::Ptr factor_abs = std::make_shared<AbsolutePoseFactor>(pose, sensor, T, H);
    factor_abs->setIsReliable(true);
    factor_abs->setCost(COST_TYPE);

    if(!graph.addFactor(factor_abs))
    {
        ROS_ERROR("Failed to add factor_abs. %s", toString(id).c_str());
        return false;
    }
    return true;
}

bool addRelativeFactor(Graph &graph, pointcloud_tools::SensorDataID &id_ref, pointcloud_tools::SensorDataID &id_in, Transform &T, ParamMatrix &H)
{
    auto pose_ref = graph.getVariable<Pose>(id_ref, true);
    auto pose_in = graph.getVariable<Pose>(id_in, true);

    Factor::Ptr factor = std::make_shared<RelativePose2Factor>(pose_ref, pose_in, T, H);
    
    factor->setIsReliable(true);
    factor->setCost(COST_TYPE);

    if (!graph.addFactor(factor))
    {
        ROS_ERROR("Failed to add factor between %s and %s poses.", toString(id_ref).c_str(), toString(id_in).c_str());
        return false;
    }
    return true;
}

bool addRelativeFactorWithSensor(Graph &graph, pointcloud_tools::SensorDataID &id_ref, pointcloud_tools::SensorDataID &id_in, pointcloud_tools::SensorFrameID &sensor_id_ref, pointcloud_tools::SensorFrameID &sensor_id_in, Transform &T, ParamMatrix &H)
{
    auto pose_ref = graph.getVariable<Pose>(id_ref, true);
    auto pose_in = graph.getVariable<Pose>(id_in, true);

    auto sensor_ref = graph.getSensorVariable(sensor_id_ref, true);
    auto sensor_in = graph.getSensorVariable(sensor_id_in, true);

    Factor::Ptr factor = std::make_shared<RelativePose2Factor>(pose_ref, pose_in, sensor_ref, sensor_in, T, H);

    factor->setIsReliable(true);
    factor->setCost(COST_TYPE);

    if (!graph.addFactor(factor))
    {
        ROS_ERROR("Failed to add factor between %s and %s poses.", toString(id_ref).c_str(), toString(id_in).c_str());
        return false;
    }
    return true;
}

void removeRelativeFactor(Graph &graph, pointcloud_tools::SensorDataID &id_ref, pointcloud_tools::SensorDataID &id_in, Transform &T, ParamMatrix &H)
{
    auto pose_ref = graph.getVariable<Pose>(id_ref, true);
    auto pose_in = graph.getVariable<Pose>(id_in, true);

    Factor::Ptr factor = std::make_shared<RelativePose2Factor>(pose_ref, pose_in, T, H);
    factor->setIsReliable(true);

    graph.removeFactor(factor);
    return;
}

// get ndt result pose and add it to graph
void responseRelativeFactor(pg_editor::GetNDTMatchingResult matching_result_service)
{
    Transform T;
    geometry_msgs::Pose pose = matching_result_service.response.result_pose;
    // std::string frame1 = matching_result_service.request.pointcloud1.header.frame_id;
    // std::string frame2 = matching_result_service.request.pointcloud2.header.frame_id;

    //TODO
    int frame_num1 = matching_result_service.request.time_step1;
    int frame_num2 = matching_result_service.request.time_step2;
    T(0, 3) = pose.position.x;
    T(1, 3) = pose.position.y;
    T(2, 3) = pose.position.z;
    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    const auto &R = tf::Matrix3x3(q);
    for (std::size_t i = 0; i < DIM; ++i)
    {
        for (std::size_t j = 0; j < DIM; ++j)
            T(i, j) = R[i][j];
    }
    printTransform(T);

    pointcloud_tools::SensorFrameID sensor_id;
    sensor_id.frame_id = init_id.sensor;
    sensor_id.vehicle = init_id.vehicle;

    ParamMatrix H;
    for (std::size_t i = 0; i < PARAM_DIM; i++)
        H(i, i) = 0.01;

    if (add_or_remove == ADD)
        addRelativeFactorWithSensor(*graph_ptr, time_step_to_sensorDataID_map[frame_num1], time_step_to_sensorDataID_map[frame_num2], sensor_id, sensor_id, T, H);
    else if (add_or_remove = REMOVE)
        removeRelativeFactor(*graph_ptr, time_step_to_sensorDataID_map[frame_num1], time_step_to_sensorDataID_map[frame_num2], T, H);

    if (do_optimize)
    {
        optimizeGraph(*graph_ptr);
    }
    // ROS_INFO("call back done");
}

// Request Relative Pose To NDT_matching node
void requestRelativeFactor(int source_time_step, int dest_time_step)
{
    pg_editor::GetNDTMatchingResult matching_result_service;
    Transform T_source, T_dest;

    T_source = time_step_to_init_transform_map[source_time_step];
    T_dest = time_step_to_init_transform_map[dest_time_step];

    matching_result_service.request.pointcloud1 = time_step_to_pointcloud_map[source_time_step];
    matching_result_service.request.pointcloud2 = time_step_to_pointcloud_map[dest_time_step];
    matching_result_service.request.time_step1 = source_time_step;
    matching_result_service.request.time_step2 = dest_time_step;
    matching_result_service.request.initial_pose = transformToPose(T_source.inv() * T_dest);

    if (matching_result_client.call(matching_result_service))
    {
        responseRelativeFactor(matching_result_service);
    }
}

void visualizePointclouds()
{
    for (int i = 0; i < initconfiguration::frame_num; i++)
    {
        if (pc_publish_or_not[i])
        {
            pointcloud_vec_.at(i).header.stamp = ros::Time::now();
            pointcloud_vec_.at(i).header.frame_id = "sensor_frame"+std::to_string(i);
            pc_publisher_vec_.at(i).publish(pointcloud_vec_.at(i));
        }
    }
}

void publishResults(Graph &graph, tf::TransformBroadcaster &broadcaster, tf::TransformBroadcaster &sensor_broadcaster)
{
    visualization_msgs::Marker marker;
    geometry_msgs::PoseArray pose_array;
    std::vector<std::size_t> indices;
    graph.createMsgToVisualize(marker, pose_array, indices);

    pointcloud_tools::SensorFrameID sensor_id;
    sensor_id.frame_id = "pandar64_0";
    sensor_id.vehicle = "solati_v5_1";
    auto sensor_var = graph.getSensorVariable(sensor_id);
    // if(*sensor_var == *Pose::Ptr())
    //     ROS_WARN("Can not find %s/%s sensor variable in graph",sensor_id.vehicle.c_str(), sensor_id.frame_id.c_str());
    auto sensor_T = sensor_var->getData();

    auto sensor_tf = poseToTfTransform(transformToPose(sensor_T));
    for (int i = 0; i < initconfiguration::frame_num; i++)
    {
        //printPose(pose_array.poses.at(i));
        broadcaster.sendTransform(tf::StampedTransform(poseToTfTransform(pose_array.poses.at(i)), ros::Time::now(), "map", "frame"+std::to_string(i)));
        sensor_broadcaster.sendTransform(tf::StampedTransform(sensor_tf, ros::Time::now(), "frame"+std::to_string(i), "sensor_frame"+std::to_string(i)));
        pg_editor::TransformInfo transforminfo;
        transforminfo.pose = pose_array.poses.at(i);
        transforminfo.frame_num = i;
        transforminfo_pub.publish(transforminfo);
    }
    ros::spinOnce();

    visualizeGraph(graph, edge_pub, pose_pub, pose_pc_pub, "map");
    visualizePointclouds();
}
// arent you infected in COVID19?
// TOSET
void addAbsFactorFromIMUPose(Graph &graph)
{
    pointcloud_tools::SensorDataID temp_id =init_id;
    pointcloud_tools::SensorFrameID sensor_id;
    sensor_id.frame_id = "nov_imu";
    sensor_id.vehicle = "solati_v5_1";
    Transform T;
    ParamMatrix H(ParamMatrix::eye());

    for (std::size_t i = 0; i < PARAM_DIM; i++)
        H(i, i) = 0.00001;  
    for(int i=0; i<initconfiguration::frame_num; i++){
        temp_id.time_step = i;
        T = poseToTransform(IMU_pose_vec_.at(i));
        addAbsFactorWithSensor(graph, temp_id, sensor_id, T, H);
    }
}

void addIndexArrayCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    ROS_INFO("Add factor");
    // publish "/relative_frame"
    for (int i = 0; i < initconfiguration::frame_num; i++)
    {
        pc_publish_or_not[i] = true;
    }
    add_or_remove = ADD;
    requestRelativeFactor(msg->data.at(0), msg->data.at(1));
}

void removeIndexArrayCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    ROS_INFO("Remove factor");
    // publish "/relative_frame"
    for (int i = 0; i < initconfiguration::frame_num; i++)
    {
        pc_publish_or_not[i] = true;
    }
    add_or_remove = REMOVE;
    requestRelativeFactor(msg->data.at(0), msg->data.at(1));
}

void pcPublishIndexArrayCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    for (int i = 0; i < initconfiguration::frame_num; i++)
    {
        pc_publish_or_not[i] = msg->data.at(i) == 1;
    }
}
void saveImuPosesFromSrv(pg_editor::GetImuPoseResult &get_imu_pose_result_srv){
    geometry_msgs::Pose pose;
    ROS_INFO("%d", get_imu_pose_result_srv.response.pose_array.poses.size());
    for(int i=0; i<get_imu_pose_result_srv.response.pose_array.poses.size(); i++){
        pose = get_imu_pose_result_srv.response.pose_array.poses.at(i);
        transform_vec_.push_back(poseToTransform(pose));
        IMU_pose_vec_.push_back(pose);
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_server");
    ros::NodeHandle nh("~");

    Graph graph;
    graph_ptr = &graph;
    pointcloud_client = nh.serviceClient<pg_editor::GetPointcloud>("/pc_read_service");

    //initconfiguration::initSensorInfo();
    initconfiguration::readConfiguration();
    initconfiguration::initPointclouds();
    initconfiguration::initPointcloudPublisherList(nh);
    initconfiguration::initSensorDataID();

    first_pub = nh.advertise<visualization_msgs::Marker>("/first_marker", 1);
    second_pub = nh.advertise<visualization_msgs::Marker>("/second_marker", 1);
    edge_pub = nh.advertise<visualization_msgs::Marker>("graph_edge", 1, true);
    pose_pub = nh.advertise<geometry_msgs::PoseArray>("graph_pose", 1, true);
    pose_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("graph_pose_pc", 1, true);
    transforminfo_pub = nh.advertise<pg_editor::TransformInfo>("/transform_info", 1, true);

    ros::Subscriber add_index_subs = nh.subscribe<std_msgs::Int32MultiArray>("/add_edge_index_array", 10, addIndexArrayCallback);
    ros::Subscriber remove_index_subs = nh.subscribe<std_msgs::Int32MultiArray>("/remove_edge_index_array", 10, removeIndexArrayCallback);
    ros::Subscriber pc_publish_index_subs = nh.subscribe<std_msgs::Int32MultiArray>("/pc_publish_index_array", 1, pcPublishIndexArrayCallback);

    matching_result_client = nh.serviceClient<pg_editor::GetNDTMatchingResult>("/matching_result");
    imu_pose_result_client = nh.serviceClient<pg_editor::GetImuPoseResult>("/imu_pose_result");

    pg_editor::GetImuPoseResult get_imu_pose_result_srv;
    get_imu_pose_result_srv.request.temp = 0;
    imu_pose_result_client.call(get_imu_pose_result_srv);
    saveImuPosesFromSrv(get_imu_pose_result_srv);
    addAbsFactorFromIMUPose(graph);

    pointcloud_tools::SensorFrameID lidar_sensor_id;
    lidar_sensor_id.frame_id = init_id.sensor;
    lidar_sensor_id.vehicle = init_id.vehicle;
    graph.getSensorVariable(lidar_sensor_id, true);

    do_optimize = true;
    optimizeGraph(graph);

    server.reset(new InteractiveMarkerServer("pose_graph_example_node", "", false));

    tf::TransformBroadcaster broadcaster;
    tf::TransformBroadcaster sensor_broadcaster;

    while (ros::ok())
    {
        publishResults(graph, broadcaster, sensor_broadcaster);
        ros::spinOnce();
        ros::Duration(0.3).sleep();
    }
    return 0;
}