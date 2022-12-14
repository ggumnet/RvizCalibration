// Contain main graph
// optimize graph and visualize it(publish pc and markers of graph)

#include <stdexcept>
#include <ros/ros.h>
#include <pg_lib/graph.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <experimental/filesystem>

//srv
#include <pg_editor/RelativeFramesInfo.h>
#include <pg_editor/RelativePoseInfo.h>
#include <pg_editor/GetPointcloud.h>
#include <pg_editor/GetNDTMatchingResult.h>
#include <pg_editor/GetImuPoseResult.h>
#include <pg_editor/TfBroadcastInfo.h>
#include <pg_editor_panel/GetCalibration.h>

//cfg
#include <pg_editor/InitialConfigurationConfig.h>
#include <pg_editor/SendConfiguration.h>

#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32MultiArray.h>
#include <boost/tokenizer.hpp>
#include <dynamic_reconfigure/server.h>

using namespace visualization_msgs;
using namespace interactive_markers;
using namespace pg_lib;

#include "main_server.h"
#include "configurations.h"
#include "read_configuration.h"
#include "rot2quat.h"
#include "type_conversion.h"
#include "print_tool.h"

#define ADD 0
#define REMOVE 1
#define COST_TYPE Cost::TYPE::SQUARED
#define INDEX_NOT_SET -1

//Reference frame: "map" frame

//TOSET
namespace init
{
    void pointclouds()
    {
        pg_editor::GetPointcloud pointcloud_service;
        rideflux_msgs::SensorDataID data_id;
        data_id.vehicle = vehicle;
        data_id.sensor = sensor_vec_.at(0);
        data_id.bag_time = bag_time;
        pointcloud_vec_.clear();
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
    void pointcloudPublishers(ros::NodeHandle &nh)
    {
        pc_in_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_in", 1);
        pc_ref_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_ref", 1);
        map_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_pc", 1);
    }
    void sensorDataIDMap()
    {
        pointcloud_tools::SensorDataID temp_id = init_id;
        for(int i=0; i<frame_num; i++){
            temp_id.time_step = i;
            time_step_to_sensorDataID_map.insert(std::make_pair(i, temp_id));
        }
    }
}

namespace markerhandler
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

    // make sphere according to the size of InteractiveMarker
    Marker makeSphere(InteractiveMarker &msg)
    {
        Marker marker;
        marker.type = Marker::SPHERE;
        marker.scale.x = msg.scale * 0.35;
        marker.scale.y = msg.scale * 0.35;
        marker.scale.z = msg.scale * 0.35;
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
        marker.color.a = 1.0;
        return marker;
    }
    // make menu marker and push it to server
    void makeMenuMarker(std::string name, geometry_msgs::Pose pose, std::string frame_id)
    {
        InteractiveMarker int_marker = markerhandler::makeEmptyMarker(pose, frame_id);
        int_marker.name = name;

        InteractiveMarkerControl control;

        control.interaction_mode = InteractiveMarkerControl::BUTTON;
        control.always_visible = true;

        control.markers.push_back(markerhandler::makeSphere(int_marker));
        int_marker.controls.push_back(control);

        marker_server_->insert(int_marker);
        marker_server_->applyChanges();
    }
    void setFirst(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
        ROS_INFO("set first");
        return;
        ROS_INFO("set first: %d", feedback.get()->menu_entry_id);
    }
    void setSecond(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
        ROS_INFO("set second");
        return;
        ROS_INFO("set second: %d", feedback.get()->menu_entry_id);
    }
    MenuHandler initMenu(geometry_msgs::Pose pose)
    {
        MenuHandler menu_handler;
        MenuHandler::EntryHandle pose_menu, set_menu;
        pose_menu = menu_handler.insert("pose");
        set_menu = menu_handler.insert("set");
        menu_handler.insert(pose_menu, "tx " + std::to_string(pose.position.x));
        menu_handler.insert(pose_menu, "ty " + std::to_string(pose.position.y));
        menu_handler.insert(pose_menu, "tz " + std::to_string(pose.position.z));
        menu_handler.insert(pose_menu, "qx " + std::to_string(pose.orientation.x));
        menu_handler.insert(pose_menu, "qy " + std::to_string(pose.orientation.y));
        menu_handler.insert(pose_menu, "qz " + std::to_string(pose.orientation.z));
        menu_handler.insert(pose_menu, "qw " + std::to_string(pose.orientation.w));
        menu_handler.insert(set_menu, "Set First", &setFirst);
        menu_handler.insert(set_menu, "Set Second", &setSecond);
        return menu_handler;
    }
}

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
    graph.optimize(false);
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
        marker.scale.x = 0.03;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;

        edge_pub.publish(marker);
        poses_pub.publish(pose_array);
        for (int i=0; i<frame_num; i++)
        {
            auto pose = pose_array.poses.at(i);
            MenuHandler menu_handler;
            markerhandler::makeMenuMarker("marker" + std::to_string(i), pose, frame_id); // make menu marker and add to server

            setMarkerColorByIndex();

            // ROS_INFO("done");
            menu_handler = markerhandler::initMenu(pose);
            menu_handler.apply(*marker_server_, "marker" + std::to_string(i)); // apply menu entry to menu marker
        }
    }
    marker_server_->applyChanges();
}

tf::Vector3 getOriginFromFrameIndex(int index){
    pointcloud_tools::SensorDataID id = init_id;
    id.time_step = index;
    Pose::Ptr pose = (*graph_ptr).getVariable<Pose>(id, true);
    cv::Matx<Scalar, 3, 1> translation = (*pose).getData().getTranslation();
    tf::Vector3 origin = tf::Vector3(translation(0,0), translation(1,0), translation(2,0));
    return origin;
}

void publishArrowEdge(int index_ref, int index_in)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.type = visualization_msgs::Marker::ARROW;
    marker.ns = "edges";
    marker.id = 1;

    marker.points.clear();
    marker.points.push_back(geometry_msgs::Point());
    marker.points.push_back(geometry_msgs::Point());

    tf::Vector3 origin_ref = getOriginFromFrameIndex(index_ref);
    tf::Vector3 origin_in = getOriginFromFrameIndex(index_in);
    
    marker.points.at(0) = vector3toPoint(origin_ref);
    marker.points.at(1) = vector3toPoint(origin_in);

    marker.scale.x = 0.1;
    marker.scale.y = 0.2;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    arrow_edge_pub.publish(marker);
}

void publishEmptyArrowEdge()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.type = visualization_msgs::Marker::ARROW;
    marker.ns = "edges";
    marker.id = 1;

    marker.scale.x = 0.1;
    marker.scale.y = 0.2;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    arrow_edge_pub.publish(marker);
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

void addAbsFactorFromIMUPose(Graph &graph)
{
    pointcloud_tools::SensorDataID temp_id =init_id;
    pointcloud_tools::SensorFrameID sensor_id;
    sensor_id.frame_id = "nov_imu";
    sensor_id.vehicle = vehicle;
    Transform T;
    ParamMatrix H(ParamMatrix::eye());

    for (std::size_t i = 0; i < PARAM_DIM; i++)
        H(i, i) = 0.00001;  
    for(int i=0; i<frame_num; i++){
        temp_id.time_step = i;
        T = poseToTransform(IMU_pose_vec_.at(i));
        addAbsFactorWithSensor(graph, temp_id, sensor_id, T, H);
    }
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

    pointcloud_tools::SensorFrameID sensor_id;
    sensor_id.frame_id = init_id.sensor;
    sensor_id.vehicle = init_id.vehicle;

    ParamMatrix H;
    for (std::size_t i = 0; i < PARAM_DIM; i++)
        H(i, i) = 0.01;

    pointcloud_tools::SensorDataID ref_id = init_id, in_id = init_id;
    ref_id.time_step = frame_num1;
    in_id.time_step = frame_num2;

    if (add_or_remove == ADD)
        addRelativeFactorWithSensor(*graph_ptr, ref_id, in_id, sensor_id, sensor_id, T, H);
    else if (add_or_remove = REMOVE)
        removeRelativeFactor(*graph_ptr, ref_id, in_id, T, H);
}

Transform getTransformFromTimestep(int time_step){
    Transform T;
    pointcloud_tools::SensorDataID sensor_data_id;
    sensor_data_id = init_id;
    sensor_data_id.time_step = time_step;
    Pose::Ptr pose = (*graph_ptr).getVariable<Pose>(sensor_data_id, true);
    T.setTranslation((*pose).getData().getTranslation());
    T.setRotation((*pose).getData().getRotation());
    return T;
}

// Request Relative Pose To NDT_matching node
geometry_msgs::Pose requestNDTMatching(int source_time_step, int dest_time_step, bool directly_add_factor)
{
    pg_editor::GetNDTMatchingResult matching_result_service;
    Transform T_source, T_dest, T_lidar;
    pointcloud_tools::SensorDataID id_source, id_dest;
    pointcloud_tools::SensorFrameID id_lidar;

    id_lidar.frame_id = init_id.sensor;
    id_lidar.vehicle = init_id.vehicle;

    Pose::Ptr pose_lidar = (*graph_ptr).getSensorVariable(id_lidar, true);  

    T_source = getTransformFromTimestep(source_time_step);
    T_dest = getTransformFromTimestep(dest_time_step);

    T_lidar.setTranslation((*pose_lidar).getData().getTranslation());
    T_lidar.setRotation((*pose_lidar).getData().getRotation());

    matching_result_service.request.pointcloud1 = pointcloud_vec_.at(source_time_step);
    matching_result_service.request.pointcloud2 = pointcloud_vec_.at(dest_time_step);
    matching_result_service.request.time_step1 = source_time_step;
    matching_result_service.request.time_step2 = dest_time_step;
    matching_result_service.request.initial_pose = transformToPose(T_lidar.inv()*T_source.inv() * T_dest*T_lidar);

    if (matching_result_client.call(matching_result_service))
    {
        if(directly_add_factor){
            responseRelativeFactor(matching_result_service);
        }
    }
    // ROS_WARN("pc size print");
    // ROS_INFO("%d %d", matching_result_service.request.pointcloud1.data.size(), matching_result_service.request.pointcloud2.data.size());
    ROS_WARN("NDT result!!!");
    printPose(matching_result_service.response.result_pose);
    return matching_result_service.response.result_pose;
}

void visualize3Pointclouds()
{
    //ROS_WARN("ref in: %d %d", index_of_ref_pc, index_of_in_pc);

    sensor_msgs::PointCloud2 pointcloud_ref;
    sensor_msgs::PointCloud2 pointcloud_in;

    if(index_of_ref_pc!=INDEX_NOT_SET){
        pointcloud_ref = pointcloud_vec_.at(index_of_ref_pc);
    }
    if(index_of_in_pc!=INDEX_NOT_SET){
        pointcloud_in = pointcloud_vec_.at(index_of_in_pc);
    }

    pointcloud_ref.header.stamp = ros::Time::now();
    pointcloud_ref.header.frame_id = "sensor_frame_ref";
    pc_ref_pub.publish(pointcloud_ref);

    pointcloud_in.header.stamp = ros::Time::now();
    pointcloud_in.header.frame_id = "sensor_frame_in";
    pc_in_pub.publish(pointcloud_in);

    ros::spinOnce();
}

void publishResults(Graph &graph, tf::TransformBroadcaster &broadcaster, tf::TransformBroadcaster &sensor_broadcaster)
{
    visualization_msgs::Marker marker;
    geometry_msgs::PoseArray pose_array;
    std::vector<std::size_t> indices;
    (*graph_ptr).createMsgToVisualize(marker, pose_array, indices);

    pointcloud_tools::SensorFrameID sensor_id;
    if(is_single_lidar_imu_graph){
        sensor_id.frame_id = sensor_vec_.at(0);
    }
    sensor_id.vehicle = vehicle;
    auto sensor_var = (*graph_ptr).getSensorVariable(sensor_id, true);
    // if(*sensor_var == *Pose::Ptr())
    //     ROS_WARN("Can not find %s/%s sensor variable in graph",sensor_id.vehicle.c_str(), sensor_id.frame_id.c_str());
    auto sensor_T = sensor_var->getData();
    auto sensor_tf = poseToTfTransform(transformToPose(sensor_T));
    if(index_of_ref_pc!=-1&&index_of_in_pc!=-1){
        broadcaster.sendTransform(tf::StampedTransform(poseToTfTransform(pose_array.poses.at(index_of_ref_pc)), ros::Time::now(), "map", "frame_ref"));
        sensor_broadcaster.sendTransform(tf::StampedTransform(sensor_tf, ros::Time::now(), "frame_ref", "sensor_frame_ref"));

        broadcaster.sendTransform(tf::StampedTransform(poseToTfTransform(pose_array.poses.at(index_of_in_pc)), ros::Time::now(), "map", "frame_in"));
        sensor_broadcaster.sendTransform(tf::StampedTransform(sensor_tf, ros::Time::now(), "frame_in", "sensor_frame_in"));
            
        ros::spinOnce();
    }
    visualizeGraph((*graph_ptr), edge_pub, pose_pub, pose_pc_pub, "map");
    visualize3Pointclouds();
}
// TOSET

void saveImuPosesFromSrv(pg_editor::GetImuPoseResult &get_ECEF_pose_result_srv){
    ROS_WARN("ecef result size: %d", get_ECEF_pose_result_srv.response.imu_pose_array.poses.size());
    geometry_msgs::Pose pose0 = get_ECEF_pose_result_srv.response.imu_pose_array.poses.at(0);
    pg_lib::Transform transform0 = poseToTransform(get_ECEF_pose_result_srv.response.imu_pose_array.poses.at(0)), temp_transform;
    IMU_transform_vec_.clear();
    IMU_pose_vec_.clear();
    IMU_transform_vec_.push_back(pg_lib::Transform::eye());
    IMU_pose_vec_.push_back(transformToPose(pg_lib::Transform::eye()));
    for(int i=1; i<get_ECEF_pose_result_srv.response.imu_pose_array.poses.size(); i++){
        temp_transform = poseToTransform(get_ECEF_pose_result_srv.response.imu_pose_array.poses.at(i));
        IMU_transform_vec_.push_back(transform0.inv()*temp_transform); 
        IMU_pose_vec_.push_back(transformToPose(transform0.inv()*temp_transform));
    }
}

float getDistanceBetweenVariables(int num1, int num2){
    pointcloud_tools::SensorDataID temp_id1=init_id, temp_id2=init_id;
    temp_id1.time_step = num1;
    temp_id2.time_step = num2;

    Pose::Ptr pose_1 = (*graph_ptr).getVariable<Pose>(temp_id1);
    Pose::Ptr pose_2 = (*graph_ptr).getVariable<Pose>(temp_id2);
    
    cv::Matx<Scalar, 3, 1> translation1 = (*pose_1).getData().getTranslation();
    cv::Matx<Scalar, 3, 1> translation2 = (*pose_2).getData().getTranslation();
    cv::Matx<Scalar, 3, 1> translation_diff = translation1 - translation2;

    ROS_INFO("%f %f %f", translation1(0,0), translation1(1,0), translation1(2,0));
    ROS_INFO("%f %f %f", translation2(0,0), translation2(1,0), translation2(2,0));

    return sqrt(translation_diff.ddot(translation_diff));
}

void addEdges(){
    pointcloud_tools::SensorFrameID sensor_id;
    if(is_single_lidar_imu_graph){
        sensor_id.frame_id = sensor_vec_.at(0);
    }
    sensor_id.vehicle = vehicle;
    Transform T_init;
    ParamMatrix H(ParamMatrix::eye());
    for (std::size_t i = 0; i < PARAM_DIM; i++)
        H(i, i) = 0.00001;  
    for(int i=0; i<frame_num; i++){
        for(int j=i+1; j<frame_num; j++){
            if(getDistanceBetweenVariables(i, j)<edge_distance_threshold){
                ROS_WARN("%f", getDistanceBetweenVariables(i, j));
                ROS_WARN("%d %d", i,j);
                requestNDTMatching(i, j, true);
            }
        }
    }
}

bool isNumber(const std::string& s)
{
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it)) ++it;
    return !s.empty() && it == s.end();
}

bool isRemoveFactorOperation(){

}

bool isAddFactorOperation(pg_editor::InitialConfigurationConfig &config){
    return config.add_frame_num_in.compare("")!=0&&config.add_frame_num_ref.compare("")!=0&&(config.Add||config.Match);
}

bool isConfigurationSetDone(){
    return root_dirname_.compare("")!=0&&bag_time.compare("")!=0&&frame_num!=0;
}

void redefineRefToInTransformByNDT(geometry_msgs::Pose &new_pose){
    tf::TransformBroadcaster broadcaster;
    visualization_msgs::Marker marker;
    geometry_msgs::PoseArray pose_array;
    std::vector<std::size_t> indices;
    (*graph_ptr).createMsgToVisualize(marker, pose_array, indices);
    tf::Transform map_to_in_tf_transform;
    ROS_WARN("pose Results!");
    printPose(pose_array.poses.at(index_of_ref_pc));
    printPose(new_pose);
    map_to_in_tf_transform = poseToTfTransform(pose_array.poses.at(index_of_ref_pc))*poseToTfTransform(new_pose);
    broadcaster.sendTransform(tf::StampedTransform(map_to_in_tf_transform, ros::Time::now(), "map", "frame_in"));
    ros::spinOnce();
}

void configCallback(pg_editor::InitialConfigurationConfig &config, uint32_t level)
{
    ROS_INFO("config call back called");
    // For DEBUG
    root_dirname_ = config.root_dirname;
    bag_time = init_id.bag_time = config.bag_time;
    vehicle = init_id.vehicle = config.vehicle;
    max_iteration = config.max_iteration;
    edge_distance_threshold = config.edge_distance_threshold;

    if(config.load_single_lidar_imu_graph){
        sensor_num = 1;
        sensor_vec_.clear();
        sensor_vec_.push_back(config.lidar_sensor);
        ROS_WARN("new sensor is %s", sensor_vec_.at(0).c_str());
        init_id.sensor = config.lidar_sensor;
        is_single_lidar_imu_graph = true;
        config.load_single_lidar_imu_graph = false;
    }
    setFrameNum();
    if(!isConfigurationSetDone())
    {
        configuration_set_done = false;
        ROS_INFO("config set not done yet");
        return;
    }
    afterConfigurationSet();
    global_config_ = config;

    configuration_set_done = true;
    if(config.add_frame_num_ref.compare("")==0||!isNumber(config.add_frame_num_ref)||std::stoi(config.add_frame_num_ref)>=frame_num){
        index_of_ref_pc = -1;
    }
    else{
        index_of_ref_pc = std::stoi(config.add_frame_num_ref);
    }
    if(config.add_frame_num_in.compare("")==0||!isNumber(config.add_frame_num_in)||std::stoi(config.add_frame_num_in)>=frame_num){
        index_of_in_pc = -1;
    }
    else{
        index_of_in_pc = std::stoi(config.add_frame_num_in);
    }
    if(index_of_ref_pc==-1||index_of_in_pc==-1){
        ROS_WARN("Index Not Set Yet");
        config.Match = false;
        config.Add = false;
        return;
    }
    publishArrowEdge(index_of_ref_pc, index_of_in_pc);

    if(config.Match){
        ROS_INFO("match called %d %d", index_of_ref_pc, index_of_in_pc);
        geometry_msgs::Pose new_pose = requestNDTMatching(index_of_ref_pc, index_of_in_pc, false);
        redefineRefToInTransformByNDT(new_pose);
        visualize3Pointclouds();
        publishArrowEdge(index_of_ref_pc, index_of_in_pc);
        config.Match = false;
    }

    if(config.Add){
        ROS_INFO("add called %d %d", index_of_ref_pc, index_of_in_pc);
        requestNDTMatching(index_of_ref_pc, index_of_in_pc, true);
        publishEmptyArrowEdge();
        config.Add = false;
        config.add_frame_num_ref = "";
        config.add_frame_num_in = "";
        index_of_ref_pc = -1;
        index_of_in_pc = -1;
    }
    ROS_INFO("configure set again!!!");
}

void callTfBroadcaster(){
    pg_editor::TfBroadcastInfo Tf_broadcast_info_srv;
    if(is_single_lidar_imu_graph){
        Tf_broadcast_info_srv.request.sensor_name = sensor_vec_.at(0);
    }
    Tf_broadcast_info_client.call(Tf_broadcast_info_srv);
}

void afterConfigurationSet(){
    graph_ptr = new Graph;
    sendConfiguration();
    init::pointclouds();
    callTfBroadcaster();

    pg_editor::GetImuPoseResult get_ECEF_pose_result_srv;
    ECEF_pose_result_client.call(get_ECEF_pose_result_srv);
    saveImuPosesFromSrv(get_ECEF_pose_result_srv);
    addAbsFactorFromIMUPose((*graph_ptr));


    pointcloud_tools::SensorFrameID lidar_sensor_id;
    lidar_sensor_id.frame_id = init_id.sensor;
    lidar_sensor_id.vehicle = init_id.vehicle;
    ROS_INFO("new sensor name:%s", lidar_sensor_id.frame_id = init_id.sensor.c_str());
    (*graph_ptr).getSensorVariable(lidar_sensor_id, true);
    
    addEdges();

    //FOR DEBUG
    //true -> print sensor values
    //graph.optimize(true);
    (*graph_ptr).setMaxIteration(max_iteration);

    (*graph_ptr).optimize(false);

}

bool setFrameNum(){
    int cnt = 0;
    std::string path = root_dirname_+vehicle+"/"+bag_time+"/"+sensor_vec_.at(0)+"/";
    try{
        for (const auto & entry : std::experimental::filesystem::directory_iterator(path)){
            cnt++;
        }
    }
    catch(std::experimental::filesystem::filesystem_error& e){
        ROS_WARN("Wrong Path Inserted!!");
        return false;
    }
    frame_num = cnt;
    //ROS_WARN("frame number %d", frame_num);
    return true;
}

//send configuration to data_reader_server node
void sendConfiguration(){
    ROS_INFO("frame num: %d", frame_num);
    pg_editor::SendConfiguration send_configuration_srv;
    send_configuration_srv.request.root_dirname = root_dirname_;
    send_configuration_srv.request.bag_time = bag_time;
    send_configuration_srv.request.vehicle = vehicle;
    send_configuration_srv.request.sensor_num = sensor_num;
    send_configuration_srv.request.frame_num = frame_num;
    for(int i=0; i<sensor_num ;i++){
        send_configuration_srv.request.sensor_vec.push_back(sensor_vec_.at(i));
    }
    send_configuration_client.call(send_configuration_srv);
}

void forDebugSetConfiguration(){
    root_dirname_ = "/home/rideflux/v5_1_sample_data_1/";
    bag_time = init_id.bag_time = "2022-07-14-11-46-25";
    vehicle = init_id.vehicle = "solati_v5_1";
    max_iteration = 100;
    edge_distance_threshold = 10;
    sensor_num = 1;
    sensor_vec_.push_back("pandar64_0");
    init_id.sensor = "pandar64_0"; 
    is_single_lidar_imu_graph = true;
    setFrameNum();
}

float get3dDistance(tf::Vector3 vector1, tf::Vector3 vector2)
{
    return pow(pow(vector1.getX() - vector2.getX(), 2) + pow(vector1.getY() - vector2.getY(), 2) + pow(vector1.getZ() - vector2.getZ(), 2), 0.5);
}

bool getCalibrationCallback(pg_editor_panel::GetCalibration::Request &req, pg_editor_panel::GetCalibration::Response &res){
    Transform T_ref, T_in;
    pointcloud_tools::SensorFrameID id_ref, id_in;

    id_ref.frame_id = req.sensor_ref;
    id_ref.vehicle = vehicle;

    id_in.frame_id = req.sensor_in;
    id_in.vehicle = vehicle;    

    Pose::Ptr pose_ref = (*graph_ptr).getSensorVariable(id_ref);
    Pose::Ptr pose_in = (*graph_ptr).getSensorVariable(id_in);
    
    // if(pose_ref==nullptr||pose_in==nullptr){
    //     res.validate_sensor_name = false;
    // }

    T_ref.setTranslation((*pose_ref).getData().getTranslation());
    T_ref.setRotation((*pose_ref).getData().getRotation());

    T_in.setTranslation((*pose_in).getData().getTranslation());
    T_in.setRotation((*pose_in).getData().getRotation());

    geometry_msgs::Pose result_pose = transformToPose(T_ref.inv()*T_in);

    res.calibration_result_vec.push_back(result_pose.position.x);
    res.calibration_result_vec.push_back(result_pose.position.y);
    res.calibration_result_vec.push_back(result_pose.position.z);
    res.calibration_result_vec.push_back(result_pose.orientation.x);
    res.calibration_result_vec.push_back(result_pose.orientation.y);
    res.calibration_result_vec.push_back(result_pose.orientation.z);
    res.calibration_result_vec.push_back(result_pose.orientation.w);

    res.validate_sensor_name = true;
    return true;
}

void optimizeMsgCallback(const std_msgs::EmptyConstPtr &msg){
    (*graph_ptr).optimize(false);
}

void rClickCallback(const geometry_msgs::PointConstPtr &msg)
{
    ROS_INFO("clicked");
    int index = findClosestPoint(tf::Vector3(msg->x, msg->y, msg->z));
    ROS_INFO("closest index: %d", index);
    
    index_of_ref_pc = index;
    setMarkerColorByIndex();
    setRqtConfigByIndex();
}

void iClickCallback(const geometry_msgs::PointConstPtr &msg)
{
    ROS_INFO("clicked");
    int index = findClosestPoint(tf::Vector3(msg->x, msg->y, msg->z));
    ROS_INFO("closest index: %d", index);
    
    index_of_in_pc = index;
    setMarkerColorByIndex();
    setRqtConfigByIndex();
}

int findClosestPoint(tf::Vector3 new_vector)
{
    float min = INT_MAX, temp;
    int index;
    for (int i = 0; i < frame_num; i++)
    {
        temp = get3dDistance(new_vector, getOriginFromFrameIndex(i));
        if (temp < min)
        {
            min = temp;
            index = i;
        }
    }
    return index;
}

void setRqtConfigByIndex(){
    if(index_of_ref_pc == -1) {
        global_config_.add_frame_num_ref = "";
    }
    else{
        global_config_.add_frame_num_ref = std::to_string(index_of_ref_pc);
    }
    if(index_of_in_pc == -1) {
        global_config_.add_frame_num_in = "";
    }
    else{
        global_config_.add_frame_num_in = std::to_string(index_of_in_pc);
    }
    server_ptr_->updateConfig(global_config_);
}

//TODO
void setMarkerColorByIndex(){
    for(int i=0; i<frame_num; i++){
        visualization_msgs::InteractiveMarker temp_int_marker;
        if(!marker_server_->get("marker" + std::to_string(i), temp_int_marker)){
            return;
        }
        if(i==index_of_ref_pc){
            temp_int_marker.controls.at(0).markers.at(0).color.r = 0.5;
            temp_int_marker.controls.at(0).markers.at(0).color.g = 0;
            temp_int_marker.controls.at(0).markers.at(0).color.b = 0;
        }
        else if(i==index_of_in_pc){
            temp_int_marker.controls.at(0).markers.at(0).color.r = 0;
            temp_int_marker.controls.at(0).markers.at(0).color.g = 0;
            temp_int_marker.controls.at(0).markers.at(0).color.b = 0.5;
        }
        else{
            temp_int_marker.controls.at(0).markers.at(0).color.r = 0.5;
            temp_int_marker.controls.at(0).markers.at(0).color.g = 0.5;
            temp_int_marker.controls.at(0).markers.at(0).color.b = 0.5;    
        }
        marker_server_->erase("marker" + std::to_string(i));
        marker_server_->insert(temp_int_marker);
    }
    marker_server_->applyChanges();
}

void addMsgCallback(const std_msgs::EmptyConstPtr &msg)
{
    ROS_INFO("Add factor");
    if(index_of_ref_pc==INDEX_NOT_SET||index_of_in_pc==INDEX_NOT_SET){
        ROS_WARN("Index Not Set Yet");
        return;
    }
    add_or_remove = ADD;
    requestNDTMatching(index_of_ref_pc, index_of_in_pc, true);
    index_of_ref_pc = -1;
    index_of_in_pc = -1;
}

void removeMsgCallback(const std_msgs::EmptyConstPtr &msg)
{
    ROS_INFO("Remove factor");
    if(index_of_ref_pc==INDEX_NOT_SET||index_of_in_pc==INDEX_NOT_SET){
        ROS_WARN("Index Not Set Yet");
        return;
    }
    add_or_remove = REMOVE;
    requestNDTMatching(index_of_ref_pc, index_of_in_pc, true);
    index_of_ref_pc = -1;
    index_of_in_pc = -1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_server");
    ros::NodeHandle nh("~");
    //Graph graph;
    //graph_ptr = new Graph;
    pointcloud_client = nh.serviceClient<pg_editor::GetPointcloud>("/pc_read_service");

    edge_pub = nh.advertise<visualization_msgs::Marker>("/graph_edge", 1, true);
    pose_pub = nh.advertise<geometry_msgs::PoseArray>("/graph_pose", 1, true);
    pose_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/graph_pose_pc", 1, true);
    arrow_edge_pub = nh.advertise<visualization_msgs::Marker>("/edge_arrow", 1);

    Tf_broadcast_info_client = nh.serviceClient<pg_editor::TfBroadcastInfo>("/Tf_broadcast_info");
    matching_result_client = nh.serviceClient<pg_editor::GetNDTMatchingResult>("/matching_result");
    ECEF_pose_result_client = nh.serviceClient<pg_editor::GetImuPoseResult>("/ECEF_pose_result");
    send_configuration_client = nh.serviceClient<pg_editor::SendConfiguration>("/configuration");

    tf::TransformBroadcaster broadcaster;
    tf::TransformBroadcaster sensor_broadcaster;

    init::pointcloudPublishers(nh);

    marker_server_.reset(new InteractiveMarkerServer("main_server", "", false));
    dynamic_reconfigure::Server<pg_editor::InitialConfigurationConfig> server_;
    server_.setCallback(configCallback);
    server_ptr_ = &server_;

    //FOR DEBUG
    //configuration_set_done = true;
    while(!configuration_set_done){
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    //FOR DEBUG
    //forDebugSetConfiguration();

    ros::Subscriber r_click_subs = nh.subscribe("/rf_tool_r_click", 1, rClickCallback);
    ros::Subscriber i_click_subs = nh.subscribe("/rf_tool_i_click", 1, iClickCallback);

    ros::Subscriber add_msg_subs = nh.subscribe("/pg_editor_panel/add", 1, addMsgCallback);
    ros::Subscriber remove_msg_subs = nh.subscribe("/pg_editor_panel/remove", 1, removeMsgCallback);
    ros::Subscriber optimize_msg_subs = nh.subscribe("/pg_editor_panel/optimize", 1, optimizeMsgCallback);    

    ros::ServiceServer get_calibration_result_service = nh.advertiseService("/pg_editor_panel/get_calibration", getCalibrationCallback);

    while (ros::ok())
    {
        publishResults(*graph_ptr, broadcaster, sensor_broadcaster);
        ros::spinOnce();
        ros::Duration(0.3).sleep();
    }
    return 0;
}