// Contain main graph
// optimize graph and visualize it(publish pc and markers of graph)

#include <ros/ros.h>
#include <pg_lib/graph.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <pg_editor/TransformationInfo.h>
#include <pg_editor/RelativeFramesInfo.h>
#include <pg_editor/RelativePoseInfo.h>
#include <pg_editor/GetPointcloud.h>
#include <pg_editor/GetMatchingResult.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32MultiArray.h>

using namespace visualization_msgs;
using namespace interactive_markers;
using namespace pg_lib;

#include <rot2quat.h>
#include <transform_pose_conversion.h>
#include <print_tool.h>

#define ADD 0
#define REMOVE 1

pointcloud_tools::SensorDataID id;
boost::shared_ptr<InteractiveMarkerServer> server;

const int N = 5;
bool pc_publish_or_not[N] = {true, true, true, true, true};

ros::Publisher first_pub;
ros::Publisher second_pub;

ros::Publisher relative_frame_pub;
ros::Subscriber relative_pose_sub;

// marker and edges publisher
ros::Publisher edge_pub;
ros::Publisher pose_pub;
ros::Publisher pose_pc_pub;

ros::Publisher pc_pd0_pub;
ros::Publisher pc_pd1_pub;
ros::Publisher pc_xt0_pub;
ros::Publisher pc_xt1_pub;
ros::Publisher pc_xt2_pub;

std::vector<std::string> frame_id_list;
std::vector<sensor_msgs::PointCloud2> pointcloud_list;
std::vector<ros::Publisher *> pc_publisher_list;

pg_editor::TransformationInfo transforminfos_xt0, transforminfos_xt1, transforminfos_xt2, transforminfos_pd0, transforminfos_pd1;

std::map<std::string, pointcloud_tools::SensorDataID> id_to_sensorDataID_map;
std::map<std::string, sensor_msgs::PointCloud2> id_to_pointcloud_map;
std::map<std::string, Transform> id_to_init_transform_map;

ros::ServiceClient pointcloud_client;
ros::ServiceClient matching_result_client;

pg_editor::GetMatchingResult matching_result_service;

// std::shared_ptr<Graph> graph_ptr;
Graph *graph_ptr;

bool do_optimize = false;
int add_or_remove;

void initPointcloudIdAndFrameIds()
{
    id.vehicle = "solati_v5_1";
    id.bag_time = "2022-07-14-11-46-25";

    frame_id_list.insert(frame_id_list.end(), {"pandar64_0", "pandar64_1", "xt32_0", "xt32_1", "xt32_2"});
}

void initPointcloudPublisherList(){
    for(int i=0; i<frame_id_list.size(); i++){
        

    }
}

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
}

geometry_msgs::Pose pose1, pose2;

void setFirst(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    pose1 = feedback.get()->pose;
    ROS_INFO("set first");
    return;
    ROS_INFO("set first: %d", feedback.get()->menu_entry_id);
}

void setSecond(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    Transform T1, T2;
    Transform T_1_to_2;
    geometry_msgs::Pose relative_pose;
    pose2 = feedback.get()->pose;
}

// TODO
void optimizeGraph(Graph &graph)
{
    // optimize
    (*graph_ptr).optimize(false);
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

void visualizeGraph(const Graph &graph, ros::Publisher &edge_pub, ros::Publisher &poses_pub, ros::Publisher &pose_pc_pub, std::string frame_id)
{
    visualization_msgs::Marker marker;
    geometry_msgs::PoseArray pose_array;
    std::vector<std::size_t> indices;
    graph.createMsgToVisualize(marker, pose_array, indices);

    // ROS_INFO("%d", marker.points.size());

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
            menu_handler = initMenu(pose);
            menu_handler.apply(*server, "marker" + std::to_string(i)); // apply menu entry to menu marker
        }
    }
    server->applyChanges();
}

bool addAbsFactor(Graph &graph, pointcloud_tools::SensorDataID &id, Transform &T, ParamMatrix &H)
{
    pointcloud_tools::SensorFrameID frame_id;
    frame_id.frame_id = id.sensor;
    frame_id.vehicle = id.vehicle;
    auto pose = graph.getVariable<Pose>(id, true);
    // auto sensor = graph.getSensorVariable(frame_id, true);

    Factor::Ptr factor_abs = std::make_shared<AbsolutePoseFactor>(pose, T, H);
    factor_abs->setIsReliable(true);

    if (!graph.addFactor(factor_abs))
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

//get ndt result pose and add it to graph
void responseRelativeFactor(pg_editor::GetMatchingResult matching_result_service)
{
    Transform T;
    geometry_msgs::Pose pose = matching_result_service.response.result_pose;
    std::string frame1 = matching_result_service.request.pointcloud1.header.frame_id;
    std::string frame2 = matching_result_service.request.pointcloud2.header.frame_id;

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

    ROS_INFO("frames: %s %s", frame1.c_str(), frame2.c_str());
    printTransform(T);

    ParamMatrix H;
    for (std::size_t i = 0; i < PARAM_DIM; i++)
        H(i, i) = 0.01;

    if (add_or_remove == ADD)
        addRelativeFactor(*graph_ptr, id_to_sensorDataID_map[frame1], id_to_sensorDataID_map[frame2], T, H);
    else if (add_or_remove = REMOVE)
        removeRelativeFactor(*graph_ptr, id_to_sensorDataID_map[frame1], id_to_sensorDataID_map[frame2], T, H);

    if (do_optimize)
    {
        optimizeGraph(*graph_ptr);
    }
    // ROS_INFO("call back done");
}

// TODO
// Request Relative Pose To NDT_matching node
void requestRelativeFactor(std::string source_frame, std::string dest_frame)
{
    pg_editor::GetMatchingResult matching_result_service;
    Transform T_source, T_dest;

    T_source = id_to_init_transform_map[source_frame];
    T_dest = id_to_init_transform_map[dest_frame];

    matching_result_service.request.pointcloud1 = id_to_pointcloud_map[source_frame];
    matching_result_service.request.pointcloud2 = id_to_pointcloud_map[dest_frame];
    matching_result_service.request.initial_pose = transformToPose(T_source.inv() * T_dest);

    // OK

    if (matching_result_client.call(matching_result_service))
    {
        responseRelativeFactor(matching_result_service);
    }
}

void visualizePointclouds()
{
    for (int i = 0; i < N; i++)
    {
        if (pc_publish_or_not[i])
        {
            pointcloud_list.at(i).header.stamp = ros::Time::now();
            pc_publisher_list.at(i)->publish(pointcloud_list.at(i));
        }
    }
}

tf::Transform poseToTfTransform(geometry_msgs::Pose pose)
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
    return transform;
}
void publishTransformInfo()
{
}

void publishResults(Graph &graph, tf::TransformBroadcaster &broadcaster)
{
    visualization_msgs::Marker marker;
    geometry_msgs::PoseArray pose_array;
    std::vector<std::size_t> indices;
    graph.createMsgToVisualize(marker, pose_array, indices);

    for (int i = 0; i < N; i++)
    {
        broadcaster.sendTransform(tf::StampedTransform(poseToTfTransform(pose_array.poses.at(i)), ros::Time::now(), "antenna", frame_id_list.at(i)));
    }

    ros::spinOnce();

    visualizeGraph(graph, edge_pub, pose_pub, pose_pc_pub, "antenna");
    visualizePointclouds();
    publishTransformInfo();

    // ROS_INFO("done request");
}

void initPointclouds()
{
    pg_editor::GetPointcloud pointcloud_service;
    for (int i = 0; i < N; i++)
    {
        pointcloud_service.request.pointcloud_name = frame_id_list.at(i);
        if (pointcloud_client.call(pointcloud_service))
        {
            auto pc = pointcloud_service.response.pointcloud;
            ROS_INFO("frame_id, width: %s, %d", pc.header.frame_id.c_str(), pc.width);
            pointcloud_list.push_back(pc);
            id_to_pointcloud_map.insert(std::make_pair(pointcloud_service.request.pointcloud_name, pointcloud_service.response.pointcloud));
        }
        // ROS_INFO("response frame: %s", pointcloud_service.response.pointcloud.header.frame_id.c_str());
    }
}

void initGraph(Graph &graph)
{
    pointcloud_tools::SensorDataID temp_id = id;
    temp_id.sensor = "pandar64_0";
    temp_id.time_step = 0;

    Transform T(Transform::eye());
    ParamMatrix H(ParamMatrix::eye());
    for (std::size_t i = 0; i < PARAM_DIM; i++)
        H(i, i) = 0.00001;

    addAbsFactor(graph, temp_id, T, H);

    requestRelativeFactor("pandar64_0", "pandar64_1");
    requestRelativeFactor("pandar64_0", "xt32_0");
    requestRelativeFactor("pandar64_1", "xt32_1");
    requestRelativeFactor("xt32_1", "xt32_2");
}

void initSensorDataID()
{
    pointcloud_tools::SensorDataID temp_id = id;
    temp_id.sensor = "pandar64_0";
    id_to_sensorDataID_map.insert(std::make_pair("pandar64_0", temp_id));
    temp_id.sensor = "pandar64_1";
    id_to_sensorDataID_map.insert(std::make_pair("pandar64_1", temp_id));
    temp_id.sensor = "xt32_0";
    id_to_sensorDataID_map.insert(std::make_pair("xt32_0", temp_id));
    temp_id.sensor = "xt32_1";
    id_to_sensorDataID_map.insert(std::make_pair("xt32_1", temp_id));
    temp_id.sensor = "xt32_2";
    id_to_sensorDataID_map.insert(std::make_pair("xt32_2", temp_id));
}

void initTransformMap()
{
    Transform transform;
    transform.setRotation(cv::Matx<double, 3UL, 3UL>(-1, 0, 0, 0, -1, 0, 0, 0, 1));
    transform.setTranslation(cv::Matx31d(3.672, 0.930, -0.369));
    id_to_init_transform_map.insert(std::make_pair("pandar64_0", transform));

    transform.setRotation(cv::Matx<double, 3UL, 3UL>(1, 0, 0, 0, 1, 0, 0, 0, 1));
    transform.setTranslation(cv::Matx31d(3.672, -0.925, -0.369));
    id_to_init_transform_map.insert(std::make_pair("pandar64_1", transform));

    transform.setRotation(cv::Matx<double, 3UL, 3UL>(-1, 0, 0, 0, -1, 0, 0, 0, 1));
    transform.setTranslation(cv::Matx31d(4.517, 1.022, -1.589));
    id_to_init_transform_map.insert(std::make_pair("xt32_0", transform));

    transform.setRotation(cv::Matx<double, 3UL, 3UL>(1, 0, 0, 0, 1, 0, 0, 0, 1));
    transform.setTranslation(cv::Matx31d(4.517, -1.042, -1.589));
    id_to_init_transform_map.insert(std::make_pair("xt32_1", transform));

    transform.setRotation(cv::Matx<double, 3UL, 3UL>(0, 1, 0, -1, 0, 0, 0, 0, 1));
    transform.setTranslation(cv::Matx31d(-0.631, 0, -2.249));
    id_to_init_transform_map.insert(std::make_pair("xt32_2", transform));
}

void addIndexArrayCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    ROS_INFO("Add factor");
    // publish "/relative_frame"
    for (int i = 0; i < N; i++)
    {
        pc_publish_or_not[i] = true;
    }
    add_or_remove = ADD;
    requestRelativeFactor(frame_id_list.at(msg->data.at(0)), frame_id_list.at(msg->data.at(1)));
}

void removeIndexArrayCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    ROS_INFO("Remove factor");
    // publish "/relative_frame"
    for (int i = 0; i < N; i++)
    {
        pc_publish_or_not[i] = true;
    }
    add_or_remove = REMOVE;
    requestRelativeFactor(frame_id_list.at(msg->data.at(0)), frame_id_list.at(msg->data.at(1)));
}

void indexArrayCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    for (int i = 0; i < N; i++)
    {
        pc_publish_or_not[i] = msg->data.at(i) == 1;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_server");
    ros::NodeHandle nh("~");

    Graph graph;

    // graph_ptr = std::make_shared<Graph>(graph);
    graph_ptr = &graph;

    pointcloud_client = nh.serviceClient<pg_editor::GetPointcloud>("/pc_read_service");

    initPointcloudIdAndFrameIds();
    initPointclouds();
    initTransformMap();

    first_pub = nh.advertise<visualization_msgs::Marker>("/first_marker", 1);
    second_pub = nh.advertise<visualization_msgs::Marker>("/second_marker", 1);

    // relative_frame_pub = nh.advertise<pg_editor::RelativeFramesInfo>("/relative_frame", 10);
    // relative_pose_sub = nh.subscribe<pg_editor::RelativePoseInfo>("/relative_pose", 10, relativePoseCallback);

    edge_pub = nh.advertise<visualization_msgs::Marker>("graph_edge", 1, true);
    pose_pub = nh.advertise<geometry_msgs::PoseArray>("graph_pose", 1, true);
    pose_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("graph_pose_pc", 1, true);

    initPointcloudPublisherList();

    pc_pd0_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_pandar64_0", 1);
    pc_pd1_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_pandar64_1", 1);
    pc_xt0_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_xt32_0", 1);
    pc_xt1_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_xt32_1", 1);
    pc_xt2_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_xt32_2", 1);

    pc_publisher_list.push_back(&pc_pd0_pub);
    pc_publisher_list.push_back(&pc_pd1_pub);
    pc_publisher_list.push_back(&pc_xt0_pub);
    pc_publisher_list.push_back(&pc_xt1_pub);
    pc_publisher_list.push_back(&pc_xt2_pub);

    ros::Subscriber add_index_subs = nh.subscribe<std_msgs::Int32MultiArray>("/add_edge_index_array", 10, addIndexArrayCallback);
    ros::Subscriber remove_index_subs = nh.subscribe<std_msgs::Int32MultiArray>("/remove_edge_index_array", 10, removeIndexArrayCallback);

    matching_result_client = nh.serviceClient<pg_editor::GetMatchingResult>("/matching_result");

    initSensorDataID();
    initGraph(graph);

    ros::Duration(0.3).sleep();
    ros::spinOnce();

    do_optimize = true;
    optimizeGraph(graph);

    server.reset(new InteractiveMarkerServer("pose_graph_example_node", "", false));

    tf::TransformBroadcaster broadcaster;

    while (ros::ok())
    {
        publishResults(graph, broadcaster);
        ros::spinOnce();
        ros::Duration(0.3).sleep();
    }

    return 0;
}