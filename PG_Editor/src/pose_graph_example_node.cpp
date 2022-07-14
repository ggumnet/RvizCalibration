#include <ros/ros.h>
#include <pg_lib/graph.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <pg_editor/TransformationInfo.h>
#include <pg_editor/RelativeFramesInfo.h>
#include <pg_editor/RelativePose2FactorInfo.h>

#include <gihyun_custom/rot2quat.h>

using namespace visualization_msgs;
using namespace interactive_markers;

using namespace pg_lib;
boost::shared_ptr<InteractiveMarkerServer> server;


ros::Publisher first_pub;
ros::Publisher second_pub;

ros::Publisher relative_frame_pub;
ros::Subscriber relative_pose_sub;

//marker and edges publisher
ros::Publisher edge_pub;
ros::Publisher pose_pub;
ros::Publisher pose_pc_pub;

//Transformation information publisher
ros::Publisher pgo_xt32_0_pub; 
ros::Publisher pgo_xt32_1_pub; 
ros::Publisher pgo_xt32_2_pub; 
ros::Publisher pgo_pd0_pub;
ros::Publisher pgo_pd1_pub;

Graph graph;

pg_editor::TransformationInfo transforminfos_pgo_xt0, transforminfos_pgo_xt1, transforminfos_pgo_xt2, transforminfos_pgo_pd0, transforminfos_pgo_pd1;

std::map<std::string, pointcloud_tools::SensorDataID> id_to_sensorDataID_map;


//make marker at the certain position
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

//make box according to the size of InteractiveMarker
Marker makeBox(InteractiveMarker &msg)
{
    Marker marker;

    marker.type = Marker::CUBE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    return marker;
}

//make menu marker and push it to server
void makeMenuMarker(std::string name, geometry_msgs::Pose pose, std::string frame_id)
{
    InteractiveMarker int_marker = makeEmptyMarker(pose, frame_id);
    int_marker.name = name;

    InteractiveMarkerControl control;

    control.interaction_mode = InteractiveMarkerControl::BUTTON;
    control.always_visible = true;

    control.markers.push_back(makeBox(int_marker));
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->applyChanges();
}

geometry_msgs::Pose pose1, pose2;

void set_first(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    // visualization_msgs::Marker marker;
    // marker.header.frame_id = feedback.get()->header.frame_id;
    // marker.header.stamp = ros::Time();
    // marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.pose = feedback.get()->pose;
    // marker.scale.z = 1;
    // marker.text = "first";
    pose1 = feedback.get()->pose;
}

void set_second(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    Transform T1, T2;
    Transform T_1_to_2;
    geometry_msgs::Pose relative_pose;
    pose2 = feedback.get()->pose;
    // if(pose1!=NULL){
    //     T1.setRotation(tf::Matrix3x3(tf::Quaternion(pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w)));
    //     T1.setTranslation(cv::Matx31d(pose1.position.x, pose1.position.y, pose1.position.z));
    //     T2.setRotation(tf::Quaternion(pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w));
    //     T2.setTranslation(cv::Matx31d(pose2.position.x, pose2.position.y, pose2.position.z));
    //     T_1_to_2 = T1.inv()*T2;
    //     visualization_msgs::Marker menu_marker;
        
    // }    
}

MenuHandler initMenu(geometry_msgs::Pose pose)
{   
    MenuHandler menu_handler;
    MenuHandler::EntryHandle pose_menu, set_menu;
    pose_menu = menu_handler.insert("pose");
    menu_handler.insert(pose_menu, "tx "+std::to_string(pose.position.x));
    menu_handler.insert(pose_menu, "ty "+std::to_string(pose.position.y));
    menu_handler.insert(pose_menu, "tz "+std::to_string(pose.position.z));
    menu_handler.insert(pose_menu, "qx "+std::to_string(pose.orientation.x));
    menu_handler.insert(pose_menu, "qy "+std::to_string(pose.orientation.y));
    menu_handler.insert(pose_menu, "qz "+std::to_string(pose.orientation.z));
    menu_handler.insert(pose_menu, "qw "+std::to_string(pose.orientation.w));
    menu_handler.insert("first set", &set_first);
    menu_handler.insert("second set", &set_second); 
    return menu_handler;
}

void visualizeGraph(const Graph &graph, ros::Publisher &edge_pub, ros::Publisher &poses_pub, ros::Publisher &pose_pc_pub, std::string frame_id)
{
    visualization_msgs::Marker marker;
    geometry_msgs::PoseArray pose_array;
    std::vector<std::size_t> indices;
    graph.createMsgToVisualize(marker, pose_array, indices); 

    if(!indices.empty()){
        marker.header.frame_id = pose_array.header.frame_id = frame_id;
        marker.header.stamp = pose_array.header.stamp = ros::Time::now();

        ros::NodeHandle nh_priv("~");
        marker.id = 0;
        marker.ns = "edges";
        //marker.color.a = 1;
        //marker.color.r = 1;
        marker.scale.x = 0.2;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;

        edge_pub.publish(marker);
        poses_pub.publish(pose_array);
        int i=0;
        for(auto pose : pose_array.poses){
            MenuHandler menu_handler;
            i++;
            makeMenuMarker("marker"+std::to_string(i), pose, frame_id); //make menu marker and add to server
            ROS_INFO("done");
            menu_handler = initMenu(pose);
            menu_handler.apply(*server, "marker"+std::to_string(i)); //apply menu entry to menu marker
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

    if(!graph.addFactor(factor_abs))
    {
        ROS_ERROR("Failed to add factor_abs. %s", toString(id).c_str());
        return false;
    }
    return true;
}

bool addRelativeFactor(Graph &graph, pointcloud_tools::SensorDataID &id_ref, pointcloud_tools::SensorDataID &id_in, Transform &T, ParamMatrix &H)
{
    // pointcloud_tools::SensorFrameID frame_id;
    // frame_id.frame_id = id_ref.sensor;
    // frame_id.vehicle = id_ref.vehicle;
    auto pose_ref = graph.getVariable<Pose>(id_ref, true);
    // auto sensor_ref = graph.getSensorVariable(frame_id, true);

    // frame_id.frame_id = id_in.sensor;
    // frame_id.vehicle = id_in.vehicle;
    auto pose_in = graph.getVariable<Pose>(id_in, true);
    // auto sensor_in = graph.getSensorVariable(frame_id, true);
    
    Factor::Ptr factor = std::make_shared<RelativePose2Factor>(pose_ref, pose_in, T, H);
    factor->setIsReliable(true);

    if (!graph.addFactor(factor))
    {
        ROS_ERROR("Failed to add factor between %s and %s poses.", toString(id_ref).c_str(), toString(id_in).c_str());
        return false;
    }
    return true;
}

void requestRelativeFactor(std::string source_frame, std::string dest_frame){
    pg_editor::RelativeFramesInfo relative_frames_info;
    relative_frames_info.source_frame = source_frame;
    relative_frames_info.dest_frame = dest_frame;
    
    relative_frame_pub.publish(relative_frames_info);
}

//get relative pose, add it to the graph, optimize it and print it
void relative_pose_callback(const pg_editor::RelativePoseInfoConstPtr &msg)
{
    Transform T;
    T(0,3) = msg->pose.position.x;
    T(1,3) = msg->pose.position.y; 
    T(2,3) = msg->pose.position.z;
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    
    const auto &R = tf::Matrix3x3(q);
    for (std::size_t i=0; i<DIM; ++i)
    {
      for (std::size_t j=0; j<DIM; ++j)
        T(i, j) = R[i][j];
    }

    ParamMatrix H;
    for(std::size_t i=0; i<PARAM_DIM; i++)
        H(i,i) = 0.01;

    addRelativeFactor(graph, id_to_sensorDataID_map[msg->source_frame], id_to_sensorDataID_map[msg->dest_frame], T, H);
    //optimize
    graph.optimize(false);        

    //visualize
    std::vector<Graph::DataID> var_indices;
    graph.getVariableIndices(var_indices);
    ROS_INFO("variable size: %d", var_indices.size());

    auto var = graph.getVariable<Pose>(0);
    ROS_INFO_STREAM("data:" << var->getData());
    ROS_INFO_STREAM("factor size: " << var->getNumFactors());

    auto factor = var->getFactor(0).lock();
    ROS_INFO("factor type: %d",factor->type());

}

void publisher_init(){


}

pg_editor::TransformationInfo pose_to_transforminfo(geometry_msgs::Pose pose, int i){
    pg_editor::TransformationInfo transformationInfo;
    transformationInfo.frame_num = i;
    transformationInfo.qx = pose.orientation.x;
    transformationInfo.qy = pose.orientation.y;
    transformationInfo.qz = pose.orientation.z;
    transformationInfo.qw = pose.orientation.w;
    transformationInfo.tx = pose.position.x;
    transformationInfo.ty = pose.position.y;
    transformationInfo.tz = pose.position.z;
    transformationInfo.info_name = "pgo";
}

void publish_results(){
    visualization_msgs::Marker marker;
    geometry_msgs::PoseArray pose_array;
    std::vector<std::size_t> indices;
    graph.createMsgToVisualize(marker, pose_array, indices); 

    transforminfos_pgo_pd0 = pose_to_transforminfo(pose_array.poses.at(0), 0);
    transforminfos_pgo_pd1 = pose_to_transforminfo(pose_array.poses.at(1), 1);
    transforminfos_pgo_xt0 = pose_to_transforminfo(pose_array.poses.at(2), 2);
    transforminfos_pgo_xt1 = pose_to_transforminfo(pose_array.poses.at(3), 3);
    transforminfos_pgo_xt2 = pose_to_transforminfo(pose_array.poses.at(4), 4);

    pgo_xt32_0_pub.publish(transforminfos_pgo_xt0);
    pgo_xt32_1_pub.publish(transforminfos_pgo_xt1);
    pgo_xt32_2_pub.publish(transforminfos_pgo_xt2);
    pgo_pd0_pub.publish(transforminfos_pgo_pd0);
    pgo_pd1_pub.publish(transforminfos_pgo_pd1);
}

void init_sensorDataID(){
    pointcloud_tools::SensorDataID id;
    id.bag_time = "2022-06-14-17-30-13";
    id.sensor = "pandar64_0";
    id.time_step = 0;
    id.vehicle = "solati_v5_1";

    id.sensor = "pandar64_0";
    id_to_sensorDataID_map.insert(std::make_pair("pandar64_0",id));
    id.sensor = "pandar64_1";
    id_to_sensorDataID_map.insert(std::make_pair("pandar64_1",id));

    id.sensor = "xt32_0";
    id_to_sensorDataID_map.insert(std::make_pair("xt32_0",id));
    id.sensor = "xt32_1";
    id_to_sensorDataID_map.insert(std::make_pair("xt32_1",id));
    id.sensor = "xt32_2";
    id_to_sensorDataID_map.insert(std::make_pair("xt32_2",id));
}

int main(int argc, char **argv){
    ros::init(argc, argv, "pose_graph_example_node");
    ros::NodeHandle nh("~");

    init_sensorDataID();

    first_pub = nh.advertise<visualization_msgs::Marker>("/first_marker", 1);
    second_pub = nh.advertise<visualization_msgs::Marker>("/second_marker", 1);

    relative_frame_pub = nh.advertise<pg_editor::RelativeFramesInfo>("/relative_frame", 1);
    relative_pose_sub = nh.subscribe<pg_editor::RelativePoseInfo>("/relative_pose", 1, relative_pose_callback);


    edge_pub = nh.advertise<visualization_msgs::Marker>("graph_edge",1, true);
    pose_pub = nh.advertise<geometry_msgs::PoseArray>("graph_pose",1, true);
    pose_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("graph_pose_pc",1, true);


    pgo_xt32_0_pub = nh.advertise<pg_editor::TransformationInfo>("/pgo_xt32_0", 1);
    pgo_xt32_1_pub = nh.advertise<pg_editor::TransformationInfo>("/pgo_xt32_1", 1);
    pgo_xt32_2_pub = nh.advertise<pg_editor::TransformationInfo>("/pgo_xt32_2", 1);
    pgo_pd0_pub = nh.advertise<pg_editor::TransformationInfo>("/pgo_pandar0", 1);
    pgo_pd1_pub = nh.advertise<pg_editor::TransformationInfo>("/pgo_pandar1", 1);


    server.reset(new InteractiveMarkerServer("pose_graph_example_node", "", false));

    /* optimization start*/

    while(ros::ok()){
        publish_results();
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    
    // ros::spin();

    // while(ros::ok())
    //     ros::spinOnce();

    return 0;
}