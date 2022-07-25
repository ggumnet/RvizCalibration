#include <ros/ros.h>
#include <pg_lib/graph.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <pg_editor/TransformationInfo.h>
#include <pg_editor/RelativeFramesInfo.h>
#include <pg_editor/RelativePoseInfo.h>

#include <gihyun_custom/rot2quat.h>

#include <std_msgs/Int32MultiArray.h>

#define ADD 0
#define REMOVE 1

using namespace visualization_msgs;
using namespace interactive_markers;

using namespace pg_lib;
boost::shared_ptr<InteractiveMarkerServer> server;


const int N = 5;

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

std::vector<std::string> frame_id_list;

pg_editor::TransformationInfo transforminfos_pgo_xt0, transforminfos_pgo_xt1, transforminfos_pgo_xt2, transforminfos_pgo_pd0, transforminfos_pgo_pd1;

std::map<std::string, pointcloud_tools::SensorDataID> id_to_sensorDataID_map;

//std::shared_ptr<Graph> graph_ptr;
Graph* graph_ptr;

bool do_optimize = false;
int add_or_remove;


void printTransform(Transform transform){
    rf_geometry::SO<double, 3UL> rotation = transform.getRotation();
    cv::Matx<double, 3UL, 3UL> cv_rotation_matrix = (cv::Matx<double, 3UL, 3UL>)rotation;
    cv::Vec<double, 3UL> cv_translation_vector = transform.getTranslation();
    tf::Quaternion quaternion = mRot2Quat(cv_rotation_matrix);
    ROS_INFO("%f, %f, %f, %f, %f, %f, %f", cv_translation_vector[0], cv_translation_vector[1], cv_translation_vector[2], quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getW());  
}

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

void setFirst(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    // visualization_msgs::Marker marker;
    // marker.header.frame_id = feedback.get()->header.frame_id;
    // marker.header.stamp = ros::Time();
    // marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.pose = feedback.get()->pose;
    // marker.scale.z = 1;
    // marker.text = "first";
    pose1 = feedback.get()->pose;
    ROS_INFO("set first");
    return;
    ROS_INFO("set first: %d", feedback.get()->menu_entry_id);
}

void setSecond(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
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

void optimizeGraph(Graph& graph){

    //optimize
    (*graph_ptr).optimize(false);        

    //show optimization results
    /*
    std::vector<Graph::DataID> var_indices;
    (*graph_ptr).getVariableIndices(var_indices);
    ROS_INFO("variable size: %d", var_indices.size());

    auto var = (*graph_ptr).getVariable<Pose>(0);
    ROS_INFO_STREAM("data:" << var->getData());
    ROS_INFO_STREAM("factor size: " << var->getNumFactors());

    auto factor = var->getFactor(0).lock();
    ROS_INFO("factor type: %d",factor->type());*/
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
    //menu_handler.insert("Set First", &setFirst);
    //menu_handler.insert("Set Second", &setSecond); 
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
        //marker.type = visualization_msgs::Marker::ARROW;

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
            //ROS_INFO("done");
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


void removeRelativeFactor(Graph &graph, pointcloud_tools::SensorDataID &id_ref, pointcloud_tools::SensorDataID &id_in, Transform &T, ParamMatrix &H){

    auto pose_ref = graph.getVariable<Pose>(id_ref, true);
    auto pose_in = graph.getVariable<Pose>(id_in, true);
    
    Factor::Ptr factor = std::make_shared<RelativePose2Factor>(pose_ref, pose_in, T, H);
    factor->setIsReliable(true);

    graph.removeFactor(factor);
    return;
}

//OK
void requestRelativeFactor(std::string source_frame, std::string dest_frame){
    pg_editor::RelativeFramesInfo relative_frames_info;
    relative_frames_info.source_frame = source_frame;
    relative_frames_info.dest_frame = dest_frame;
    relative_frame_pub.publish(relative_frames_info);
    ros::Duration(0.3).sleep();
}

//get relative pose, add it to the graph, optimize it and print it
void relativePoseCallback(const pg_editor::RelativePoseInfoConstPtr &msg)
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

    ROS_INFO("%s %s", msg->source_frame.c_str(), msg->dest_frame.c_str());
    printTransform(T);

    ParamMatrix H;
    for(std::size_t i=0; i<PARAM_DIM; i++)
        H(i,i) = 0.01;

    //Graph graph = *graph_ptr;
    
    if(add_or_remove == ADD) addRelativeFactor(*graph_ptr, id_to_sensorDataID_map[msg->source_frame], id_to_sensorDataID_map[msg->dest_frame], T, H);
    else if(add_or_remove = REMOVE) removeRelativeFactor(*graph_ptr, id_to_sensorDataID_map[msg->source_frame], id_to_sensorDataID_map[msg->dest_frame], T, H);

    if(do_optimize){
        optimizeGraph(*graph_ptr);
    }
    //ROS_INFO("call back done");
}

void publisherInit(){


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

    return transformationInfo;
}

void publishResults(Graph &graph){
    visualization_msgs::Marker marker;
    geometry_msgs::PoseArray pose_array;
    std::vector<std::size_t> indices;
    graph.createMsgToVisualize(marker, pose_array, indices); 


    //ROS_INFO("pose array size: %d", pose_array.poses.size());    

    transforminfos_pgo_pd0 = pose_to_transforminfo(pose_array.poses.at(0), 0);
    transforminfos_pgo_pd1 = pose_to_transforminfo(pose_array.poses.at(1), 1);
    transforminfos_pgo_xt0 = pose_to_transforminfo(pose_array.poses.at(2), 2);
    transforminfos_pgo_xt1 = pose_to_transforminfo(pose_array.poses.at(3), 3);
    transforminfos_pgo_xt2 = pose_to_transforminfo(pose_array.poses.at(4), 4);


    pgo_pd0_pub.publish(transforminfos_pgo_pd0);
    pgo_pd1_pub.publish(transforminfos_pgo_pd1);
    pgo_xt32_0_pub.publish(transforminfos_pgo_xt0);
    pgo_xt32_1_pub.publish(transforminfos_pgo_xt1);
    pgo_xt32_2_pub.publish(transforminfos_pgo_xt2);



    visualizeGraph(graph, edge_pub, pose_pub, pose_pc_pub, "pgo_antenna");

    //ROS_INFO("done request");
}

void initGraph(Graph& graph){

    pointcloud_tools::SensorDataID id;
    id.bag_time = "2022-06-14-17-30-13";
    id.sensor = "pandar64_0";
    id.time_step = 0;
    id.vehicle = "solati_v5_1";

    Transform T(Transform::eye());
    ParamMatrix H(ParamMatrix::eye());
    for(std::size_t i=0; i<PARAM_DIM; i++)
        H(i,i) = 0.00001;

    addAbsFactor(graph, id, T, H);

    requestRelativeFactor("pandar64_0", "pandar64_1");
    requestRelativeFactor("pandar64_0", "xt32_0");
    requestRelativeFactor("pandar64_1", "xt32_1");
    requestRelativeFactor("xt32_1", "xt32_2");
}

void initSensorDataID(){
    pointcloud_tools::SensorDataID id;
    id.bag_time = "2022-06-14-17-30-13";
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

void addIndexArrayCallback(const std_msgs::Int32MultiArray::ConstPtr& msg){
    ROS_INFO("Add factor");
    //publish "/relative_frame"
    add_or_remove = ADD;
    requestRelativeFactor(frame_id_list.at(msg->data.at(0)), frame_id_list.at(msg->data.at(1)));
}

void removeIndexArrayCallback(const std_msgs::Int32MultiArray::ConstPtr& msg){
    ROS_INFO("Remove factor");
    //publish "/relative_frame"
    add_or_remove = REMOVE;
    requestRelativeFactor(frame_id_list.at(msg->data.at(0)), frame_id_list.at(msg->data.at(1)));
}



int main(int argc, char **argv){
    ros::init(argc, argv, "pose_graph_example_node");
    ros::NodeHandle nh("~");

    frame_id_list.push_back("pandar64_0");
    frame_id_list.push_back("pandar64_1");
    frame_id_list.push_back("xt32_0");
    frame_id_list.push_back("xt32_1");
    frame_id_list.push_back("xt32_2");

    Graph graph;

    //graph_ptr = std::make_shared<Graph>(graph);
    graph_ptr = &graph;

    first_pub = nh.advertise<visualization_msgs::Marker>("/first_marker", 1);
    second_pub = nh.advertise<visualization_msgs::Marker>("/second_marker", 1);

    relative_frame_pub = nh.advertise<pg_editor::RelativeFramesInfo>("/relative_frame", 10);
    relative_pose_sub = nh.subscribe<pg_editor::RelativePoseInfo>("/relative_pose", 10, relativePoseCallback);

    edge_pub = nh.advertise<visualization_msgs::Marker>("graph_edge",1, true);
    pose_pub = nh.advertise<geometry_msgs::PoseArray>("graph_pose",1, true);
    pose_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("graph_pose_pc",1, true);


    pgo_pd0_pub = nh.advertise<pg_editor::TransformationInfo>("/pgo_pandar64_0", 1);
    pgo_pd1_pub = nh.advertise<pg_editor::TransformationInfo>("/pgo_pandar64_1", 1);
    pgo_xt32_0_pub = nh.advertise<pg_editor::TransformationInfo>("/pgo_xt32_0", 1);
    pgo_xt32_1_pub = nh.advertise<pg_editor::TransformationInfo>("/pgo_xt32_1", 1);
    pgo_xt32_2_pub = nh.advertise<pg_editor::TransformationInfo>("/pgo_xt32_2", 1);


    ros::Subscriber add_index_subs = nh.subscribe<std_msgs::Int32MultiArray>("/add_edge_index_array", 10, addIndexArrayCallback);
    ros::Subscriber remove_index_subs = nh.subscribe<std_msgs::Int32MultiArray>("/remove_edge_index_array", 10, removeIndexArrayCallback);

    
    ros::Duration(3).sleep();

    initSensorDataID();

    initGraph(graph);

    ros::Duration(0.3).sleep();

    ros::spinOnce();

    optimizeGraph(graph);

    do_optimize = true;

    server.reset(new InteractiveMarkerServer("pose_graph_example_node", "", false));

    //ROS_INFO("first opti");

    /* optimization start*/

    while(ros::ok()){
        //ROS_INFO("loop");
        //ros::spin();
        publishResults(graph);
        ros::spinOnce();
        ros::Duration(0.3).sleep();
    }

    // while(ros::ok())
    //     ros::spinOnce();

    return 0;
}