#include <ros/ros.h>
#include <pg_lib/graph.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <std_msgs/Int32MultiArray.h>


using namespace visualization_msgs;
using namespace interactive_markers;

using namespace pg_lib;

#define COST_TYPE Cost::TYPE::SQUARED

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

bool addAbsFactor(Graph &graph, pointcloud_tools::SensorDataID &id, pointcloud_tools::SensorFrameID &sensor_id, Transform &T, ParamMatrix &H)
{
    auto pose = graph.getVariable<Pose>(id, true);
    auto sensor = graph.getSensorVariable(sensor_id, true);
    
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

bool addRelativeFactor(Graph &graph, pointcloud_tools::SensorDataID &id_ref, pointcloud_tools::SensorDataID &id_in, pointcloud_tools::SensorFrameID &sensor_id_ref, pointcloud_tools::SensorFrameID &sensor_id_in, Transform &T, ParamMatrix &H)
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


int main(int argc, char **argv){
    ros::init(argc, argv, "pose_graph_example_node");
    ros::NodeHandle nh("~");

    Graph graph;
    graph.setMaxIteration(3);

    ros::Duration(1.0).sleep();

    // create 0th variable and absolute factor without sensor
    //7
    ROS_INFO("Add 0th abs factor");
    pointcloud_tools::SensorDataID id0;
    id0.vehicle = "solati_v5_1";
    id0.bag_time = "2022-07-14-11-46-25";
    id0.sensor = "pandar64_0";
    id0.time_step = 0;

    Transform T_abs_0 = Transform::eye();
    ParamMatrix H_abs_0;
    for(std::size_t i=0; i<PARAM_DIM; i++)
        H_abs_0(i,i) = 0.01;

    addAbsFactor(graph, id0, T_abs_0, H_abs_0); 


    // create 1th variable and absolute factor with sensor
    //8
    ROS_INFO("Add 1th abs factor");
    pointcloud_tools::SensorDataID id1;
    id1 = id0;
    id1.time_step = 1;

    Transform T_abs_1 = Transform::eye();
    for(std::size_t i=0; i<3; i++)
        T_abs_1(i,3) = 1.01;
    ParamMatrix H_abs_1 = H_abs_0;

    pointcloud_tools::SensorFrameID abs_sensor_id1;
    abs_sensor_id1.vehicle = id1.vehicle;
    abs_sensor_id1.frame_id = "nov_imu";
    addAbsFactor(graph, id1, abs_sensor_id1, T_abs_1, H_abs_1); 

    // create 2th variable and absolute factor with sensor
    //9
    ROS_INFO("Add 2th abs factor");
    pointcloud_tools::SensorDataID id2;
    id2 = id1;
    id2.time_step = 2;
    Transform T_abs_2 = Transform::eye();
    for(std::size_t i=0; i<3; i++)
        T_abs_1(i,3) = 2.02;
    ParamMatrix H_abs_2 = H_abs_1;

    pointcloud_tools::SensorFrameID abs_sensor_id2;
    abs_sensor_id2.vehicle = id2.vehicle;
    abs_sensor_id2.frame_id = "nov_imu";
    addAbsFactor(graph, id2, abs_sensor_id2, T_abs_2, H_abs_2); 

    // create relative 0th factor without sensor
    //10
    ROS_INFO("Add 0th rel factor");
    Transform T_rel_0 = Transform::eye();
    for(std::size_t i=0; i<3; i++)
        T_rel_0(i,3) = 0.99;
    ParamMatrix H_rel_0;
    for(std::size_t i=0; i<PARAM_DIM; i++)
        H_rel_0(i,i) = 0.1;
    addRelativeFactor(graph, id0, id1, T_rel_0, H_rel_0); 

    // create relative 1th factor with sensor
    //11
    ROS_INFO("Add 1th rel factor");
    pointcloud_tools::SensorFrameID rel_sensor_id1;
    rel_sensor_id1.vehicle = id1.vehicle;
    rel_sensor_id1.frame_id = "pandar64_0";
    pointcloud_tools::SensorFrameID rel_sensor_id2;
    rel_sensor_id2.vehicle = id2.vehicle;
    rel_sensor_id2.frame_id = "pandar64_0";

    Transform T_rel_1 = Transform::eye();
    for(std::size_t i=0; i<3; i++)
        T_rel_1(i,3) = 0.98;
    ParamMatrix H_rel_1;
    for(std::size_t i=0; i<PARAM_DIM; i++)
        H_rel_1(i,i) = 0.1;
    addRelativeFactor(graph, id1, id2, rel_sensor_id1, rel_sensor_id2, T_rel_1, H_rel_1); 

    // ROS_INFO("Optimization without sensor.");
    // graph.optimize();

    ROS_INFO("Optimization with sensor.");
    graph.optimize(true);

    ros::spin();

    return 0;
}