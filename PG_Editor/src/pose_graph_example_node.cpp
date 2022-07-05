#include <ros/ros.h>
#include <pg_lib/graph.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>

using namespace pg_lib;

void visualizeGraph(const Graph &graph, ros::Publisher &edge_pub, ros::Publisher &poses_pub, ros::Publisher &pose_pc_pub)
{
    visualization_msgs::Marker marker;
    geometry_msgs::PoseArray pose_array;
    std::vector<std::size_t> indices;
    graph.createMsgToVisualize(marker, pose_array, indices);
    
    if (!indices.empty())
    {
        marker.header.frame_id = pose_array.header.frame_id = "map";
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

        sensor_msgs::PointCloud2 pose_pc_msg;
        pose_pc_msg.header.frame_id = "map";
        pose_pc_msg.header.stamp = ros::Time::now();
        pose_pc_msg.is_bigendian = false;
        pose_pc_msg.is_dense = false;
        pose_pc_msg.height = 1;
        pose_pc_msg.fields.resize(4);
        pose_pc_msg.fields[0].name = "x";
        pose_pc_msg.fields[0].offset = 0;
        pose_pc_msg.fields[0].count = 1;
        pose_pc_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        pose_pc_msg.fields[1].name = "y";
        pose_pc_msg.fields[1].offset = 4;
        pose_pc_msg.fields[1].count = 1;
        pose_pc_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        pose_pc_msg.fields[2].name = "z";
        pose_pc_msg.fields[2].offset = 8;
        pose_pc_msg.fields[2].count = 1;
        pose_pc_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        pose_pc_msg.fields[3].name = "index";
        pose_pc_msg.fields[3].offset = 12;
        pose_pc_msg.fields[3].count = 1;
        pose_pc_msg.fields[3].datatype = sensor_msgs::PointField::UINT32;
        pose_pc_msg.point_step = 16;

        // to-do : add id info of pc

        auto max_index = *std::max_element(indices.begin(), indices.end())+1;
        pose_pc_msg.width = max_index;
        pose_pc_msg.row_step = pose_pc_msg.width * pose_pc_msg.point_step;
        pose_pc_msg.data.resize(pose_pc_msg.row_step * pose_pc_msg.height);
        std::size_t n = 0;
        for (const auto &pose : pose_array.poses)
        {
        uint32_t index = indices[n];
        std::size_t offset = index * pose_pc_msg.point_step;
        float d;

        d = pose.position.x;
        std::memcpy(&pose_pc_msg.data[offset + pose_pc_msg.fields[0].offset], &d, sizeof(float));
        d = pose.position.y;
        std::memcpy(&pose_pc_msg.data[offset + pose_pc_msg.fields[1].offset], &d, sizeof(float));
        d = pose.position.z;
        std::memcpy(&pose_pc_msg.data[offset + pose_pc_msg.fields[2].offset], &d, sizeof(float));
        std::memcpy(&pose_pc_msg.data[offset + pose_pc_msg.fields[3].offset], &index, sizeof(uint32_t));
        ++n;
        }

        pose_pc_pub.publish(pose_pc_msg);
    }
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
    pointcloud_tools::SensorFrameID frame_id;
    frame_id.frame_id = id_ref.sensor;
    frame_id.vehicle = id_ref.vehicle;
    auto pose_ref = graph.getVariable<Pose>(id_ref, true);
    // auto sensor_ref = graph.getSensorVariable(frame_id, true);

    frame_id.frame_id = id_in.sensor;
    frame_id.vehicle = id_in.vehicle;
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

int main(int argc, char **argv){
    ros::init(argc, argv, "pose_graph_example_node");
    ros::NodeHandle nh("~");

    Graph graph;
        
    pointcloud_tools::SensorDataID id;
    id.bag_time = "2022-06-14-17-30-13";
    id.sensor = "pandar64_0";
    id.time_step = 0;
    id.vehicle = "solati_v5_1";

    Transform T(Transform::eye());
    ParamMatrix H(ParamMatrix::eye());
    for(std::size_t i=0; i<PARAM_DIM; i++)
        H(i,i) = 0.00001;

    if(!addAbsFactor(graph, id, T, H))
        return -1;

    // Add relative factor between pandar64_0 and pandar64_1 (-0.037123, 1.853627, 0.030135, -0.006315, -0.009281, 0.999918, -0.006111)
    pointcloud_tools::SensorDataID id_ref, id_in;
    id_ref = id_in = id;
    id_in.sensor = "pandar64_1";

    T(0,3) = -0.037123;
    T(1,3) = 1.853627; 
    T(2,3) = 0.030135;
    tf::Quaternion q1(-0.006315, -0.009281, 0.999918, -0.006111);
    const auto &R1 = tf::Matrix3x3(q1);
    for (std::size_t i=0; i<DIM; ++i)
    {
      for (std::size_t j=0; j<DIM; ++j)
        T(i, j) = R1[i][j];
    }

    for(std::size_t i=0; i<PARAM_DIM; i++)
        H(i,i) = 0.01;

    if(!addRelativeFactor(graph, id_ref,id_in, T, H))
        return -1;


    // Add relative factor between pandar64_0 and xt32_0 (-0.812523, -0.072326, -1.244487, 0.015393, 0.003190, -0.003695, 0.999870)
    id_ref = id;
    id_in.sensor = "xt32_0";

    T(0,3) = -0.812523;
    T(1,3) = -0.072326; 
    T(2,3) = -1.244487;
    tf::Quaternion q2(0.015393, 0.003190, -0.003695, 0.999870);
    const auto &R2 = tf::Matrix3x3(q2);
    for (std::size_t i=0; i<DIM; ++i)
    {
      for (std::size_t j=0; j<DIM; ++j)
        T(i, j) = R2[i][j];
    }

    for(std::size_t i=0; i<PARAM_DIM; i++)
        H(i,i) = 0.01;

    if(!addRelativeFactor(graph, id_ref,id_in, T, H))
        return -1;


    // Add relative factor between pandar64_1 and xt32_1 (0.824716, -0.038392, -1.199882, 0.002466, -0.000772, 0.035121, 0.999380)
    id_ref.sensor = "pandar64_1";
    id_in.sensor = "xt32_1";

    T(0,3) = 0.824716;
    T(1,3) = -0.038392; 
    T(2,3) = -1.199882;
    tf::Quaternion q3(0.002466, -0.000772, 0.035121, 0.999380);
    const auto &R3 = tf::Matrix3x3(q3);
    for (std::size_t i=0; i<DIM; ++i)
    {
      for (std::size_t j=0; j<DIM; ++j)
        T(i, j) = R3[i][j];
    }

    for(std::size_t i=0; i<PARAM_DIM; i++)
        H(i,i) = 0.01;

    if(!addRelativeFactor(graph, id_ref,id_in, T, H))
        return -1;


    // Add relative factor between xt32_1 and xt32_2 (-5.218188, 1.281358, -0.574805, -0.000000, 0.000846, 0.729292, -0.684202)
    id_ref.sensor = "xt32_1";
    id_in.sensor = "xt32_2";

    T(0,3) = -5.218188;
    T(1,3) = 1.281358; 
    T(2,3) = -0.574805;
    tf::Quaternion q4(-0.000000, 0.000846, 0.729292, -0.684202);
    const auto &R4 = tf::Matrix3x3(q4);
    for (std::size_t i=0; i<DIM; ++i)
    {
      for (std::size_t j=0; j<DIM; ++j)
        T(i, j) = R4[i][j];
    }

    for(std::size_t i=0; i<PARAM_DIM; i++)
        H(i,i) = 0.01;

    if(!addRelativeFactor(graph, id_ref,id_in, T, H))
        return -1;


    graph.optimize(false);        
    std::vector<Graph::DataID> var_indices;
    graph.getVariableIndices(var_indices);
    ROS_INFO("variable size: %d", var_indices.size());

    auto var = graph.getVariable<Pose>(0);
    ROS_INFO_STREAM("data:" << var->getData());
    ROS_INFO_STREAM("factor size: " << var->getNumFactors());

    auto factor = var->getFactor(0).lock();
    ROS_INFO("factor tpye: %d",factor->type());

    ros::Publisher edge_pub = nh.advertise<visualization_msgs::Marker>("graph_edge",1, true);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseArray>("graph_pose",1, true);
    ros::Publisher pose_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("graph_pose_pc",1, true);

    visualizeGraph(graph,edge_pub, pose_pub, pose_pc_pub);
    
    ros::spin();

    return 0;
}
