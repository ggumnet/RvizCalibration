#include <ros/ros.h>
#include <pg_lib/graph.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>

#include <pg_editor/TransformationInfo.h>
#include <gihyun_custom/rot2quat.h>


using namespace pg_lib;

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
        pose_pc_msg.header.frame_id = frame_id;
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

int main(int argc, char **argv){
    ros::init(argc, argv, "pose_graph_example_node");
    ros::NodeHandle nh("~");

    Graph graph1, graph2;
        
    pointcloud_tools::SensorDataID id;
    id.bag_time = "2022-06-14-17-30-13";
    id.sensor = "pandar64_0";
    id.time_step = 0;
    id.vehicle = "solati_v5_1";

    Transform T(Transform::eye());
    ParamMatrix H(ParamMatrix::eye());
    for(std::size_t i=0; i<PARAM_DIM; i++)
        H(i,i) = 0.00001;

    if(!addAbsFactor(graph1, id, T, H))
        return -1;

    if(!addAbsFactor(graph2, id, T, H))
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

    if(!addRelativeFactor(graph1, id_ref,id_in, T, H))
        return -1;

    if(!addRelativeFactor(graph2, id_ref,id_in, T, H))
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

    if(!addRelativeFactor(graph1, id_ref,id_in, T, H))
        return -1;

    if(!addRelativeFactor(graph2, id_ref,id_in, T, H))
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

    if(!addRelativeFactor(graph1, id_ref,id_in, T, H))
        return -1;

    if(!addRelativeFactor(graph2, id_ref,id_in, T, H))
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

    if(!addRelativeFactor(graph1, id_ref,id_in, T, H))
        return -1;

    if(!addRelativeFactor(graph2, id_ref,id_in, T, H))
        return -1;  

    /* optimization start*/
    
    graph1.optimize(false);        
    std::vector<Graph::DataID> var_indices1;
    graph1.getVariableIndices(var_indices1);
    ROS_INFO("variable size: %d", var_indices1.size());

    auto var1 = graph1.getVariable<Pose>(0);
    ROS_INFO_STREAM("data:" << var1->getData());
    ROS_INFO_STREAM("factor size: " << var1->getNumFactors());

    auto factor1 = var1->getFactor(0).lock();
    ROS_INFO("factor type1: %d",factor1->type());

    ros::Publisher edge_pub1 = nh.advertise<visualization_msgs::Marker>("graph_edge1",1, true);
    ros::Publisher pose_pub1 = nh.advertise<geometry_msgs::PoseArray>("graph_pose1",1, true);
    ros::Publisher pose_pc_pub1 = nh.advertise<sensor_msgs::PointCloud2>("graph_pose_pc1",1, true);

    visualizeGraph(graph1,edge_pub1, pose_pub1, pose_pc_pub1, "fixed_antenna");

    visualization_msgs::Marker marker;
    geometry_msgs::PoseArray pose_array;
    std::vector<std::size_t> indices;
    graph1.createMsgToVisualize(marker, pose_array, indices);

    ROS_INFO("optimization 1 result");


    //T is from map frame to lidar frame
    Transform T_xt0, T_xt1, T_xt2, T_pd0, T_pd1;
    std::vector<Transform*> transform_list;
    transform_list.push_back(&T_pd0);
    transform_list.push_back(&T_pd1);
    transform_list.push_back(&T_xt0);
    transform_list.push_back(&T_xt1);
    transform_list.push_back(&T_xt2);


    for(int i=0; i<pose_array.poses.size(); i++){
        auto pose = pose_array.poses.at(i);
        transform_list.at(i)->setRotation(cv::Matx41d(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z));
        transform_list.at(i)->setTranslation(cv::Matx31d(pose.position.x, pose.position.y, pose.position.z));      
    }

    Transform T_pd0_to_pd1_1, T_pd0_to_xt0_1, T_pd1_to_xt1_1, T_xt1_to_xt2_1;
    Transform T_pd0_to_pd1_2, T_pd0_to_xt0_2, T_pd1_to_xt1_2, T_xt1_to_xt2_2;

    T_pd0_to_pd1_1 = T_pd0.inv()*T_pd1;
    T_pd0_to_xt0_1 = T_pd0.inv()*T_xt0;
    T_pd1_to_xt1_1 = T_pd1.inv()*T_xt1;
    T_xt1_to_xt2_1 = T_xt1.inv()*T_xt2;

    std::vector<Transform> transfrom_result_list1, transfrom_result_list2;

    transfrom_result_list1.push_back(T_pd0_to_pd1_1);
    transfrom_result_list1.push_back(T_pd0_to_xt0_1);
    transfrom_result_list1.push_back(T_pd1_to_xt1_1);
    transfrom_result_list1.push_back(T_xt1_to_xt2_1);

    std::vector<std::string> result_name_list;

    result_name_list.push_back("T_pd0_to_pd1");
    result_name_list.push_back("T_pd0_to_xt0");
    result_name_list.push_back("T_pd1_to_xt1");
    result_name_list.push_back("T_xt1_to_xt2");


    rf_geometry::SO<double, 3UL> rotation;
    cv::Matx<double, 3UL, 3UL> cv_rotation_matrix;
    cv::Vec<double, 3UL> cv_translation_vector;
    tf::Quaternion quaternion;

    
    //Add relative factor between pandar64_0 and pandar64_1 (-0.037123, 1.853627, 0.030135, -0.006315, -0.009281, 0.999918, -0.006111)

    id_ref = id_in = id;
    id_ref.sensor = "xt32_0";
    id_in.sensor = "pandar64_1";

    T(0,3) = 0.749554;
    T(1,3) = 1.963856; 
    T(2,3) = 1.273790;
    tf::Quaternion q5(-0.006208, 0.000000, 0.999929, -0.010215);
    const auto &R5 = tf::Matrix3x3(q5);
    for (std::size_t i=0; i<DIM; ++i)
    {
      for (std::size_t j=0; j<DIM; ++j)
        T(i, j) = R5[i][j];
    }

    for(std::size_t i=0; i<PARAM_DIM; i++)
        H(i,i) = 0.01;

    if(!addRelativeFactor(graph2, id_ref,id_in, T, H))
        return -1;


    /* optimization start*/


    graph2.optimize(false);        
    std::vector<Graph::DataID> var_indices;
    graph2.getVariableIndices(var_indices);
    ROS_INFO("variable size: %d", var_indices.size());

    auto var2 = graph2.getVariable<Pose>(0);
    ROS_INFO_STREAM("data:" << var2->getData());
    ROS_INFO_STREAM("factor size: " << var2->getNumFactors());

    auto factor2 = var2->getFactor(0).lock();
    ROS_INFO("factor type2: %d",factor2->type());

    ros::Publisher edge_pub2 = nh.advertise<visualization_msgs::Marker>("graph_edge2",1, true);
    ros::Publisher pose_pub2 = nh.advertise<geometry_msgs::PoseArray>("graph_pose2",1, true);
    ros::Publisher pose_pc_pub2 = nh.advertise<sensor_msgs::PointCloud2>("graph_pose_pc2",1, true);

    visualizeGraph(graph2, edge_pub2, pose_pub2, pose_pc_pub2, "pgo_antenna");


    graph2.createMsgToVisualize(marker, pose_array, indices);

    for(int i=0; i<pose_array.poses.size(); i++){
        auto pose = pose_array.poses.at(i);
        transform_list.at(i)->setRotation(cv::Matx41d(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z));
        transform_list.at(i)->setTranslation(cv::Matx31d(pose.position.x, pose.position.y, pose.position.z));      
    }

    T_pd0_to_pd1_2 = T_pd0.inv()*T_pd1;
    T_pd0_to_xt0_2 = T_pd0.inv()*T_xt0;
    T_pd1_to_xt1_2 = T_pd1.inv()*T_xt1;
    T_xt1_to_xt2_2 = T_xt1.inv()*T_xt2;


    transfrom_result_list2.push_back(T_pd0_to_pd1_2);
    transfrom_result_list2.push_back(T_pd0_to_xt0_2);
    transfrom_result_list2.push_back(T_pd1_to_xt1_2);
    transfrom_result_list2.push_back(T_xt1_to_xt2_2);

    

    std::vector<pg_editor::TransformationInfo*> pgo_tf_info_list;
    pg_editor::TransformationInfo transforminfos_pgo_xt0, transforminfos_pgo_xt1, transforminfos_pgo_xt2, transforminfos_pgo_pd0, transforminfos_pgo_pd1; 
    
    pgo_tf_info_list.push_back(&transforminfos_pgo_pd0);
    pgo_tf_info_list.push_back(&transforminfos_pgo_pd1);
    pgo_tf_info_list.push_back(&transforminfos_pgo_xt0);
    pgo_tf_info_list.push_back(&transforminfos_pgo_xt1);
    pgo_tf_info_list.push_back(&transforminfos_pgo_xt2);

    ROS_INFO("optimization 2 result");

    for(int i=0; i<pose_array.poses.size(); i++){
        auto pose = pose_array.poses.at(i);
        
        pgo_tf_info_list.at(i)->frame_num = i;
        pgo_tf_info_list.at(i)->qx = pose.orientation.x;
        pgo_tf_info_list.at(i)->qy = pose.orientation.y;
        pgo_tf_info_list.at(i)->qz = pose.orientation.z;
        pgo_tf_info_list.at(i)->qw = pose.orientation.w;
        pgo_tf_info_list.at(i)->tx = pose.position.x;
        pgo_tf_info_list.at(i)->ty = pose.position.y;
        pgo_tf_info_list.at(i)->tz = pose.position.z;
        
        ROS_INFO("%f, %f, %f, %f, %f, %f, %f", pgo_tf_info_list.at(i)->tx, pgo_tf_info_list.at(i)->ty, pgo_tf_info_list.at(i)->tz, pgo_tf_info_list.at(i)->qw, pgo_tf_info_list.at(i)->qx, pgo_tf_info_list.at(i)->qy, pgo_tf_info_list.at(i)->qz); 
    }


    //print relative transformation between Lidar frames
    ROS_INFO("Result 1 (txyz - qxyzw)");

    for(int i=0; i<transfrom_result_list1.size(); i++){    
        rotation = transfrom_result_list1.at(i).getRotation();
        cv_rotation_matrix = (cv::Matx<double, 3UL, 3UL>)rotation;
        cv_translation_vector = transfrom_result_list1.at(i).getTranslation();
        quaternion = mRot2Quat(cv_rotation_matrix);
        ROS_INFO("%s : %f, %f, %f, %f, %f, %f, %f", result_name_list.at(i).c_str(), cv_translation_vector[0], cv_translation_vector[1], cv_translation_vector[2], quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getW()); 
    }

    ROS_INFO("Result 2 (txyz - qxyzw)");

    for(int i=0; i<transfrom_result_list2.size(); i++){    
        rotation = transfrom_result_list2.at(i).getRotation();
        cv_rotation_matrix = (cv::Matx<double, 3UL, 3UL>)rotation;
        cv_translation_vector = transfrom_result_list2.at(i).getTranslation();
        quaternion = mRot2Quat(cv_rotation_matrix);
        ROS_INFO("%s : %f, %f, %f, %f, %f, %f, %f", result_name_list.at(i).c_str(), cv_translation_vector[0], cv_translation_vector[1], cv_translation_vector[2], quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getW()); 
    }


    ros::Publisher pgo_xt32_0_pub = nh.advertise<pg_editor::TransformationInfo>("/pgo_xt32_0", 1);
    ros::Publisher pgo_xt32_1_pub = nh.advertise<pg_editor::TransformationInfo>("/pgo_xt32_1", 1);
    ros::Publisher pgo_xt32_2_pub = nh.advertise<pg_editor::TransformationInfo>("/pgo_xt32_2", 1);
    ros::Publisher pgo_pd0_pub = nh.advertise<pg_editor::TransformationInfo>("/pgo_pandar0", 1);
    ros::Publisher pgo_pd1_pub = nh.advertise<pg_editor::TransformationInfo>("/pgo_pandar1", 1);

    while(ros::ok()){
        pgo_xt32_0_pub.publish(transforminfos_pgo_xt0);
        pgo_xt32_1_pub.publish(transforminfos_pgo_xt1);
        pgo_xt32_2_pub.publish(transforminfos_pgo_xt2);
        pgo_pd0_pub.publish(transforminfos_pgo_pd0);
        pgo_pd1_pub.publish(transforminfos_pgo_pd1);
        ros::Duration(0.1).sleep();
    }
    ros::spin();

    return 0;
}
