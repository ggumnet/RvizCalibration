#include <tf/transform_broadcaster.h>

#include <ros/ros.h>
#include <fstream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <rideflux_msgs/SensorDataID.h>

#include <pointcloud_tools/ndt_octree_gpu.h>
#include <pointcloud_tools/ndt_matcher_gpu.h>

#include <gihyun_custom/rot2quat.h>

#include <pg_editor/TransformationInfo.h>
#include <pg_editor/RelativeFramesInfo.h>
#include <pg_editor/RelativePoseInfo.h>
#include <pg_editor/GetMatchingResult.h>
#include <pg_editor/GetPointcloud.h>


#include <std_msgs/Int32MultiArray.h>


using pointcloud_tools::NDTOctreeGPU;
using NDTMatcherGPU = pointcloud_tools::NDTMatcherGPU<NDTOctreeGPU>;
constexpr std::size_t DIM = NDTOctreeGPU::DIM;
constexpr std::size_t PARAM_DIM = NDTMatcherGPU::PARAM_DIM;
using Scalar = typename NDTOctreeGPU::Scalar;
using Vector = typename NDTOctreeGPU::Vector;
using Matrix = typename NDTOctreeGPU::Matrix;
using VectorSet = typename NDTOctreeGPU::VectorSet;
using Transform = typename NDTOctreeGPU::Transform;
using ParamMatrix = typename NDTMatcherGPU::ParamMatrix;
using ParamVector = typename NDTMatcherGPU::ParamVector;
using IndexScalar = typename NDTOctreeGPU::IndexScalar;

std::string root_dirname_ = "/home/rideflux/Data/output/pc";


//global maps
std::map<std::pair<std::string, std::string>, Transform> id_id_transform_table;
std::map<std::string, sensor_msgs::PointCloud2> id_pointcloud_map;
std::map<std::string, Transform> id_default_transform_map;
std::vector<std::string> frame_name_list;
std::map<std::string, rideflux_msgs::SensorDataID> id_DataID_map;

rideflux_msgs::SensorDataID data_id1, data_id2, data_id3, data_id4, data_id5;
std::vector<std::string> field_names{"x","y","z","intensity"};
sensor_msgs::PointCloud2 pc_pandar0, pc_pandar1, pc_xt0, pc_xt1, pc_xt2;
Transform T_ant_to_xt0, T_ant_to_xt1, T_ant_to_xt2, T_ant_to_pd0, T_ant_to_pd1;

ros::Subscriber relative_frame_sub;
ros::Publisher relative_pose_pub;

const int N = 5;
bool pc_publish_or_not [N] = {true, true, true, true, true};


struct MatchingOptions
{
  Vector cell_size;
  NDTMatcherGPU::OPTIM_METHOD optim_method;
  bool use_new_cost;
  std::vector<IndexScalar> max_iter_set;

  MatchingOptions() : cell_size(Vector(0.4, 0.4, 0.4)), optim_method(NDTMatcherGPU::OPTIM_METHOD::NEWTON_TRUSTREGION), use_new_cost(true), max_iter_set({30, 30, 30, 30})
  {
  }
  MatchingOptions(float cell_size) : cell_size(Vector(cell_size, cell_size, cell_size)), optim_method(NDTMatcherGPU::OPTIM_METHOD::NEWTON_TRUSTREGION), use_new_cost(true), max_iter_set({30, 30, 30, 30}){
  }
};

Transform matchTwoPCs(sensor_msgs::PointCloud2 &pc1, sensor_msgs::PointCloud2 &pc2, const MatchingOptions &option, Transform &T_init)
{
  NDTOctreeGPU::Ptr ndt1 = std::make_shared<NDTOctreeGPU>(option.cell_size);
  ndt1->addPointcloudMsg(pc1);
  ndt1->setNDTDataFromPoints();
  ndt1->computeNDsAtAllLevels();

  NDTOctreeGPU::Ptr ndt2 = std::make_shared<NDTOctreeGPU>(option.cell_size);
  ndt2->addPointcloudMsg(pc2);
  ndt2->setNDTDataFromPoints();
  ndt2->computeNDsAtAllLevels();

  NDTMatcherGPU matcher;
  matcher.setNDTRef(ndt1);
  matcher.setNDTIn(ndt2);

  matcher.setUseNewCost(option.use_new_cost);

  ParamMatrix C;
  bool is_converged;
  Transform T_est1;
  for (std::size_t n=0; n<option.max_iter_set.size()-0; ++n)
  {
    matcher.setMaxIteration(option.max_iter_set[n]);
    matcher.setOptimMethod(option.optim_method);
    matcher.setMatchingLevel(option.max_iter_set.size()-1-n);

    T_est1 = matcher.match(T_init);

    C = matcher.getPoseCovariance();
    is_converged = matcher.isConverged();
  }
  
  return T_est1;
}


Transform iterativeNdt(Transform T_init, sensor_msgs::PointCloud2 pc_ref, sensor_msgs::PointCloud2 pc_est, float cell_size1, float cell_size2){
    MatchingOptions option;
    option = MatchingOptions(cell_size1);
    auto T_est = matchTwoPCs(pc_ref, pc_est, option, T_init); 
    option = MatchingOptions(cell_size2);
    T_est = matchTwoPCs(pc_ref, pc_est, option, T_est);
    return T_est;
}

void printTransform(Transform transform){
    rf_geometry::SO<double, 3UL> rotation = transform.getRotation();
    cv::Matx<double, 3UL, 3UL> cv_rotation_matrix = (cv::Matx<double, 3UL, 3UL>)rotation;
    cv::Vec<double, 3UL> cv_translation_vector = transform.getTranslation();
    tf::Quaternion quaternion = mRot2Quat(cv_rotation_matrix);
    ROS_INFO("%f, %f, %f, %f, %f, %f, %f", cv_translation_vector[0], cv_translation_vector[1], cv_translation_vector[2], quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getW());  
}



void set_T_ant_to_frames(){
    T_ant_to_xt0.setRotation(cv::Matx<double, 3UL, 3UL>(-1,0,0,0,-1,0,0,0,1));
    T_ant_to_xt0.setTranslation(cv::Matx31d(4.517, 1.022, -1.589));

    T_ant_to_xt1.setRotation(cv::Matx<double, 3UL, 3UL>(1,0,0,0,1,0,0,0,1));
    T_ant_to_xt1.setTranslation(cv::Matx31d(4.517, -1.042, -1.589));

    T_ant_to_xt2.setRotation(cv::Matx<double, 3UL, 3UL>(0,1,0,-1,0,0,0,0,1));
    T_ant_to_xt2.setTranslation(cv::Matx31d( -0.631, 0, -2.249));

    T_ant_to_pd0.setRotation(cv::Matx<double, 3UL, 3UL>(-1,0,0,0,-1,0,0,0,1));
    T_ant_to_pd0.setTranslation(cv::Matx31d(3.672, 0.930, -0.369));
    
    T_ant_to_pd1.setRotation(cv::Matx<double, 3UL, 3UL>(1,0,0,0,1,0,0,0,1));
    T_ant_to_pd1.setTranslation(cv::Matx31d(3.672, -0.925, -0.369));
}

geometry_msgs::Pose transformToPose(Transform transform){

    rf_geometry::SO<double, 3UL> rotation = transform.getRotation();
    cv::Matx<double, 3UL, 3UL> cv_rotation_matrix = (cv::Matx<double, 3UL, 3UL>)rotation;;
    cv::Vec<double, 3UL> cv_translation_vector = transform.getTranslation();
    tf::Quaternion quaternion = mRot2Quat(cv_rotation_matrix);
    
    geometry_msgs::Pose pose;

    pose.position.x = cv_translation_vector[0]; 
    pose.position.y = cv_translation_vector[1];
    pose.position.z = cv_translation_vector[2];
    pose.orientation.x = quaternion.getX();
    pose.orientation.y = quaternion.getY();
    pose.orientation.z = quaternion.getZ();
    pose.orientation.w = quaternion.getW();

    return pose;
}

void relativeFrameCallback(const pg_editor::RelativeFramesInfoConstPtr &msg){

    ROS_INFO("called");

    Transform T;

    pg_editor::RelativePoseInfo relative_pose;

    relative_pose.source_frame = msg->source_frame;
    relative_pose.dest_frame = msg->dest_frame;

    //find matching result from table

    T = id_id_transform_table[std::make_pair(msg->source_frame, msg->dest_frame)];

    ROS_INFO("%s %s", msg->source_frame.c_str(), msg->dest_frame.c_str());
    printTransform(T);

    relative_pose.pose = transformToPose(T);

    relative_pose_pub.publish(relative_pose);
    ros::Duration(0.1).sleep();
}

void indexArrayCallback(const std_msgs::Int32MultiArray::ConstPtr& msg){
    for(int i=0; i<N; i++){
        pc_publish_or_not[i] = msg->data.at(i)==1;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "ndt_matching_server");
    ros::NodeHandle nh("~");

    relative_pose_pub = nh.advertise<pg_editor::RelativePoseInfo>("/relative_pose", 10);
    relative_frame_sub = nh.subscribe<pg_editor::RelativeFramesInfo>("/relative_frame", 10, relativeFrameCallback);


    ros::Publisher pc_pd0_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_pandar0", 1);
    ros::Publisher pc_pd1_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_pandar1", 1);
    ros::Publisher pc_xt0_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_xt0", 1);
    ros::Publisher pc_xt1_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_xt1", 1);
    ros::Publisher pc_xt2_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_xt2", 1);


    ros::Subscriber pc_viz_index_subs = nh.subscribe<std_msgs::Int32MultiArray>("/visualize_index_array", 1, indexArrayCallback);


    frame_name_list.push_back("pandar64_0");
    frame_name_list.push_back("pandar64_1");
    frame_name_list.push_back("xt32_0");
    frame_name_list.push_back("xt32_1");
    frame_name_list.push_back("xt32_2");


    readPointclouds();

    Transform T_tree_xt0, T_tree_xt1, T_tree_xt2, T_tree_pd0, T_tree_pd1;
    Transform T_xt0_to_pd1;

    initIdPointcloudMap();

    set_T_ant_to_frames();

    printTransform(T_ant_to_pd0);


    id_default_transform_map.insert(std::make_pair("pandar64_0", T_ant_to_pd0));
    id_default_transform_map.insert(std::make_pair("pandar64_1", T_ant_to_pd1));
    id_default_transform_map.insert(std::make_pair("xt32_0", T_ant_to_xt0));
    id_default_transform_map.insert(std::make_pair("xt32_1", T_ant_to_xt1));
    id_default_transform_map.insert(std::make_pair("xt32_2", T_ant_to_xt2));

    makeNdtTable();

    while(ros::ok()){
        pc_pandar0.header.stamp = ros::Time::now();
        pc_pandar1.header.stamp = ros::Time::now();
        pc_xt0.header.stamp = ros::Time::now();
        pc_xt1.header.stamp = ros::Time::now();
        pc_xt2.header.stamp = ros::Time::now();

        //ROS_INFO("%d %d %d %d %d", pc_publish_or_not[0], pc_publish_or_not[1], pc_publish_or_not[2], pc_publish_or_not[3], pc_publish_or_not[4]);

        if(pc_publish_or_not[0]) pc_pd0_pub.publish(pc_pandar0);
        if(pc_publish_or_not[1]) pc_pd1_pub.publish(pc_pandar1);
        if(pc_publish_or_not[2]) pc_xt0_pub.publish(pc_xt0);
        if(pc_publish_or_not[3]) pc_xt1_pub.publish(pc_xt1);
        if(pc_publish_or_not[4]) pc_xt2_pub.publish(pc_xt2);

        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    return 0;
}