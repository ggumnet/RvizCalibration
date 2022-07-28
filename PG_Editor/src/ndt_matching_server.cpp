#include <tf/transform_broadcaster.h>

#include <ros/ros.h>
#include <fstream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <rideflux_msgs/SensorDataID.h>

#include <pointcloud_tools/ndt_octree_gpu.h>
#include <pointcloud_tools/ndt_matcher_gpu.h>

#include <pg_editor/RelativeFramesInfo.h>
#include <pg_editor/RelativePoseInfo.h>
#include <pg_editor/GetMatchingResult.h>
#include <pg_editor/GetPointcloud.h>

#include <std_msgs/Int32MultiArray.h>

#include <rot2quat.h>
#include <types.h>
#include <transform_pose_conversion.h>
#include <print_tool.h>

std::string root_dirname_ = "/home/rideflux/Data/output/pc";

//global maps
std::map<std::pair<std::string, std::string>, Transform> id_id_transform_table;
std::map<std::string, sensor_msgs::PointCloud2> id_pointcloud_map;
std::map<std::string, Transform> id_default_transform_map;
std::vector<std::string> frame_id_list;
std::map<std::string, rideflux_msgs::SensorDataID> id_DataID_map;

rideflux_msgs::SensorDataID data_id1, data_id2, data_id3, data_id4, data_id5;
std::vector<std::string> field_names{"x","y","z","intensity"};
sensor_msgs::PointCloud2 pc_pandar0, pc_pandar1, pc_xt0, pc_xt1, pc_xt2;

ros::Subscriber relative_frame_sub;
ros::Publisher relative_pose_pub;


namespace initconfiguration{
  int frame_num;
  void initFrameIDs(){
    frame_id_list.insert(frame_id_list.end(), {"pandar64_0", "pandar64_1", "xt32_0", "xt32_1", "xt32_2"});
    frame_num = frame_id_list.size();
  }
}

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

//TODO
bool matching_result_callback(pg_editor::GetMatchingResult::Request& req, pg_editor::GetMatchingResult::Response& res){
    pg_editor::GetPointcloud pointcloud_service;
    sensor_msgs::PointCloud2 pointcloud1, pointcloud2;
    Transform T_init;

    ROS_INFO("matching result callback called");

    pointcloud1 = req.pointcloud1;
    pointcloud2 = req.pointcloud2;

    T_init = poseToTransform(req.initial_pose);
    
    Transform T_result = iterativeNdt(T_init, pointcloud1, pointcloud2, 0.4, 0.2);

    res.result_pose = transformToPose(T_result);

    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "ndt_matching_server");
    ros::NodeHandle nh("~");

    ROS_INFO("done");

    ros::ServiceServer matching_result_service = nh.advertiseService("/matching_result", matching_result_callback);

    initconfiguration::initFrameIDs();
    while(ros::ok()){
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    return 0;
}