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
#include <pg_editor/GetNDTMatchingResult.h>
#include <pg_editor/GetPointcloud.h>

#include <std_msgs/Int32MultiArray.h>

#include <rot2quat.h>
#include <types.h>
#include <transform_pose_conversion.h>
#include <print_tool.h>

namespace initconfiguration{
  int frame_num = 12;
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

//TODO
bool matching_result_callback(pg_editor::GetNDTMatchingResult::Request& req, pg_editor::GetNDTMatchingResult::Response& res){
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

    while(ros::ok()){
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    return 0;
}