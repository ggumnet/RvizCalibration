#include <ros/ros.h>
#include <pg_lib/graph.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "sensor_calib_node");
    ros::NodeHandle nh("~");

    pg_lib::Graph graph;
    
    std::string graph_file = "/home/gw/v3_1_cal_test/output/graph/JEJU114_220426.graph";
    
    graph.importGraphData(graph_file);
    
    std::vector<pg_lib::Graph::DataID> var_indices;
    graph.getVariableIndices(var_indices);
    ROS_INFO("variable size: %d", var_indices.size());

    auto var = graph.getVariable<pg_lib::Pose>(0);
    ROS_INFO_STREAM("data:" << var->getData());
    ROS_INFO_STREAM("factor size: " << var->getNumFactors());

    auto factor = var->getFactor(0).lock();
    ROS_INFO("factor tpye: %d",factor->type());
    
    


    

}
