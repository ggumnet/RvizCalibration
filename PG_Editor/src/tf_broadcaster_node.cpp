#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pg_editor/TfBroadcastInfo.h>


std::string child_frame_id;
bool Tf_broadcast_info_called = false;
tf2_ros::StaticTransformBroadcaster *broadcaster_ptr;


void sendTransform(){
    try
    {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.frame_id = "solati_v5_1/nov_imu";
        transformStamped.child_frame_id = "solati_v5_1/"+child_frame_id;
        transformStamped.header.stamp = ros::Time(0);
        transformStamped.transform.translation.x = 3.846;
        transformStamped.transform.translation.y = 0.93;
        transformStamped.transform.translation.z = 1.685;
        transformStamped.transform.rotation.x = 0;
        transformStamped.transform.rotation.y = 0;
        transformStamped.transform.rotation.z = 1;
        transformStamped.transform.rotation.w = 0;
        (*broadcaster_ptr).sendTransform(transformStamped);
        ros::Duration(0.01).sleep();
        ROS_WARN("send called");
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(0.1).sleep();
    }
} 

bool TfBroadcastInfoCallback(pg_editor::TfBroadcastInfo::Request &req, pg_editor::TfBroadcastInfo::Response &res){
    child_frame_id = req.sensor_name;
    Tf_broadcast_info_called = true;
    ROS_INFO("%s", req.sensor_name.c_str());
    sendTransform();
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_visualizer_node");
    ros::NodeHandle nh;

    //In broadcaster's constructor, nh is internally used
    tf2_ros::StaticTransformBroadcaster broadcaster;
    broadcaster_ptr = &broadcaster;

    ros::ServiceServer Tf_broadcast_info_service = nh.advertiseService("/Tf_broadcast_info", TfBroadcastInfoCallback);
    ros::spin();
}