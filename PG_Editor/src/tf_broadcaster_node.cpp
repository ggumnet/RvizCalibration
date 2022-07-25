#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_visualizer_node");
    ros::NodeHandle nh;

    tf2_ros::StaticTransformBroadcaster broadcaster;

    try
    {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.frame_id = "solati_v5_1/nov_imu";
        transformStamped.child_frame_id = "solati_v5_1/pandar64_0";
        transformStamped.header.stamp = ros::Time(0);
        transformStamped.transform.translation.x = .2;
        transformStamped.transform.translation.y = .2;
        transformStamped.transform.translation.z = .2;
        transformStamped.transform.rotation.x = 0;
        transformStamped.transform.rotation.y = 0;
        transformStamped.transform.rotation.z = 0;
        transformStamped.transform.rotation.w = 1;
        
        broadcaster.sendTransform(transformStamped);

        ros::Duration(0.01).sleep();
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    
    ros::spin();
}
