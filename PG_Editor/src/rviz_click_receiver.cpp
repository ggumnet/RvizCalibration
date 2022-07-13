#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pg_editor/TransformationInfo.h>

#include <geometry_msgs/PointStamped.h>


void click_handler_callback(const geometry_msgs::PointStampedConstPtr &msg){

    


}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "rviz_click_receiver");
    ros::NodeHandle nh;

    ros::Subscriber click_subs = nh.subscribe("/clicked_point", 1, click_handler_callback);

}
