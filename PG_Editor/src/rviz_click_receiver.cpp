#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pg_editor/TransformationInfo.h>

#include <geometry_msgs/PointStamped.h>


void click_handler_callback(const geometry_msgs::PointStampedConstPtr &msg){




}

void tf_handler_callback(const pg_editor::TransformationInfoConstPtr &msg){


    
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "rviz_click_receiver");
    ros::NodeHandle nh;

    ros::Subscriber click_subs = nh.subscribe("/clicked_point", 1, click_handler_callback);

    ros::Subscriber pgo_xt32_0_subs = nh.subscribe("/pgo_xt32_0", 1, tf_handler_callback);
    ros::Subscriber pgo_xt32_1_subs = nh.subscribe("/pgo_xt32_1", 1, tf_handler_callback);
    ros::Subscriber pgo_xt32_2_subs = nh.subscribe("/pgo_xt32_2", 1, tf_handler_callback);
    ros::Subscriber pgo_pd0_subs = nh.subscribe("/pgo_pandar0", 1, tf_handler_callback);
    ros::Subscriber pgo_pd1_subs = nh.subscribe("/pgo_pandar1", 1, tf_handler_callback);

}
