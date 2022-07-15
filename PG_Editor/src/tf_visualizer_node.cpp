#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pg_editor/TransformationInfo.h>
#include <map>

const int N=5;
tf::Transform pgo_trforms[N];


void pgo_tf_handler_callback(const pg_editor::TransformationInfoConstPtr &msg)
{
    ROS_INFO("called");
    tf::Transform* trform;
    trform = &pgo_trforms[msg->frame_num];

    trform->setRotation(tf::Quaternion(msg->qx, msg->qy, msg->qz, msg->qw));
    trform->setOrigin(tf::Vector3(msg->tx, msg->ty, msg->tz));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_visualizer_node");
    ros::NodeHandle nh;

    tf::TransformBroadcaster broadcaster;

    ros::Subscriber pgo_xt32_0_subs = nh.subscribe("/pgo_xt32_0", 1, pgo_tf_handler_callback);
    ros::Subscriber pgo_xt32_1_subs = nh.subscribe("/pgo_xt32_1", 1, pgo_tf_handler_callback);
    ros::Subscriber pgo_xt32_2_subs = nh.subscribe("/pgo_xt32_2", 1, pgo_tf_handler_callback);
    ros::Subscriber pgo_pd0_subs = nh.subscribe("/pgo_pandar0", 1, pgo_tf_handler_callback);
    ros::Subscriber pgo_pd1_subs = nh.subscribe("/pgo_pandar1", 1, pgo_tf_handler_callback);


    while (ros::ok())
    {
        try
        {
            broadcaster.sendTransform(tf::StampedTransform(pgo_trforms[0], ros::Time::now(), "pgo_antenna", "pgo_pandar0"));
            broadcaster.sendTransform(tf::StampedTransform(pgo_trforms[1], ros::Time::now(), "pgo_antenna", "pgo_pandar1"));  
            broadcaster.sendTransform(tf::StampedTransform(pgo_trforms[2], ros::Time::now(), "pgo_antenna", "pgo_xt32_0"));
            broadcaster.sendTransform(tf::StampedTransform(pgo_trforms[3], ros::Time::now(), "pgo_antenna", "pgo_xt32_1"));
            broadcaster.sendTransform(tf::StampedTransform(pgo_trforms[4], ros::Time::now(), "pgo_antenna", "pgo_xt32_2"));

            ros::Duration(1.0).sleep();
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        ros::spinOnce();
    }
    ros::spin();
}
