#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pg_editor/TransformationInfo.h>

const int N=5;
tf::Transform trforms[N];
tf::Transform fixed_trforms[N];

void tf_handler_callback(const pg_editor::TransformationInfoConstPtr &msg)
{
    ROS_INFO("called\n");
    tf::Transform* trform;
    trform = &trforms[msg->frame_num];

    trform->setRotation(tf::Quaternion(msg->qx, msg->qy, msg->qz, msg->qw));
    trform->setOrigin(tf::Vector3(msg->tx, msg->ty, msg->tz));
}

void fixed_tf_handler_callback(const pg_editor::TransformationInfoConstPtr &msg)
{
    ROS_INFO("called\n");
    tf::Transform* trform;
    trform = &fixed_trforms[msg->frame_num];

    trform->setRotation(tf::Quaternion(msg->qx, msg->qy, msg->qz, msg->qw));
    trform->setOrigin(tf::Vector3(msg->tx, msg->ty, msg->tz));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_visualizer_node");
    ros::NodeHandle nh;

    tf::TransformBroadcaster broadcaster;

    ros::Subscriber xt32_0_subs = nh.subscribe("/xt32_0", 1, tf_handler_callback);
    ros::Subscriber xt32_1_subs = nh.subscribe("/xt32_1", 1, tf_handler_callback);
    ros::Subscriber xt32_2_subs = nh.subscribe("/xt32_2", 1, tf_handler_callback);
    ros::Subscriber pd0_subs = nh.subscribe("/pandar0", 1, tf_handler_callback);
    ros::Subscriber pd1_subs = nh.subscribe("/pandar1", 1, tf_handler_callback);

    ros::Subscriber fixed_xt32_0_subs = nh.subscribe("/fixed_xt32_0", 1, fixed_tf_handler_callback);
    ros::Subscriber fixed_xt32_1_subs = nh.subscribe("/fixed_xt32_1", 1, fixed_tf_handler_callback);
    ros::Subscriber fixed_xt32_2_subs = nh.subscribe("/fixed_xt32_2", 1, fixed_tf_handler_callback);
    ros::Subscriber fixed_pd0_subs = nh.subscribe("/fixed_pandar0", 1, fixed_tf_handler_callback);
    ros::Subscriber fixed_pd1_subs = nh.subscribe("/fixed_pandar1", 1, fixed_tf_handler_callback);


    while (ros::ok())
    {
        try
        {
            broadcaster.sendTransform(tf::StampedTransform(trforms[0], ros::Time::now(), "antenna", "xt32_0"));
            broadcaster.sendTransform(tf::StampedTransform(trforms[1], ros::Time::now(), "antenna", "xt32_1"));
            broadcaster.sendTransform(tf::StampedTransform(trforms[2], ros::Time::now(), "antenna", "xt32_2"));
            broadcaster.sendTransform(tf::StampedTransform(trforms[3], ros::Time::now(), "antenna", "pandar0"));
            broadcaster.sendTransform(tf::StampedTransform(trforms[4], ros::Time::now(), "antenna", "pandar1"));            
            

            // broadcaster.sendTransform(tf::StampedTransform(fixed_trforms[0], ros::Time::now(), "fixed_antenna", "fixed_xt32_0"));
            // broadcaster.sendTransform(tf::StampedTransform(fixed_trforms[1], ros::Time::now(), "fixed_antenna", "fixed_xt32_1"));
            // broadcaster.sendTransform(tf::StampedTransform(fixed_trforms[2], ros::Time::now(), "fixed_antenna", "fixed_xt32_2"));
            // broadcaster.sendTransform(tf::StampedTransform(fixed_trforms[3], ros::Time::now(), "fixed_antenna", "fixed_pandar0"));
            // broadcaster.sendTransform(tf::StampedTransform(fixed_trforms[4], ros::Time::now(), "fixed_antenna", "fixed_pandar1"));

            broadcaster.sendTransform(tf::StampedTransform(fixed_trforms[0], ros::Time::now(), "fixed_pandar0", "fixed_xt32_0"));
            broadcaster.sendTransform(tf::StampedTransform(fixed_trforms[1], ros::Time::now(), "fixed_pandar1", "fixed_xt32_1"));
            broadcaster.sendTransform(tf::StampedTransform(fixed_trforms[2], ros::Time::now(), "fixed_xt32_1", "fixed_xt32_2"));
            broadcaster.sendTransform(tf::StampedTransform(fixed_trforms[3], ros::Time::now(), "fixed_antenna", "fixed_pandar0"));
            broadcaster.sendTransform(tf::StampedTransform(fixed_trforms[4], ros::Time::now(), "fixed_pandar0", "fixed_pandar1"));

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
