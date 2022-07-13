#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pg_editor/TransformationInfo.h>
#include <map>

const int N=5;
std::map<std::string, tf::Transform(*)[N]> TransformListMap;
tf::Transform raw_trforms[N];
tf::Transform tree_trforms[N];
tf::Transform pgo_trforms[N];


void initTransformListMap(){
    TransformListMap.insert(std::make_pair("raw_trforms", &raw_trforms));
    TransformListMap.insert(std::make_pair("tree_trforms", &tree_trforms));
    TransformListMap.insert(std::make_pair("pgo_trforms", &pgo_trforms));
}

void tf_handler_callback(const pg_editor::TransformationInfoConstPtr &msg)
{
    tf::Transform* trform;
    trform = &raw_trforms[msg->frame_num];

    trform->setRotation(tf::Quaternion(msg->qx, msg->qy, msg->qz, msg->qw));
    trform->setOrigin(tf::Vector3(msg->tx, msg->ty, msg->tz));
}

void tree_tf_handler_callback(const pg_editor::TransformationInfoConstPtr &msg)
{
    tf::Transform* trform;
    trform = &tree_trforms[msg->frame_num];

    trform->setRotation(tf::Quaternion(msg->qx, msg->qy, msg->qz, msg->qw));
    trform->setOrigin(tf::Vector3(msg->tx, msg->ty, msg->tz));
}

void pgo_tf_handler_callback(const pg_editor::TransformationInfoConstPtr &msg)
{
    tf::Transform* trform;
    trform = &pgo_trforms[msg->frame_num];

    trform->setRotation(tf::Quaternion(msg->qx, msg->qy, msg->qz, msg->qw));
    trform->setOrigin(tf::Vector3(msg->tx, msg->ty, msg->tz));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_visualizer_node");
    ros::NodeHandle nh;

    initTransformListMap();

    tf::TransformBroadcaster broadcaster;

    ros::Subscriber xt32_0_subs = nh.subscribe("/xt32_0", 1, tf_handler_callback);
    ros::Subscriber xt32_1_subs = nh.subscribe("/xt32_1", 1, tf_handler_callback);
    ros::Subscriber xt32_2_subs = nh.subscribe("/xt32_2", 1, tf_handler_callback);
    ros::Subscriber pd0_subs = nh.subscribe("/pandar0", 1, tf_handler_callback);
    ros::Subscriber pd1_subs = nh.subscribe("/pandar1", 1, tf_handler_callback);

    ros::Subscriber tree_xt32_0_subs = nh.subscribe("/tree_xt32_0", 1, tree_tf_handler_callback);
    ros::Subscriber tree_xt32_1_subs = nh.subscribe("/tree_xt32_1", 1, tree_tf_handler_callback);
    ros::Subscriber tree_xt32_2_subs = nh.subscribe("/tree_xt32_2", 1, tree_tf_handler_callback);
    ros::Subscriber tree_pd0_subs = nh.subscribe("/tree_pandar0", 1, tree_tf_handler_callback);
    ros::Subscriber tree_pd1_subs = nh.subscribe("/tree_pandar1", 1, tree_tf_handler_callback);

    ros::Subscriber pgo_xt32_0_subs = nh.subscribe("/pgo_xt32_0", 1, pgo_tf_handler_callback);
    ros::Subscriber pgo_xt32_1_subs = nh.subscribe("/pgo_xt32_1", 1, pgo_tf_handler_callback);
    ros::Subscriber pgo_xt32_2_subs = nh.subscribe("/pgo_xt32_2", 1, pgo_tf_handler_callback);
    ros::Subscriber pgo_pd0_subs = nh.subscribe("/pgo_pandar0", 1, pgo_tf_handler_callback);
    ros::Subscriber pgo_pd1_subs = nh.subscribe("/pgo_pandar1", 1, pgo_tf_handler_callback);


    while (ros::ok())
    {
        try
        {
            broadcaster.sendTransform(tf::StampedTransform(raw_trforms[0], ros::Time::now(), "antenna", "xt32_0"));
            broadcaster.sendTransform(tf::StampedTransform(raw_trforms[1], ros::Time::now(), "antenna", "xt32_1"));
            broadcaster.sendTransform(tf::StampedTransform(raw_trforms[2], ros::Time::now(), "antenna", "xt32_2"));
            broadcaster.sendTransform(tf::StampedTransform(raw_trforms[3], ros::Time::now(), "antenna", "pandar0"));
            broadcaster.sendTransform(tf::StampedTransform(raw_trforms[4], ros::Time::now(), "antenna", "pandar1"));            
            
            broadcaster.sendTransform(tf::StampedTransform(tree_trforms[0], ros::Time::now(), "tree_pandar0", "tree_xt32_0"));
            broadcaster.sendTransform(tf::StampedTransform(tree_trforms[1], ros::Time::now(), "tree_pandar1", "tree_xt32_1"));
            broadcaster.sendTransform(tf::StampedTransform(tree_trforms[2], ros::Time::now(), "tree_xt32_1", "tree_xt32_2"));
            broadcaster.sendTransform(tf::StampedTransform(tree_trforms[3], ros::Time::now(), "tree_antenna", "tree_pandar0"));
            broadcaster.sendTransform(tf::StampedTransform(tree_trforms[4], ros::Time::now(), "tree_pandar0", "tree_pandar1"));

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
