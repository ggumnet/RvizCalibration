#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pg_editor/TransformationInfo.h>
#include <map>
#include <cmath>
#include <visualization_msgs/Marker.h>

const int N=5;
tf::Vector3 origin_list[N];

std_msgs::Int32MultiArray index_array;
ros::Publisher pc_viz_index_pubs, arrow_edge_pubs, add_index_pubs, remove_index_pubs;

int index1, index2;

bool index_pair_done = true, first_index_done = false;


void resetIndex(){
    index1 = -1;
    index2 = -1;
}

void printVector3D(tf::Vector3 vector){
    ROS_INFO("xyz: %f %f %f", vector.getX(), vector.getY(), vector.getZ());
}

float get3dDistance(tf::Vector3 vector1, tf::Vector3 vector2){
    return pow(pow(vector1.getX()-vector2.getX(),2) + pow(vector1.getY()-vector2.getY(),2) + pow(vector1.getZ()-vector2.getZ(),2), 0.5); 
}

void pgoTFHandlerCallback(const pg_editor::TransformationInfoConstPtr &msg)
{
    //ROS_INFO("called");
    tf::Vector3* origin;
    origin = &origin_list[msg->frame_num];

    origin->setX(msg->tx); origin->setY(msg->ty); origin->setZ(msg->tz);
}

int findClosestPoint(tf::Vector3 new_vector){
    float min = INT_MAX, temp;
    int index;
    for(int i=0; i<N; i++){
        temp = get3dDistance(new_vector, origin_list[i]);
        if(temp<min) {
            min = temp;
            index = i;
        }
    }
    return index;
}

geometry_msgs::Point vector3toPoint(tf::Vector3 vector3){
    geometry_msgs::Point point;
    point.x = vector3.getX();
    point.y = vector3.getY();
    point.z = vector3.getZ();
    return point;
}

void publishArrowEdge(int index1, int index2){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "pgo_antenna";
    marker.header.stamp = ros::Time::now();

    marker.type = visualization_msgs::Marker::ARROW;
    marker.ns = "edges";
    marker.id = 1;

    marker.points.clear();
    marker.points.push_back(geometry_msgs::Point());
    marker.points.push_back(geometry_msgs::Point());

    tf::Vector3 origin1 = origin_list[index1], origin2 = origin_list[index2];
    marker.points.at(0) = vector3toPoint(origin1);
    marker.points.at(1) = vector3toPoint(origin2);

    marker.scale.x = 0.1;
    marker.scale.y = 0.2;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    arrow_edge_pubs.publish(marker);
}

//publish which pointcloud to publish and publish arrow marker
void aClickCallback(const geometry_msgs::PointConstPtr &msg){
    int index = findClosestPoint(tf::Vector3(msg->x, msg->y, msg->z));
    ROS_INFO("closest index: %d", index);

    first_index_done = true;

    for(int i=0; i<N; i++){
        index_array.data.at(i) = 0;
    }
    if(index_pair_done){
        index_pair_done = false;
        index1 = index;
        index_array.data.at(index) = 1;
        pc_viz_index_pubs.publish(index_array);
    }
    else{
        index2 = index;
        index_array.data.at(index1) = 1;
        index_array.data.at(index2) = 1;
        pc_viz_index_pubs.publish(index_array);
        publishArrowEdge(index1, index2);
        index_pair_done = true;
    }
}

void addMsgCallback(const std_msgs::EmptyConstPtr &msg){
    if(!first_index_done) return ;
    if(!index_pair_done) {
        ROS_INFO("return");
        return;
    }
    ROS_INFO("Add");
    std_msgs::Int32MultiArray add_msg;
    add_msg.data.push_back(index1);
    add_msg.data.push_back(index2);
    add_index_pubs.publish(add_msg);
    for(int i=0; i<N; i++){
        index_array.data.at(i) = 1;
    }
    pc_viz_index_pubs.publish(index_array);
}

void removeMsgCallback(const std_msgs::EmptyConstPtr &msg){
    if(!first_index_done) return ;
    if(!index_pair_done) {
        ROS_INFO("return");
        return;
    }
    ROS_INFO("Remove");
    std_msgs::Int32MultiArray remove_msg;
    remove_msg.data.push_back(index1);
    remove_msg.data.push_back(index2);
    remove_index_pubs.publish(remove_msg);
    for(int i=0; i<N; i++){
        index_array.data.at(i) = 1;
    }
    pc_viz_index_pubs.publish(index_array);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "a_click_receiver_node");
    ros::NodeHandle nh;

    tf::TransformBroadcaster broadcaster;

    ros::Subscriber pgo_pd0_subs = nh.subscribe("/pgo_pandar0", 1, pgoTFHandlerCallback);
    ros::Subscriber pgo_pd1_subs = nh.subscribe("/pgo_pandar1", 1, pgoTFHandlerCallback);
    ros::Subscriber pgo_xt32_0_subs = nh.subscribe("/pgo_xt32_0", 1, pgoTFHandlerCallback);
    ros::Subscriber pgo_xt32_1_subs = nh.subscribe("/pgo_xt32_1", 1, pgoTFHandlerCallback);
    ros::Subscriber pgo_xt32_2_subs = nh.subscribe("/pgo_xt32_2", 1, pgoTFHandlerCallback);

    ros::Subscriber a_click_subs = nh.subscribe("/rf_tool_a_click", 1, aClickCallback);
    ros::Subscriber add_msg_subs = nh.subscribe("/pg_editor_panel/add", 1, addMsgCallback);
    ros::Subscriber remove_msg_subs = nh.subscribe("/pg_editor_panel/remove", 1, removeMsgCallback);

    pc_viz_index_pubs = nh.advertise<std_msgs::Int32MultiArray>("/visualize_index_array", 1);
    arrow_edge_pubs = nh.advertise<visualization_msgs::Marker>("edge_arrow", 1);
    add_index_pubs = nh.advertise<std_msgs::Int32MultiArray>("add_edge_index_array",1, true);
    remove_index_pubs = nh.advertise<std_msgs::Int32MultiArray>("remove_edge_index_array",1, true);

    for(int i=0; i<N; i++){
        index_array.data.push_back(0);
    }

    while (ros::ok())
    {
        //printVector3D(origin_list[0]);
        //printVector3D(origin_list[1]);
        //printVector3D(origin_list[2]);
        //printVector3D(origin_list[3]);
        //printVector3D(origin_list[4]);
        //ROS_INFO("%d %d %d %d %d", index_array.data.at(0), index_array.data.at(1), index_array.data.at(2), index_array.data.at(3), index_array.data.at(4));

        try
        {
            ros::Duration(1.0).sleep();
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(0.1).sleep();
            continue;
        }
        ros::spinOnce();
    }
}
