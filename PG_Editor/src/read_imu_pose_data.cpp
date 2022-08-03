//Experimental Node
//Not actually executed

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pg_editor/TransformInfo.h>
#include <map>
#include <fstream>
#include <boost/tokenizer.hpp>
#include <pg_lib/types.h>
#include <geometry_msgs/Pose.h>
#include <pg_editor/GetImuPoseResult.h>

using namespace pg_lib;

#include <transform_pose_conversion.h>

namespace initconfiguration{
    std::string root_dirname_ = "/home/rideflux/v5_1_sample_data_1/solati_v5_1/2022-07-14-11-46-25";
    std::string file_name_ = root_dirname_+"/pc_pose.txt";
    const int pose_num = 12; 
}

std::vector<pg_lib::Transform> transforms_vec_;
std::vector<geometry_msgs::Pose> poses_vec_;

void parse(const std::string& str, const std::string delimiters){
    boost::char_separator<char> sep(delimiters.c_str());
    boost::tokenizer<boost::char_separator<char>> tok(str, sep);

    int n = 0;
    // for(boost::tokenizer<boost::char_separator<char>>::iterator i = tok.begin(); i != tok.end(); ++i){
    //     std::cout << *i << std::endl;
    //     n++;
    // }
    boost::tokenizer<boost::char_separator<char>>::iterator itr = tok.begin();
    
    pg_lib::Transform transform;
    double ta[12]; //temp_array

    for(int i=0; i<initconfiguration::pose_num; i++){
        ++itr;
        for(int j=0; j<12; j++){
            ta[j] = stod(*itr); 
            ++itr;
        }
        transform.setRotation(cv::Matx<double, 3UL, 3UL>(ta[0], ta[1], ta[2], ta[4], ta[5], ta[6], ta[8], ta[9], ta[10]));
        transform.setTranslation(cv::Matx31d(ta[3], ta[7], ta[11]));
        for(int j=0; j<21; j++){
            ++itr;
        }
        transforms_vec_.push_back(transform);
    }
    // for(int i=0; i<12; i++){
    //     for(int j=0; j<12; j++){
    //         ROS_WARN("%f", transforms_vec_.at(i)(j/4, j-(j/4)*4));
    //     }
    // }
    ROS_WARN("read done");
}

bool IMUPoseResultCallback(pg_editor::GetImuPoseResult::Request &req, pg_editor::GetImuPoseResult::Response &res){
    for(int i=0; i<poses_vec_.size(); i++)
    {
        res.pose_array.poses.push_back(poses_vec_.at(i));
    }
}

void transform_to_pose_conversion(){
    for(int i=0; i<initconfiguration::pose_num; i++){
        poses_vec_.push_back(transformToPose(transforms_vec_.at(i)));
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "read_imu_pose_data");
    ros::NodeHandle nh;
    std::string read_string = "";
    std::ifstream file(initconfiguration::file_name_);
    ros::ServiceServer imu_pose_result_service = nh.advertiseService("/imu_pose_result", IMUPoseResultCallback);

    if (file.is_open()) {
        while (file) {
            std::string s;
            getline(file, s);  
            read_string+=s+"\n";
        }  
        file.close();
    } else {
        std::cout << "file open failed" << std::endl;
    }
    std::string delimiters = " \n\t";
    parse(read_string, delimiters);
    transform_to_pose_conversion();
    ros::spin();
}
