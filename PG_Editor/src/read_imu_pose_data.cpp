#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pg_editor/TransformationInfo.h>
#include <map>
#include <fstream>
#include <boost/tokenizer.hpp>

std::string root_dirname_ = "/home/rideflux/v5_1_sample_data_1";
std::string file_name_ = root_dirname_+"/pc_pose.txt";


typedef class transform{
    public:
        double rotation[9];
        double translation[9];
} TRANSFORM;

void parse(const std::string& str, const std::string delimiters){

    boost::char_separator<char> sep(delimiters.c_str());
    boost::tokenizer<boost::char_separator<char>> tok(str, sep);

    int n = 0;
    // for(boost::tokenizer<boost::char_separator<char>>::iterator i = tok.begin(); i != tok.end(); ++i){
    //     std::cout << *i << std::endl;
    //     n++;
    // }
    boost::tokenizer<boost::char_separator<char>>::iterator itr = tok.begin();

    
    std::vector<TRANSFORM> transforms_vec_;
    TRANSFORM transform;
    
    for(int i=0; i<12; i++){
        ++itr;
        for(int j=0; j<12; j++){
            if(j%4==3){
                transform.translation[j/4] = stod(*itr);
            }
            else{
                transform.rotation[j-j/4] = stod(*itr); 
            }
            ++itr;
        }
        transforms_vec_.push_back(transform);
        for(int j=0; j<21; j++){
            ++itr;
        }
    }
    // for(int i=0; i<12; i++){
    //     for(int j=0; j<9; j++){
    //         ROS_INFO("%f", transforms_vec_.at(i).rotation[j]);
    //     }
    //     for(int j=0; j<3; j++){
    //         ROS_INFO("%f", transforms_vec_.at(i).translation[j]);
    //     }
    // }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "read_imu_pose_data");
    ros::NodeHandle nh;
    std::string read_string = "";

    std::ifstream file(file_name_);

    if (file.is_open()) {
        while (file) {
            std::string s;
            getline(file, s);  
            read_string+=s+"\n";
        }  
        file.close();
    } else {
        std::cout << "file open fail" << std::endl;
    }

    std::string delimiters = " \n\t";
    parse(read_string, delimiters);
    //ROS_INFO("%s", read_string.c_str());

}
