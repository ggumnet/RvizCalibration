#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pg_editor/GetPointcloud.h>
#include <map>
#include <fstream>
#include <boost/tokenizer.hpp>
#include <rideflux_msgs/SensorDataID.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pg_editor/TransformInfo.h>
#include <pg_lib/types.h>
#include <pg_editor/GetImuPoseResult.h>

using namespace pg_lib;
#include "transform_pose_conversion.h"
#include "print_tool.h"
#include "configurations.h"
#include "data_reader_server.h"
#include "read_configuration.h"

void parse_imu_data(const std::string& str, const std::string delimiters){
    boost::char_separator<char> sep(delimiters.c_str());
    boost::tokenizer<boost::char_separator<char>> tok(str, sep);

    int n = 0;
    boost::tokenizer<boost::char_separator<char>>::iterator itr = tok.begin();
    
    pg_lib::Transform transform;
    geometry_msgs::Pose pose;
    
    double ta[12]; //temp_array

    for(int i=0; i<frame_num; i++){
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
        ECEF_transforms_vec_.push_back(transform);
        //poses_vec_.push_back(transformToPose(transform));
    }
    ROS_WARN("read done");
}

std::string createPCDirectoryPath(const rideflux_msgs::SensorDataID &msg) 
{
    return root_dirname_ + msg.vehicle + "/" + msg.bag_time + "/" + msg.sensor;
}

std::string createPCFilePath(const rideflux_msgs::SensorDataID &msg)
{
    std::stringstream ss;
    ss.width(6);
    ss.fill('0');
    ss << msg.time_step;
    return createPCDirectoryPath(msg) + "/" + ss.str() + ".bin";
}

bool readPointcloud(const rideflux_msgs::SensorDataID &id, const std::vector<std::string> &field_names_, sensor_msgs::PointCloud2 &pc)
{
    std::string filepath = createPCFilePath(id);

    ros::SerializedMessage serialized;
    std::ifstream ifs;
    ifs.open(filepath.c_str(), std::ifstream::binary);
    if (!ifs.is_open())
    {
        ROS_ERROR("[PointcloudServer::readPointcloud] Failed to open pc file %s.", filepath.c_str());
        ifs.close();
        return false;
    }
    sensor_msgs::PointCloud2 read_pc;
    try
    {
        ifs.seekg(0, ifs.end);
        serialized.num_bytes = ifs.tellg();
        ifs.seekg(0, ifs.beg);
        serialized.buf.reset(new uint8_t[serialized.num_bytes]);
        ifs.read(reinterpret_cast<char *>(serialized.buf.get()), serialized.num_bytes);
        ifs.close();
        serialized.message_start = serialized.buf.get() + 4;
        ros::serialization::deserializeMessage(serialized, read_pc);
    }
    catch (std::exception &e)
    {
        ROS_ERROR("[PointcloudServer::readPointcloud] Failed to read pc file %s. %s", filepath.c_str(), e.what());
        ifs.close();
        return false;
    }

    pc.header = read_pc.header;
    pc.width = read_pc.width;
    pc.height = read_pc.height;
    pc.is_bigendian = read_pc.is_bigendian;
    pc.is_dense = read_pc.is_dense;

    auto field_names = field_names_;
    if (field_names.empty())
    {
        for (const auto &field : read_pc.fields)
        field_names.emplace_back(field.name);
    }

    std::vector<int> read_pc_idxs;
    pc.fields.clear();
    int offset = 0;
    for (const auto &name : field_names)
    {
        int idx = sensor_msgs::getPointCloud2FieldIndex(read_pc, name);
        if (idx == -1)
        continue;

        read_pc_idxs.push_back(idx);

        pc.fields.push_back(read_pc.fields[idx]);
        auto &field = pc.fields.back();
        field.offset = offset;
        switch (field.datatype)
        {
        case sensor_msgs::PointField::INT8:
            offset += sizeof(sensor_msgs::pointFieldTypeAsType<sensor_msgs::PointField::INT8>::type);
            break;
        case sensor_msgs::PointField::UINT8:
            offset += sizeof(sensor_msgs::pointFieldTypeAsType<sensor_msgs::PointField::UINT8>::type);
            break;
        case sensor_msgs::PointField::INT16:
            offset += sizeof(sensor_msgs::pointFieldTypeAsType<sensor_msgs::PointField::INT16>::type);
            break;
        case sensor_msgs::PointField::UINT16:
            offset += sizeof(sensor_msgs::pointFieldTypeAsType<sensor_msgs::PointField::UINT16>::type);
            break;
        case sensor_msgs::PointField::INT32:
            offset += sizeof(sensor_msgs::pointFieldTypeAsType<sensor_msgs::PointField::INT32>::type);
            break;
        case sensor_msgs::PointField::UINT32:
            offset += sizeof(sensor_msgs::pointFieldTypeAsType<sensor_msgs::PointField::UINT32>::type);
            break;
        case sensor_msgs::PointField::FLOAT32:
            offset += sizeof(sensor_msgs::pointFieldTypeAsType<sensor_msgs::PointField::FLOAT32>::type);
            break;
        case sensor_msgs::PointField::FLOAT64:
            offset += sizeof(sensor_msgs::pointFieldTypeAsType<sensor_msgs::PointField::FLOAT64>::type);
            break;
        }
    }
    pc.point_step = offset;
    pc.row_step = pc.point_step * pc.width;
    pc.data.resize(pc.height*pc.row_step);

    for (std::size_t k=0; k<pc.fields.size(); ++k)
    {
        const auto &field = pc.fields[k];
        const auto &read_pc_offset = read_pc.fields[read_pc_idxs[k]].offset;
        for (std::size_t j=0; j<pc.height; ++j)
        {
        for (std::size_t i=0; i<pc.width; ++i)
        {
            switch (field.datatype)
            {
            case sensor_msgs::PointField::INT8:
                std::memcpy(&pc.data[j*pc.row_step + i*pc.point_step + field.offset],
                            &read_pc.data[j*read_pc.row_step + i*read_pc.point_step + read_pc_offset],
                            sizeof(sensor_msgs::pointFieldTypeAsType<sensor_msgs::PointField::INT8>::type));
                break;
            case sensor_msgs::PointField::UINT8:
                std::memcpy(&pc.data[j*pc.row_step + i*pc.point_step + field.offset],
                            &read_pc.data[j*read_pc.row_step + i*read_pc.point_step + read_pc_offset],
                            sizeof(sensor_msgs::pointFieldTypeAsType<sensor_msgs::PointField::UINT8>::type));
                break;
            case sensor_msgs::PointField::INT16:
                std::memcpy(&pc.data[j*pc.row_step + i*pc.point_step + field.offset],
                            &read_pc.data[j*read_pc.row_step + i*read_pc.point_step + read_pc_offset],
                            sizeof(sensor_msgs::pointFieldTypeAsType<sensor_msgs::PointField::INT16>::type));
                break;
            case sensor_msgs::PointField::UINT16:
                std::memcpy(&pc.data[j*pc.row_step + i*pc.point_step + field.offset],
                            &read_pc.data[j*read_pc.row_step + i*read_pc.point_step + read_pc_offset],
                            sizeof(sensor_msgs::pointFieldTypeAsType<sensor_msgs::PointField::UINT16>::type));
                break;
            case sensor_msgs::PointField::INT32:
                std::memcpy(&pc.data[j*pc.row_step + i*pc.point_step + field.offset],
                            &read_pc.data[j*read_pc.row_step + i*read_pc.point_step + read_pc_offset],
                            sizeof(sensor_msgs::pointFieldTypeAsType<sensor_msgs::PointField::INT32>::type));
                break;
            case sensor_msgs::PointField::UINT32:
                std::memcpy(&pc.data[j*pc.row_step + i*pc.point_step + field.offset],
                            &read_pc.data[j*read_pc.row_step + i*read_pc.point_step + read_pc_offset],
                            sizeof(sensor_msgs::pointFieldTypeAsType<sensor_msgs::PointField::UINT32>::type));
                break;
            case sensor_msgs::PointField::FLOAT32:
                std::memcpy(&pc.data[j*pc.row_step + i*pc.point_step + field.offset],
                            &read_pc.data[j*read_pc.row_step + i*read_pc.point_step + read_pc_offset],
                            sizeof(sensor_msgs::pointFieldTypeAsType<sensor_msgs::PointField::FLOAT32>::type));
                break;
            case sensor_msgs::PointField::FLOAT64:
                std::memcpy(&pc.data[j*pc.row_step + i*pc.point_step + field.offset],
                            &read_pc.data[j*read_pc.row_step + i*read_pc.point_step + read_pc_offset],
                            sizeof(sensor_msgs::pointFieldTypeAsType<sensor_msgs::PointField::FLOAT64>::type));
                break;
            }
        }
        }
    }
    return true;
}

void readIMUdata(){
    std::string imu_read_string = "";
    std::ifstream imu_file(imu_file_name_);
    if (imu_file.is_open()) {
        while (imu_file) {
            std::string s;
            getline(imu_file, s);  
            imu_read_string+=s+"\n";
        }  
        imu_file.close();
    } else {
        std::cout << "file open failed" << std::endl;
    }
    std::string delimiters = " \n\t";
    parse_imu_data(imu_read_string, delimiters);
}

bool IMUPoseResultCallback(pg_editor::GetImuPoseResult::Request &req, pg_editor::GetImuPoseResult::Response &res){
    ROS_INFO("imu response %d", poses_vec_.size());
    for(int i=0; i<poses_vec_.size(); i++)
    {
        res.pose_array.poses.push_back(poses_vec_.at(i));
    }
    return true;
}

bool pcReadCallback(pg_editor::GetPointcloud::Request &req, pg_editor::GetPointcloud::Response &res){
    sensor_msgs::PointCloud2 pc;
    if(!readPointcloud(req.data_id, field_names, pc))
        return false;
    pc.header.frame_id = req.data_id.sensor;
    res.pointcloud = pc;
    return true;
}

//convert ECEF coordinates to IMU0 coordinates
void ECEFToIMU0Conversion(){
    pg_lib::Transform transform0 = ECEF_transforms_vec_.at(0), temp_transform;
    IMU0_transforms_vec_.push_back(pg_lib::Transform::eye());
    poses_vec_.push_back(transformToPose(pg_lib::Transform::eye()));

    for(int i=1; i<ECEF_transforms_vec_.size(); i++){
        temp_transform = ECEF_transforms_vec_.at(i);
        IMU0_transforms_vec_.push_back(transform0.inv()*temp_transform); 
        poses_vec_.push_back(transformToPose(transform0.inv()*temp_transform));
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_reader_server");
    ros::NodeHandle nh;
    readConfiguration();
    data_dirname_ = root_dirname_+vehicle+"/"+bag_time;
    imu_file_name_ = data_dirname_+"/pc_pose.txt";
    readIMUdata();
    ECEFToIMU0Conversion();
    ros::ServiceServer pc_read_service = nh.advertiseService("/pc_read_service", pcReadCallback);
    ros::ServiceServer imu_pose_result_service = nh.advertiseService("/imu_pose_result", IMUPoseResultCallback);
    ros::spin();
}
