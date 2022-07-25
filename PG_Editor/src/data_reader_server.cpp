#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pg_editor/TransformationInfo.h>
#include <pg_editor/GetPointcloud.h>
#include <map>
#include <fstream>
#include <boost/tokenizer.hpp>
#include <rideflux_msgs/SensorDataID.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

std::string root_dirname_ = "/home/rideflux/v5_1_sample_data_1";
std::string vehicle_ = "solati_v5_1";
std::string bag_time_ = "2022-07-14-11-46-25";
std::string data_dir_ = root_dirname_+"/"+vehicle_+"/"+bag_time_;
std::string imu_file_name_ = data_dir_+"/pc_pose.txt";
std::vector<std::string> field_names{"x","y","z","intensity"};

typedef class transform{
    public:
        double rotation[9];
        double translation[9];
} TRANSFORM;

std::vector<TRANSFORM> transforms_vec_;

void parse_imu_data(const std::string& str, const std::string delimiters){

    boost::char_separator<char> sep(delimiters.c_str());
    boost::tokenizer<boost::char_separator<char>> tok(str, sep);

    int n = 0;
    boost::tokenizer<boost::char_separator<char>>::iterator itr = tok.begin();

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
}

std::string createPCDirectoryPath(const rideflux_msgs::SensorDataID &msg) 
{
    return root_dirname_ + "/" + msg.vehicle + "/" + msg.bag_time + "/" + msg.sensor;
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

void read_imu_data(){
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
        std::cout << "imu_file open fail" << std::endl;
    }

    std::string delimiters = " \n\t";
    parse_imu_data(imu_read_string, delimiters);
}

bool pc_read_callback(pg_editor::GetPointcloud::Request &req, pg_editor::GetPointcloud::Response &res){
    std::string pointcloud_name = data_dir_+"/"+req.pointcloud_name;
    sensor_msgs::PointCloud2 pc;

    rideflux_msgs::SensorDataID data_id;
    data_id.vehicle = vehicle_;
    data_id.bag_time = bag_time_;
    data_id.time_step = 0;
    data_id.sensor = req.pointcloud_name;

    readPointcloud(data_id, field_names, pc);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_reader_server");
    ros::NodeHandle nh;
    read_imu_data();

    ros::ServiceServer pc_read_service = nh.advertiseService("pc__read_service", pc_read_callback);
}
