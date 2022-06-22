#include <ros/ros.h>
#include <fstream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <rideflux_msgs/SensorDataID.h>

#include <pointcloud_tools/ndt_octree_gpu.h>
#include <pointcloud_tools/ndt_matcher_gpu.h>

using pointcloud_tools::NDTOctreeGPU;
using NDTMatcherGPU = pointcloud_tools::NDTMatcherGPU<NDTOctreeGPU>;
constexpr std::size_t DIM = NDTOctreeGPU::DIM;
constexpr std::size_t PARAM_DIM = NDTMatcherGPU::PARAM_DIM;
using Scalar = typename NDTOctreeGPU::Scalar;
using Vector = typename NDTOctreeGPU::Vector;
using Matrix = typename NDTOctreeGPU::Matrix;
using VectorSet = typename NDTOctreeGPU::VectorSet;
using Transform = typename NDTOctreeGPU::Transform;
using ParamMatrix = typename NDTMatcherGPU::ParamMatrix;
using ParamVector = typename NDTMatcherGPU::ParamVector;
using IndexScalar = typename NDTOctreeGPU::IndexScalar;

std::string root_dirname_ = "/home/gw/v3_1_cal_test/output/pc";

struct MatchingOptions
{
  Vector cell_size;
  NDTMatcherGPU::OPTIM_METHOD optim_method;
  bool use_new_cost;
  std::vector<IndexScalar> max_iter_set;

  MatchingOptions() : cell_size(Vector(0.4, 0.4, 0.4)), optim_method(NDTMatcherGPU::OPTIM_METHOD::NEWTON_TRUSTREGION), use_new_cost(true), max_iter_set({30, 30, 30, 30})
  {
  }
};

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

Transform matchTwoPCs(sensor_msgs::PointCloud2 &pc1, sensor_msgs::PointCloud2 &pc2, const MatchingOptions &option, Transform &T_init)
{
  NDTOctreeGPU::Ptr ndt1 = std::make_shared<NDTOctreeGPU>(option.cell_size);
  ndt1->addPointcloudMsg(pc1);
  ndt1->setNDTDataFromPoints();
  ndt1->computeNDsAtAllLevels();

  NDTOctreeGPU::Ptr ndt2 = std::make_shared<NDTOctreeGPU>(option.cell_size);
  ndt2->addPointcloudMsg(pc2);
  ndt2->setNDTDataFromPoints();
  ndt2->computeNDsAtAllLevels();

  NDTMatcherGPU matcher;
  matcher.setNDTRef(ndt1);
  matcher.setNDTIn(ndt2);

  matcher.setUseNewCost(option.use_new_cost);

  ParamMatrix C;
  bool is_converged;
  Transform T_est;
  for (std::size_t n=0; n<option.max_iter_set.size()-0; ++n)
  {
    matcher.setMaxIteration(option.max_iter_set[n]);
    matcher.setOptimMethod(option.optim_method);
    matcher.setMatchingLevel(option.max_iter_set.size()-1-n);

    T_est = matcher.match(T_init);

    C = matcher.getPoseCovariance();
    is_converged = matcher.isConverged();
  }
  
  return T_est;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "ndt_matcher_example_node");
    ros::NodeHandle nh("~");
    
    rideflux_msgs::SensorDataID data_id1;
    data_id1.vehicle = "ioniq_v3_1";
    data_id1.bag_time = "2022-04-08-10-57-11";
    data_id1.sensor = "hesai_0_pc_und";
    data_id1.time_step = 0;

    rideflux_msgs::SensorDataID data_id2;
    data_id2.vehicle = "ioniq_v3_1";
    data_id2.bag_time = "2022-04-08-10-57-11";
    data_id2.sensor = "hesai_1_pc_und";
    data_id2.time_step = 0;

    std::vector<std::string> field_names{"x","y","z","intensity"};

    sensor_msgs::PointCloud2 pc1,pc2;
    readPointcloud(data_id1, field_names, pc1);
    readPointcloud(data_id2, field_names, pc2);
    
    MatchingOptions option;
    Transform T_init = Transform::eye();
    auto T_est = matchTwoPCs(pc1, pc2, option, T_init); 

    ROS_INFO_STREAM("T_est\n" << T_est);
    
    return 0;
}
