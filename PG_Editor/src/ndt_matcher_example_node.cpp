#include <tf/transform_broadcaster.h>

#include <ros/ros.h>
#include <fstream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <rideflux_msgs/SensorDataID.h>

#include <pointcloud_tools/ndt_octree_gpu.h>
#include <pointcloud_tools/ndt_matcher_gpu.h>

#include <pg_editor/TransformationInfo.h>
#include <gihyun_custom/rot2quat.h>


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

std::string root_dirname_ = "/home/rideflux/Data/output/pc";

struct MatchingOptions
{
  Vector cell_size;
  NDTMatcherGPU::OPTIM_METHOD optim_method;
  bool use_new_cost;
  std::vector<IndexScalar> max_iter_set;

  MatchingOptions() : cell_size(Vector(0.4, 0.4, 0.4)), optim_method(NDTMatcherGPU::OPTIM_METHOD::NEWTON_TRUSTREGION), use_new_cost(true), max_iter_set({30, 30, 30, 30})
  {
  }
  MatchingOptions(float cell_size) : cell_size(Vector(cell_size, cell_size, cell_size)), optim_method(NDTMatcherGPU::OPTIM_METHOD::NEWTON_TRUSTREGION), use_new_cost(true), max_iter_set({30, 30, 30, 30}){
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
  Transform T_est1;
  for (std::size_t n=0; n<option.max_iter_set.size()-0; ++n)
  {
    matcher.setMaxIteration(option.max_iter_set[n]);
    matcher.setOptimMethod(option.optim_method);
    matcher.setMatchingLevel(option.max_iter_set.size()-1-n);

    T_est1 = matcher.match(T_init);

    C = matcher.getPoseCovariance();
    is_converged = matcher.isConverged();
  }
  
  return T_est1;
}


Transform iterative_ndt(Transform T_init, sensor_msgs::PointCloud2 pc_ref, sensor_msgs::PointCloud2 pc_est, float cell_size1, float cell_size2){
    MatchingOptions option;
    option = MatchingOptions(cell_size1);
    auto T_est = matchTwoPCs(pc_ref, pc_est, option, T_init); 
    option = MatchingOptions(cell_size2);
    T_est = matchTwoPCs(pc_ref, pc_est, option, T_est);
    return T_est;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "ndt_matcher_example_node");
    ros::NodeHandle nh("~");


    
    rideflux_msgs::SensorDataID data_id1, data_id2, data_id3, data_id4, data_id5;
    data_id1.vehicle = data_id2.vehicle = data_id3.vehicle = data_id4.vehicle = data_id5.vehicle = "v5_1_sample_data_2";
    data_id1.bag_time = data_id2.bag_time = data_id3.bag_time = data_id4.bag_time = data_id5.bag_time = "2022-07-05-18-01-07";
    data_id1.time_step = data_id2.time_step = data_id3.time_step = data_id4.time_step = data_id5.time_step = 0;
    
    data_id1.sensor = "pandar64_0";
    data_id2.sensor = "pandar64_1";
    data_id3.sensor = "xt32_0";
    data_id4.sensor = "xt32_1";
    data_id5.sensor = "xt32_2";
    


    std::vector<std::string> field_names{"x","y","z","intensity"};

    sensor_msgs::PointCloud2 pc_pandar0, pc_pandar1, pc_xt0, pc_xt1, pc_xt2;
    sensor_msgs::PointCloud2 pc_fixed_pandar0, pc_fixed_pandar1, pc_fixed_xt0, pc_fixed_xt1, pc_fixed_xt2;
    sensor_msgs::PointCloud2 pc_pgo_pandar0, pc_pgo_pandar1, pc_pgo_xt0, pc_pgo_xt1, pc_pgo_xt2;
    
    
    readPointcloud(data_id1, field_names, pc_pandar0);
    readPointcloud(data_id2, field_names, pc_pandar1);
    readPointcloud(data_id3, field_names, pc_xt0);
    readPointcloud(data_id4, field_names, pc_xt1);
    readPointcloud(data_id5, field_names, pc_xt2);
    
    readPointcloud(data_id1, field_names, pc_fixed_pandar0);
    readPointcloud(data_id2, field_names, pc_fixed_pandar1);
    readPointcloud(data_id3, field_names, pc_fixed_xt0);
    readPointcloud(data_id4, field_names, pc_fixed_xt1);
    readPointcloud(data_id5, field_names, pc_fixed_xt2);

    readPointcloud(data_id1, field_names, pc_pgo_pandar0);
    readPointcloud(data_id2, field_names, pc_pgo_pandar1);
    readPointcloud(data_id3, field_names, pc_pgo_xt0);
    readPointcloud(data_id4, field_names, pc_pgo_xt1);
    readPointcloud(data_id5, field_names, pc_pgo_xt2);


    pc_xt0.header.frame_id = "xt32_0";
    pc_xt1.header.frame_id = "xt32_1";
    pc_xt2.header.frame_id = "xt32_2";
    pc_pandar0.header.frame_id = "pandar0";
    pc_pandar1.header.frame_id = "pandar1";

    pc_fixed_xt0.header.frame_id = "fixed_xt32_0";
    pc_fixed_xt1.header.frame_id = "fixed_xt32_1";
    pc_fixed_xt2.header.frame_id = "fixed_xt32_2";
    pc_fixed_pandar0.header.frame_id = "fixed_pandar0";
    pc_fixed_pandar1.header.frame_id = "fixed_pandar1";
    
    pc_pgo_xt0.header.frame_id = "pgo_xt32_0";
    pc_pgo_xt1.header.frame_id = "pgo_xt32_1";
    pc_pgo_xt2.header.frame_id = "pgo_xt32_2";
    pc_pgo_pandar0.header.frame_id = "pgo_pandar0";
    pc_pgo_pandar1.header.frame_id = "pgo_pandar1";

    ros::Publisher pc_xt0_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_xt0", 1);
    ros::Publisher pc_xt1_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_xt1", 1);
    ros::Publisher pc_xt2_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_xt2", 1);
    ros::Publisher pc_pd0_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_pandar0", 1);
    ros::Publisher pc_pd1_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_pandar1", 1);

    ros::Publisher xt32_0_pub = nh.advertise<pg_editor::TransformationInfo>("/xt32_0", 1);
    ros::Publisher xt32_1_pub = nh.advertise<pg_editor::TransformationInfo>("/xt32_1", 1);
    ros::Publisher xt32_2_pub = nh.advertise<pg_editor::TransformationInfo>("/xt32_2", 1);
    ros::Publisher pd0_pub = nh.advertise<pg_editor::TransformationInfo>("/pandar0", 1);
    ros::Publisher pd1_pub = nh.advertise<pg_editor::TransformationInfo>("/pandar1", 1);



    ros::Publisher fixed_pc_xt0_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_fixed_xt0", 1);
    ros::Publisher fixed_pc_xt1_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_fixed_xt1", 1);
    ros::Publisher fixed_pc_xt2_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_fixed_xt2", 1);
    ros::Publisher fixed_pc_pd0_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_fixed_pandar0", 1);
    ros::Publisher fixed_pc_pd1_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_fixed_pandar1", 1);

    ros::Publisher fixed_xt32_0_pub = nh.advertise<pg_editor::TransformationInfo>("/fixed_xt32_0", 1);
    ros::Publisher fixed_xt32_1_pub = nh.advertise<pg_editor::TransformationInfo>("/fixed_xt32_1", 1);
    ros::Publisher fixed_xt32_2_pub = nh.advertise<pg_editor::TransformationInfo>("/fixed_xt32_2", 1);
    ros::Publisher fixed_pd0_pub = nh.advertise<pg_editor::TransformationInfo>("/fixed_pandar0", 1);
    ros::Publisher fixed_pd1_pub = nh.advertise<pg_editor::TransformationInfo>("/fixed_pandar1", 1);



    ros::Publisher pgo_pc_xt0_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_pgo_xt0", 1);
    ros::Publisher pgo_pc_xt1_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_pgo_xt1", 1);
    ros::Publisher pgo_pc_xt2_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_pgo_xt2", 1);
    ros::Publisher pgo_pc_pd0_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_pgo_pandar0", 1);
    ros::Publisher pgo_pc_pd1_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_pgo_pandar1", 1);


    
    MatchingOptions option;
    // Transform T_init = Transform::eye();
    // auto T_est = matchTwoPCs(pc1, pc2, option, T_init); 

    Transform T_xt0, T_xt1, T_xt2, T_pd0, T_pd1;
    Transform T_fixed_xt0, T_fixed_xt1, T_fixed_xt2, T_fixed_pd0, T_fixed_pd1;
    Transform T_xt0_to_pd1;

    //ROS_INFO_STREAM("T_est\n" << T_est);

    T_xt0.setRotation(cv::Matx<double, 3UL, 3UL>(-1,0,0,0,-1,0,0,0,1));
    T_xt0.setTranslation(cv::Matx31d(4.517, 1.022, -1.589));

    T_xt1.setRotation(cv::Matx<double, 3UL, 3UL>(1,0,0,0,1,0,0,0,1));
    T_xt1.setTranslation(cv::Matx31d(4.517, -1.042, -1.589));

    T_xt2.setRotation(cv::Matx<double, 3UL, 3UL>(0,1,0,-1,0,0,0,0,1));
    T_xt2.setTranslation(cv::Matx31d( -0.631, 0, -2.249));

    T_pd0.setRotation(cv::Matx<double, 3UL, 3UL>(-1,0,0,0,-1,0,0,0,1));
    T_pd0.setTranslation(cv::Matx31d(3.672, 0.930, -0.369));
    
    T_pd1.setRotation(cv::Matx<double, 3UL, 3UL>(1,0,0,0,1,0,0,0,1));
    T_pd1.setTranslation(cv::Matx31d(3.672, -0.925, -0.369));

    
    //Optimization start
    T_fixed_pd0 = T_pd0;

    //T_fixed_pd1 = T_pd0.inv()*T_pd1;  //experiment: parent is pd0, child is pd1 -> move pd1's point to pd0'2 point

    /*
    T_fixed_pd1 = T_fixed_pd0*iterative_ndt(T_pd0.inv()*T_pd1, pc_fixed_pandar0, pc_fixed_pandar1, 0.4, 0.2);
    T_fixed_xt0 = T_fixed_pd0*iterative_ndt(T_pd0.inv()*T_xt0, pc_fixed_pandar0, pc_fixed_xt0, 0.4, 0.2);
    T_fixed_xt1 = T_fixed_pd1*iterative_ndt(T_pd1.inv()*T_xt1, pc_fixed_pandar1, pc_fixed_xt1, 0.4, 0.2);
    T_fixed_xt2 = T_fixed_xt1*iterative_ndt(T_xt1.inv()*T_xt2, pc_fixed_xt1, pc_fixed_xt2, 0.4, 0.2);
    */
    

    //T_fixed_pd1 = T_xt0.inv()*T_pd1;

    T_fixed_pd1 = iterative_ndt(T_pd0.inv()*T_pd1, pc_fixed_pandar0, pc_fixed_pandar1, 0.4, 0.2);
    T_fixed_xt0 = iterative_ndt(T_pd0.inv()*T_xt0, pc_fixed_pandar0, pc_fixed_xt0, 0.4, 0.2);
    T_fixed_xt1 = iterative_ndt(T_pd1.inv()*T_xt1, pc_fixed_pandar1, pc_fixed_xt1, 0.4, 0.2);
    T_fixed_xt2 = iterative_ndt(T_xt1.inv()*T_xt2, pc_fixed_xt1, pc_fixed_xt2, 0.4, 0.2);

    T_xt0_to_pd1 = iterative_ndt(T_xt0.inv()*T_pd1, pc_fixed_xt0, pc_fixed_pandar1, 0.4, 0.2);
    
    
    //Optimization ends

    //est_list : estimation result list(result is Transform type because of function definition)
    //tf::Transform - type used for 

    std::vector<Transform> est_list;
    std::vector<Transform> fixed_est_list;

    est_list.push_back(T_xt0);
    est_list.push_back(T_xt1);
    est_list.push_back(T_xt2);
    est_list.push_back(T_pd0);
    est_list.push_back(T_pd1);

    fixed_est_list.push_back(T_fixed_xt0);
    fixed_est_list.push_back(T_fixed_xt1);
    fixed_est_list.push_back(T_fixed_xt2);
    fixed_est_list.push_back(T_fixed_pd0);
    fixed_est_list.push_back(T_fixed_pd1);

    int N = 5;

    rf_geometry::SO<double, 3UL> rotation;
    cv::Matx<double, 3UL, 3UL> cv_rotation_matrix;
    cv::Vec<double, 3UL> cv_translation_vector;
    tf::TransformBroadcaster broadcaster;
    tf::Quaternion quaternion;
    pg_editor::TransformationInfo transforminfos_xt0, transforminfos_xt1, transforminfos_xt2, transforminfos_pd0, transforminfos_pd1; 
    pg_editor::TransformationInfo transforminfos_fixed_xt0, transforminfos_fixed_xt1, transforminfos_fixed_xt2, transforminfos_fixed_pd0, transforminfos_fixed_pd1; 

    std::vector<pg_editor::TransformationInfo*> tf_info_list;
    std::vector<pg_editor::TransformationInfo*> fixed_tf_info_list;
    
    tf_info_list.push_back(&transforminfos_xt0);
    tf_info_list.push_back(&transforminfos_xt1);
    tf_info_list.push_back(&transforminfos_xt2);
    tf_info_list.push_back(&transforminfos_pd0);
    tf_info_list.push_back(&transforminfos_pd1);

    fixed_tf_info_list.push_back(&transforminfos_fixed_xt0);
    fixed_tf_info_list.push_back(&transforminfos_fixed_xt1);
    fixed_tf_info_list.push_back(&transforminfos_fixed_xt2);
    fixed_tf_info_list.push_back(&transforminfos_fixed_pd0);
    fixed_tf_info_list.push_back(&transforminfos_fixed_pd1);

    for(int i=0; i<5; i++){
        rotation = est_list.at(i).getRotation();
        cv_rotation_matrix = (cv::Matx<double, 3UL, 3UL>)rotation;
        cv_translation_vector = est_list.at(i).getTranslation();
        quaternion = mRot2Quat(cv_rotation_matrix);

        tf_info_list.at(i)->frame_num = i;

        tf_info_list.at(i)->qx = quaternion.getX();
        tf_info_list.at(i)->qy = quaternion.getY();
        tf_info_list.at(i)->qz = quaternion.getZ();
        tf_info_list.at(i)->qw = quaternion.getW();

        tf_info_list.at(i)->tx = cv_translation_vector[0];
        tf_info_list.at(i)->ty = cv_translation_vector[1];
        tf_info_list.at(i)->tz = cv_translation_vector[2];
    }

    std::vector<std::string> sensor_to_sensor;
    
    sensor_to_sensor.push_back("pandar0-xt0");
    sensor_to_sensor.push_back("pandar1-xt1");
    sensor_to_sensor.push_back("xt1-xt2");
    sensor_to_sensor.push_back("");
    sensor_to_sensor.push_back("pandar0-pandar1");


    for(int i=0; i<5; i++){
        rotation = fixed_est_list.at(i).getRotation();
        cv_rotation_matrix = (cv::Matx<double, 3UL, 3UL>)rotation;
        cv_translation_vector = fixed_est_list.at(i).getTranslation();
        quaternion = mRot2Quat(cv_rotation_matrix);

        fixed_tf_info_list.at(i)->frame_num = i;

        fixed_tf_info_list.at(i)->qx = quaternion.getX();
        fixed_tf_info_list.at(i)->qy = quaternion.getY();
        fixed_tf_info_list.at(i)->qz = quaternion.getZ();
        fixed_tf_info_list.at(i)->qw = quaternion.getW();

        fixed_tf_info_list.at(i)->tx = cv_translation_vector[0];
        fixed_tf_info_list.at(i)->ty = cv_translation_vector[1];
        fixed_tf_info_list.at(i)->tz = cv_translation_vector[2];

        if(i!=3){
            ROS_INFO("%s: %f, %f, %f, %f, %f, %f, %f", sensor_to_sensor.at(i).c_str(), cv_translation_vector[0], cv_translation_vector[1], cv_translation_vector[2], quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getW());  
        }
    }

    rotation = T_xt0_to_pd1.getRotation();
    cv_rotation_matrix = (cv::Matx<double, 3UL, 3UL>)rotation;
    cv_translation_vector = T_xt0_to_pd1.getTranslation();
    quaternion = mRot2Quat(cv_rotation_matrix);

    ROS_INFO("xt0 to pd1: %f, %f, %f, %f, %f, %f, %f", cv_translation_vector[0], cv_translation_vector[1], cv_translation_vector[2], quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getW());  

    tf::Matrix3x3 mat(quaternion);


    while(ros::ok()){
        pc_pandar0.header.stamp = ros::Time::now();
        pc_pandar1.header.stamp = ros::Time::now();
        pc_xt0.header.stamp = ros::Time::now();
        pc_xt1.header.stamp = ros::Time::now();
        pc_xt2.header.stamp = ros::Time::now();

        pc_pd0_pub.publish(pc_pandar0);
        pc_pd1_pub.publish(pc_pandar1);
        pc_xt0_pub.publish(pc_xt0);
        pc_xt1_pub.publish(pc_xt1);
        pc_xt2_pub.publish(pc_xt2);
        
        xt32_0_pub.publish(transforminfos_xt0);
        xt32_1_pub.publish(transforminfos_xt1);
        xt32_2_pub.publish(transforminfos_xt2);
        pd0_pub.publish(transforminfos_pd0);
        pd1_pub.publish(transforminfos_pd1);


        pc_fixed_pandar0.header.stamp = ros::Time::now();
        pc_fixed_pandar1.header.stamp = ros::Time::now();
        pc_fixed_xt0.header.stamp = ros::Time::now();
        pc_fixed_xt1.header.stamp = ros::Time::now();
        pc_fixed_xt2.header.stamp = ros::Time::now();

        fixed_pc_pd0_pub.publish(pc_fixed_pandar0);
        fixed_pc_pd1_pub.publish(pc_fixed_pandar1);
        fixed_pc_xt0_pub.publish(pc_fixed_xt0);
        fixed_pc_xt1_pub.publish(pc_fixed_xt1);
        fixed_pc_xt2_pub.publish(pc_fixed_xt2);
        
        fixed_xt32_0_pub.publish(transforminfos_fixed_xt0);
        fixed_xt32_1_pub.publish(transforminfos_fixed_xt1);
        fixed_xt32_2_pub.publish(transforminfos_fixed_xt2);
        fixed_pd0_pub.publish(transforminfos_fixed_pd0);
        fixed_pd1_pub.publish(transforminfos_fixed_pd1);


        pc_pgo_pandar0.header.stamp = ros::Time::now();
        pc_pgo_pandar1.header.stamp = ros::Time::now();
        pc_pgo_xt0.header.stamp = ros::Time::now();
        pc_pgo_xt1.header.stamp = ros::Time::now();
        pc_pgo_xt2.header.stamp = ros::Time::now();

        pgo_pc_pd0_pub.publish(pc_pgo_pandar0);
        pgo_pc_pd1_pub.publish(pc_pgo_pandar1);
        pgo_pc_xt0_pub.publish(pc_pgo_xt0);
        pgo_pc_xt1_pub.publish(pc_pgo_xt1);
        pgo_pc_xt2_pub.publish(pc_pgo_xt2);
    

        ros::Duration(0.1).sleep();
    }
    return 0;
}