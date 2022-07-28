#ifndef TRANSFORM_POSE_CONVERSION_H
#define TRANSFORM_POSE_CONVERSION_H

geometry_msgs::Pose transformToPose(Transform transform){

    rf_geometry::SO<double, 3UL> rotation = transform.getRotation();
    cv::Matx<double, 3UL, 3UL> cv_rotation_matrix = (cv::Matx<double, 3UL, 3UL>)rotation;;
    cv::Vec<double, 3UL> cv_translation_vector = transform.getTranslation();
    tf::Quaternion quaternion = mRot2Quat(cv_rotation_matrix);
    
    geometry_msgs::Pose pose;

    pose.position.x = cv_translation_vector[0]; 
    pose.position.y = cv_translation_vector[1];
    pose.position.z = cv_translation_vector[2];
    pose.orientation.x = quaternion.getX();
    pose.orientation.y = quaternion.getY();
    pose.orientation.z = quaternion.getZ();
    pose.orientation.w = quaternion.getW();

    return pose;
}

Transform poseToTransform(geometry_msgs::Pose pose){
    Transform transform;
    transform.setRotation(mQuat2Rot(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)));
    transform.setTranslation(cv::Matx31d(pose.position.x, pose.position.y, pose.position.z));
    return transform;
}

tf::Transform poseToTfTransform(geometry_msgs::Pose pose)
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
    return transform;
}

#endif