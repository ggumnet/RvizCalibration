void printTransform(Transform transform){
    rf_geometry::SO<double, 3UL> rotation = transform.getRotation();
    cv::Matx<double, 3UL, 3UL> cv_rotation_matrix = (cv::Matx<double, 3UL, 3UL>)rotation;
    cv::Vec<double, 3UL> cv_translation_vector = transform.getTranslation();
    tf::Quaternion quaternion = mRot2Quat(cv_rotation_matrix);
    ROS_INFO("%f, %f, %f, %f, %f, %f, %f", cv_translation_vector[0], cv_translation_vector[1], cv_translation_vector[2], quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getW());  
}

void printPose(geometry_msgs::Pose pose){
    ROS_INFO("pose xyz xyzw: %f, %f, %f, %f, %f, %f, %f", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}