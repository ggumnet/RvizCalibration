#ifndef ROTATION_TO_QUATERNION_H
#define ROTATION_TO_QUATERNION_H


inline float SIGN(float x) { 
	return (x >= 0.0f) ? +1.0f : -1.0f; 
}

inline float NORM(float a, float b, float c, float d) { 
	return sqrt(a * a + b * b + c * c + d * d); 
}

cv::Matx<double, 3UL, 3UL> mQuat2Rot(tf::Quaternion q){
    tf::Matrix3x3 m;
    cv::Matx<double, 3UL, 3UL> rotation_matrix;
    m.setRotation(q);
    rotation_matrix(0,0) = m[0][0];
    rotation_matrix(0,1) = m[0][1];
    rotation_matrix(0,2) = m[0][2];
    rotation_matrix(1,0) = m[1][0];
    rotation_matrix(1,1) = m[1][1];
    rotation_matrix(1,2) = m[1][2];
    rotation_matrix(2,0) = m[2][0];
    rotation_matrix(2,1) = m[2][1];
    rotation_matrix(2,2) = m[2][2];     
    return rotation_matrix;
}

tf::Quaternion mRot2Quat(cv::Matx<double, 3UL, 3UL> m) {
    	tf::Quaternion quaternion;

	float r11 = m.val[0];
	float r12 = m.val[1];
	float r13 = m.val[2];
	float r21 = m.val[3];
	float r22 = m.val[4];
	float r23 = m.val[5];
	float r31 = m.val[6];
	float r32 = m.val[7];
	float r33 = m.val[8];
	float q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
	float q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
	float q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
	float q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;


	if (q0 < 0.0f) {
		q0 = 0.0f;
	}
	if (q1 < 0.0f) {
		q1 = 0.0f;
	}
	if (q2 < 0.0f) {
		q2 = 0.0f;
	}
	if (q3 < 0.0f) {
		q3 = 0.0f;
	}
	q0 = sqrt(q0);
	q1 = sqrt(q1);
	q2 = sqrt(q2);
	q3 = sqrt(q3);
    
	if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
		q0 *= +1.0f;
		q1 *= SIGN(r32 - r23);
		q2 *= SIGN(r13 - r31);
		q3 *= SIGN(r21 - r12);
	}
	else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
		q0 *= SIGN(r32 - r23);
		q1 *= +1.0f;
		q2 *= SIGN(r21 + r12);
		q3 *= SIGN(r13 + r31);
	}
	else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
		q0 *= SIGN(r13 - r31);
		q1 *= SIGN(r21 + r12);
		q2 *= +1.0f;
		q3 *= SIGN(r32 + r23);
	}
	else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
		q0 *= SIGN(r21 - r12);
		q1 *= SIGN(r31 + r13);
		q2 *= SIGN(r32 + r23);
		q3 *= +1.0f;
	}
	else {
		printf("coding error\n");
	}
	float r = NORM(q0, q1, q2, q3);
	q0 /= r;
	q1 /= r;
	q2 /= r;
	q3 /= r;

	quaternion.setW(q0);
    quaternion.setX(q1);
    quaternion.setY(q2);
    quaternion.setZ(q3);
	return quaternion;
}


#endif