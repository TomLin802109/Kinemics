#pragma once
#include <math.h>
#include "struct.h"
#include "Extension.h"

namespace QrMatrix {
	Matrix4f getRotate_x(float angle_pi);
	Matrix4f getRotate_y(float angle_pi);
	Matrix4f getRotate_z(float angle_pi);
	Matrix4f getRotate_xyz(float rx_pi, float ry_pi, float rz_pi);
	Matrix4f getRotate_zyx(float rz_pi, float ry_pi, float rx_pi);
	Matrix4f getRotate_zyz(float rz1_pi, float ry_pi, float rz2_pi);
	Matrix4f Multiply(Matrix4f A, Matrix4f B);
	Vector4f Multiply(Matrix4f A, Vector4f v);
	Vector3f getEulerAngle_xyz(Matrix4f mat);
	Vector3f getEulerAngle_zyx(Matrix4f mat);
	Vector3f getEulerAngle_zyz(Matrix4f mat);
	Matrix4f Transpose(Matrix4f mat);
	float Determinant(Matrix3f mat);
	float Determinant(Matrix4f mat);
	Matrix4f Inverse(Matrix4f mat);
}
