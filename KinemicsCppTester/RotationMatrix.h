#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Extension.h"

using namespace Eigen;

static class RotationMatrix
{
public:
	static Matrix3d GetRotate_X(double x_pi) {
		Eigen::AngleAxisd rotx(x_pi, Eigen::Vector3d(1, 0, 0));
		return rotx.matrix();
	}
	static Matrix3d GetRotate_Y(double y_pi) {
		Eigen::AngleAxisd roty(y_pi, Eigen::Vector3d(0, 1, 0));
		return roty.matrix();
	}
	static Matrix3d GetRotate_Z(double z_pi) {
		Eigen::AngleAxisd rotz(z_pi, Eigen::Vector3d(0, 0, 1));
		return rotz.matrix();
	}

	static Matrix3d GetRotate_XYZ(double x_pi, double y_pi, double z_pi) {
		return GetRotate_X(x_pi) * GetRotate_Y(y_pi) * GetRotate_Z(z_pi);
	}
	static Matrix3d GetRotate_ZYX(double z_pi, double y_pi, double x_pi) {
		return GetRotate_Z(z_pi) * GetRotate_Y(y_pi) * GetRotate_X(x_pi);
	}
	static Matrix3d GetRotate_ZYZ(double z_pi1, double y_pi, double z_pi2) {
		return GetRotate_Z(z_pi1) * GetRotate_Y(y_pi) * GetRotate_Z(z_pi2);
	}

	static Vector3f GetEulerAngle_XYZ(Eigen::Matrix3d mat) {
		float a, b, c;
		if (mat(0, 2) < 1) {
			if (mat(0, 2) > -1) {
				a = toDeg(atan2(-mat(1, 2), mat(2, 2)));
				b = toDeg(asin(mat(0, 2)));
				c = toDeg(atan2(-mat(0, 1), mat(0, 0)));
			}
			else {
				a = -toDeg(atan2(mat(1, 0), mat(1, 1)));
				b = -90.0f;
				c = 0.0f;
			}
		}
		else {
			a = toDeg(atan2(mat(1, 0), mat(1, 1)));
			b = 90.0f;
			c = 0.0f;
		}
		return Vector3f(a, b, c);
	}
	static Vector3f GetEulerAngle_ZYX(Matrix3d mat) {
		float a, b, c;
		if (mat(2,0) < 1) {
			if (mat(2,0) > -1) {
				a = toDeg(atan2f(mat(1, 0), mat(0, 0)));
				b = toDeg(asinf(-mat(2, 0)));
				c = toDeg(atan2f(mat(2, 1), mat(2, 2)));
			}
			else {
				a = -toDeg(atan2f(-mat(1, 2), mat(1, 1)));
				b = 90.0f;
				c = 0.0f;
			}
		}
		else {
			a = toDeg(atan2f(-mat(1, 2), mat(1, 1)));
			b = -90.0f;
			c = 0.0f;
		}
		return Vector3f{ a,b,c };
	}
	static Vector3f GetEulerAngle_ZYZ(Matrix3d mat) {
		float a, b, c;
		if (mat(2,2) < 1) {
			if (mat(2,2) > -1) {
				a = toDeg(atan2f(mat(1, 2), mat(0, 2)));
				b = toDeg(acosf(mat(2, 2)));
				c = toDeg(atan2f(mat(2, 1), -mat(2, 0)));
			}
			else {
				a = -toDeg(atan2f(mat(1, 0), mat(1, 1)));
				b = 180.0f;
				c = 0.0f;
			}
		}
		else {
			a = toDeg(atan2f(mat(1, 0), mat(1, 1)));
			b = 0.0f;
			c = 0.0f;
		}
		return Vector3f( a,b,c );
	}
};

