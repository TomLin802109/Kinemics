#pragma once

#include "PolynomialRoot.h"
#include "RotationMatrix.h"
#include "struct.h"
#include "Matrix.h"
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

class RobotArmKinematics {
public:
	RobotArmKinematics(RobotSpec spec, WorldCoordinate tool = WorldCoordinate{0,0,0,0,0,0}) {
		this->dh_table = spec.dh_table;
		this->limits = spec.limits;
		this->eulerType = spec.EulerType;
		this->tool = tool;
		if (dh_table.size() < 7) {
			throw std::invalid_argument("The rows of input DH table was lass than 7 rows");
			return;
		}
		k1 = get_coeffs_k1();
		k2 = get_coeffs_k2();
		k3 = get_coeffs_k3();
		k4 = get_coeffs_k4();
		//std::cout << std::fixed << std::setprecision(2);
		std::cout << "K1 coeff " << k1[0] << "," << k1[1] << "," << k1[2] << std::endl;
		std::cout << "K2 coeff " << k2[0] << "," << k2[1] << "," << k2[2] << std::endl;
		std::cout << "K3 coeff " << k3[0] << "," << k3[1] << "," << k3[2] << std::endl;
		std::cout << "K4 coeff " << k4[0] << "," << k4[1] << "," << k4[2] << std::endl;
	}
	~RobotArmKinematics() {
		delete[] k1, k2, k3, k4;
	}

	WorldCoordinate Forward(JointCoordinate jc) {
		float crd[7]{ jc.j1,jc.j2 ,jc.j3 ,jc.j4 ,jc.j5 ,jc.j6 };
		Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
		std::cout << "Forward Kinematic" << std::endl;
		for (int i = 0; i < dh_table.size(); i++) {
			auto param = dh_table[i];
			mat = mat * getTransMat_Craig(DH_param{ param.alpha, param.a, param.d, crd[i] + param.theta });
			std::cout << "No." << i + 1 << endl << mat << endl;
		}
		Matrix3d r07 = mat.block<3, 3>(0, 0);
		Eigen::Vector3d shift = mat.block<3, 3>(0, 0) * Eigen::Vector3d(tool.x, tool.y, tool.z);
		mat.block<3, 3>(0, 0) = mat.block<3, 3>(0, 0) * RotationMatrix::GetRotate(toRad(tool.a), toRad(tool.b), toRad(tool.c), eulerType);
		std::cout << std::endl << "Final Transformation Matrix" << endl << mat << endl;
		Eigen::Vector3f angle = RotationMatrix::GetEulerAngle(mat.block<3, 3>(0, 0), eulerType);
		return WorldCoordinate{ (float)round(mat(0,3) + shift(0,0),2),(float)round(mat(1,3) + shift(1,0),2), (float)round(mat(2,3) + shift(2,0),2),
								round(angle.x(),2), round(angle.y(),2), round(angle.z(),2) };
	};

	JointCoordinate Inverse(WorldCoordinate rc, PostureCfg cfg = PostureCfg()) {
		//Calculate the transformation matrix from base to flange.
		Matrix4d t07 = Matrix4d::Identity();
		Matrix3d toolRot = RotationMatrix::GetRotate_XYZ(toRad(tool.a), toRad(tool.b), toRad(tool.c));
		t07.block<3, 3>(0, 0) = RotationMatrix::GetRotate_XYZ(toRad(rc.a), toRad(rc.b), toRad(rc.c)) * toolRot.inverse();
		Vector3d toolShift = t07.block<3, 3>(0, 0) * Vector3d(tool.x, tool.y, tool.z);
		t07.block<3, 1>(0, 3) = Vector3d(rc.x, rc.y, rc.z) - toolShift;
		std::cout << std::endl << "Inverse Kinematic" << std::endl;
		std::cout << "t07 = " << endl << t07 << std::endl;

		//Calculate position of joint 4
		Matrix4d t06 = t07 * getTransMat_Craig(dh_table[6]).inverse();
		std::cout << "t06 = " << endl << t06 << std::endl;
		Vector3d p4 = t06.block<3, 1>(0, 3);
		std::cout << "p4=p6= " << p4.x() << "," << p4.y() << "," << p4.z() << std::endl;
		
		//Solve J1 ~J3
		vector<JointCoordinate> jointSols;
		std::cout << std::endl << "All J1~J3 Solution" << std::endl;
		auto j3s_pi= solveJ3_method3(p4);
		for (int i = 0; i < j3s_pi.size(); i++) {
			auto j2s_pi = solveJ2(p4, j3s_pi[i]);
			for (int j = 0; j < j2s_pi.size(); j++) {
				double tmp2 = toDeg(j2s_pi[j]);
				auto j1s_pi = solveJ1(p4, j3s_pi[i], j2s_pi[j]);
				for (int k = 0; k < j1s_pi.size(); k++) {
					if (validJ4Position(toDeg(j1s_pi[k]), toDeg(j2s_pi[j]), toDeg(j3s_pi[i]), p4)) {
						std::cout << "Correct solution of (J1, J2, J3) = (" << toDeg(j1s_pi[k]) << "," << toDeg(j2s_pi[j]) << "," << toDeg(j3s_pi[i]) << ")" << std::endl;
						jointSols.push_back(JointCoordinate{ toDeg(j1s_pi[k]),toDeg(j2s_pi[j]),toDeg(j3s_pi[i]) });
					}
					else 
						std::cout << "Incorrect solution of (J1, J2, J3) = (" << toDeg(j1s_pi[k]) << "," << toDeg(j2s_pi[j]) << "," << toDeg(j3s_pi[i]) << ")" << std::endl;
				}
			}
		}

		for (int i = 0; i < jointSols.size(); i++) {
			vector<float> jointAngle = { jointSols[i].j1,jointSols[i].j2,jointSols[i].j3 };
			Matrix4d t03 = Matrix4d::Identity();
			for (int j = 0; j < 3; j++) {
				auto param = dh_table[j];
				t03 = t03 * getTransMat_Craig(DH_param{ param.alpha, param.a, param.d, jointAngle[j] });
			}
			
			Matrix3d r03 = t03.block<3, 3>(0, 0);
			Matrix3d r06 = t06.block<3, 3>(0, 0);
			Matrix3d r46 = (r03 * RotationMatrix::GetRotate_X(toRad(dh_table[3].alpha))).inverse() * r06;
			std::cout << "T03 = " << std::endl << t03 << std::endl;
			Vector3f eulerAngle = RotationMatrix::GetEulerAngle_ZYZ(r46);
			jointSols[i].j1 = round(jointSols[i].j1 - dh_table[0].theta, 2);
			jointSols[i].j2 = round(jointSols[i].j2 - dh_table[1].theta, 2);
			jointSols[i].j3 = round(jointSols[i].j3 - dh_table[2].theta, 2);
			jointSols[i].j4 = round(eulerAngle.x(), 2);
			jointSols[i].j5 = round(-eulerAngle.y(), 2);
			jointSols[i].j6 = round(eulerAngle.z(), 2);

			std::cout << "(J4,J5,J6) = (" << eulerAngle.x() << ", " << eulerAngle.y() << ", " << eulerAngle.z() << ")" << endl;
		}
		std::cout << "Final Inverse Kinematics Solutions" << std::endl;
		for (int i = 0; i < jointSols.size(); i++) {
			std::cout << "Sol" << i << " = (" << jointSols[i].j1 << ", " << jointSols[i].j2 << ", " << jointSols[i].j3 << ", " << jointSols[i].j4 << ", " << jointSols[i].j5 << ", " << jointSols[i].j6 << ")" << std::endl;
		}
		return JointCoordinate{};
	}
private:
	vector<DH_param> dh_table;
	vector<JointLimit> limits;
	EulerAngle eulerType;
	WorldCoordinate tool;
	double* k1;
	double* k2;
	double* k3;
	double* k4;

	/// <summary>
	/// Using Craig method to get the transformation matrix of DH paramteres.
	/// </summary>
	/// <param name="param">DH paramteres</param>
	/// <returns>Transformation matrix</returns>
	Eigen::Matrix4d getTransMat_Craig(DH_param param) {
		Eigen::AngleAxisd rx(toRad(param.alpha), Eigen::Vector3d(1, 0, 0));
		Eigen::AngleAxisd rz(toRad(param.theta), Eigen::Vector3d(0, 0, 1));
		Eigen::Matrix4d matx = Eigen::Matrix4d::Identity();
		matx.block<3, 3>(0, 0) = rx.matrix();
		Eigen::Matrix4d matz = Eigen::Matrix4d::Identity();
		matz.block<3, 3>(0, 0) = rz.matrix();
		Eigen::Matrix4d tx = Eigen::Matrix4d::Identity();
		tx.block<3, 1>(0, 3) = Eigen::Vector3d(param.a, 0, 0);
		Eigen::Matrix4d tz = Eigen::Matrix4d::Identity();
		tz.block<3, 1>(0, 3) = Eigen::Vector3d(0, 0, param.d);
		return matx * tx * matz * tz;
	}

#pragma region k functions
	///k functions are u*cos(th)+v*sin(th)+w
	double* get_coeffs_k1() {
		double* ptr = new double[3]{
			dh_table[3].a,
			dh_table[3].d * sin(toRad(dh_table[3].alpha)),
			dh_table[2].a
		};
		return ptr;
	}
	double* get_coeffs_k2() {
		double alpha2_pi = toRad(dh_table[2].alpha);
		double alpha3_pi = toRad(dh_table[3].alpha);
		double* ptr = new double[3]{
			dh_table[3].d * cos(alpha2_pi) * sin(alpha3_pi),
			-dh_table[3].a * cos(alpha2_pi),
			dh_table[2].d * sin(alpha2_pi) + dh_table[3].d * cos(alpha3_pi) * sin(alpha2_pi)
		};
		return ptr;
	}
	double* get_coeffs_k3() {
		double alpha2_pi = toRad(dh_table[2].alpha);
		double alpha3_pi = toRad(dh_table[3].alpha);
		double* ptr = new double[3]{
			2 * (dh_table[2].a * dh_table[3].a + dh_table[1].d * dh_table[3].d * sin(alpha2_pi) * sin(alpha3_pi)),
			2 * (dh_table[3].a * dh_table[1].d * sin(alpha2_pi) + dh_table[2].a * dh_table[3].d * sin(alpha3_pi)),
			pow(dh_table[1].a,2) + pow(dh_table[2].a,2) + pow(dh_table[3].a,2) +
			pow(dh_table[1].d,2) + pow(dh_table[2].d,2) + pow(dh_table[3].d,2) +
			2 * (dh_table[1].d * dh_table[2].d * cos(alpha2_pi) + dh_table[1].d * dh_table[3].d * cos(alpha2_pi) * cos(alpha3_pi) + dh_table[2].d * dh_table[3].d * cos(alpha3_pi))
		};
		return ptr;
	}
	double* get_coeffs_k4() {
		double alpha1_pi = toRad(dh_table[1].alpha);
		double alpha2_pi = toRad(dh_table[2].alpha);
		double alpha3_pi = toRad(dh_table[3].alpha);
		double* ptr = new double[3]{
			-dh_table[3].d * cos(alpha1_pi) * sin(alpha2_pi) * sin(alpha3_pi),
			dh_table[3].a * cos(alpha1_pi) * sin(alpha2_pi),
			dh_table[1].d * cos(alpha1_pi) + dh_table[2].d * cos(alpha1_pi) * cos(alpha2_pi) + dh_table[3].d * cos(alpha1_pi) * cos(alpha2_pi) * cos(alpha3_pi)
		};
		return ptr;
	}
#pragma endregion

	vector<float> solveJ3_method3(Vector3d p4) {
		std::vector<float> ansAry;
		float r = powf(p4.x(), 2) + powf(p4.y(), 2) + powf(p4.z(), 2);
		float alpha1_pi = toRad(dh_table[1].alpha);
		//formula = coef1 * x^4 + coef2 * x^3 + coef3 * x^2 + coef4 * x + coef5 = 0
		float coef1 = powf(r + k3[0] - k3[2], 2) - 4 * powf(dh_table[1].a, 2) * (powf(k1[0] - k1[2], 2) + powf(k2[0] - k2[2], 2)) +
			4 * powf(dh_table[1].a, 2) / powf(sin(alpha1_pi), 2) * powf(k4[0] - k4[2] + p4.z(), 2);
		float coef2 = 4 * k3[1] * (k3[2] - k3[0] - r) + 16 * powf(dh_table[1].a, 2) * (k1[1] * (k1[0] - k1[2]) + k2[1] * (k2[0] - k2[2])) -
			16 * powf(dh_table[1].a, 2) * k4[1] / powf(sin(alpha1_pi), 2) * (k4[0] - k4[2] + p4.z());
		float coef3 = 2 * powf(r - k3[2], 2) - 2 * powf(k3[0], 2) + 4 * powf(k3[1], 2) +
			8 * powf(dh_table[1].a, 2) * (powf(k1[0], 2) + powf(k2[0], 2) - 2 * (powf(k1[1], 2) + powf(k2[1], 2)) - powf(k1[2], 2) - powf(k2[2], 2)) +
			8 * powf(dh_table[1].a, 2) / powf(sin(alpha1_pi), 2) * (powf(k4[0], 2) + 2 * powf(k4[1], 2) + powf(k4[2] - p4.z(), 2));
		float coef4 = 4 * k3[1] * (k3[0] + k3[2] - r) - 16 * powf(dh_table[1].a, 2) * (k1[1] * (k1[0] + k1[2]) + k2[1] * (k2[0] + k2[2])) +
			16 * powf(dh_table[1].a, 2) / powf(sin(alpha1_pi), 2) * k4[1] * (k4[0] + k4[2] - p4.z());
		float coef5 = powf(r - k3[0] - k3[2], 2) - 4 * powf(dh_table[1].a, 2) * (powf(k1[0] + k1[2], 2) + powf(k2[0] + k2[2], 2)) +
			4 * powf(dh_table[1].a, 2) / powf(sin(alpha1_pi), 2) * powf(k4[0] + k4[2] - p4.z(), 2);
		/*std::cout << "j3 solving equation: " << std::endl;
		std::cout << coef1 << " X^4 + " << coef2 << " X^3 + " << coef3 << " X^2 + " << coef4 << " X + " << coef5 << " = 0" << std::endl;*/
		auto ans = Polynomial::solve_biquadratic_eq(coef1, coef2, coef3, coef4, coef5);
		vector<complex<double>> tt;
		for (int i = 0; i < 4; i++) {
			tt.push_back(ans[i]);
			if (abs(ans[i].imag()) <= 1E-2) {
				ansAry.push_back(atanf((float)ans[i].real()) * 2.0f);
			}
		}
		delete[] ans;
		return ansAry;
	}
	vector<float> solveJ2(Vector3d p4, float j3_pi) {
		vector<float> ansAry;
		float r = pow(p4.x(), 2) + pow(p4.y(), 2) + pow(p4.z(), 2);
		float cos3 = cos(j3_pi);
		float sin3 = sin(j3_pi);
		float coef1 = -r - 2 * dh_table[1].a * k1[2] + k3[2] + (k3[0] - 2 * dh_table[1].a * k1[0]) * cos3 + (k3[1] -
			2 * dh_table[1].a * k1[1]) * sin3;
		float coef2 = 4 * dh_table[1].a * (k2[2] + k2[0] * cos3 + k2[1] * sin3);
		float coef3= -r + 2 * dh_table[1].a * k1[2] + k3[2] + (k3[0] + 2 * dh_table[1].a * k1[0]) * cos3 + (k3[1] +
			2 * dh_table[1].a * k1[1]) * sin3;
		
		/*std::cout << "j2 solving equation: " << std::endl;
		std::cout << coef1 << " X^2 + " << coef2 << " X + " << coef3 << " = 0" << std::endl;*/
		auto ans = Polynomial::solve_quadratic_eq(coef1, coef2, coef3);
		for (int i = 0; i < 2; i++) {
			if (abs(ans[i].imag()) <= 1E-2) {
				float j2_pi = atanf((float)ans[i].real()) * 2;
				ansAry.push_back(j2_pi);
			}
		}
		delete[] ans;
		return ansAry;
	}
	vector<float> solveJ1(Vector3d p4, float j3_pi, float j2_pi) {
		vector<float> ansAry;
		float cos3 = cos(j3_pi); float sin3 = sin(j3_pi);
		float alpha1 = toRad(dh_table[1].alpha);
		float alpha2 = toRad(dh_table[2].alpha);
		float alpha3 = toRad(dh_table[3].alpha);
		float f1 = dh_table[2].a + dh_table[3].a * cos3 + dh_table[3].d * sin(alpha3) * sin3;
		float f2 = -dh_table[2].d * sin(alpha2) - dh_table[3].d * cos(alpha3) * sin(alpha2) - dh_table[3].d *
			cos(alpha2) * cos3 * sin(alpha3) + dh_table[3].a * cos(alpha2) * sin3;
		float f3 = dh_table[2].d * cos(alpha2) + dh_table[3].d * (cos(alpha2) * cos(alpha3) - cos3 * sin(alpha2) * sin(alpha3)) + 
			dh_table[3].a * sin(alpha2) * sin3;

		float cos2 = cos(j2_pi); float sin2 = sin(j2_pi);
		float g1 = cos2 * f1 - sin2 * f2 + dh_table[1].a;
		float g2 = sin2 * cos(alpha1) * f1 + cos2 * cos(alpha1) * f2 - sin(alpha1) * f3 - dh_table[1].d * sin(alpha1);
		float coef1 = -g1 - p4.x();
		float coef2 = -2* g2;
		float coef3= g1 - p4.x();

		/*std::cout << "j1 solving equation: " << std::endl;
		std::cout << coef1 << " X^2 + " << coef2 << " X + " << coef3 << " = 0" << std::endl;*/
		auto ans = Polynomial::solve_quadratic_eq(coef1, coef2, coef3);
		for (int i = 0; i < 2; i++) {
			if (abs(ans[i].imag()) <= 1E-2) {
				float j1_pi = atanf((float)ans[i].real()) * 2;
				ansAry.push_back(j1_pi);
			}
		}
		delete[] ans;
		return ansAry;
	}

	/// <summary>
	/// Check the position of joint 4 that calculated by argument of j1,j2,j3 is match to argument of P4 or not.
	/// </summary>
	/// <param name="j1">degree value of j1</param>
	/// <param name="j2">degree value of j2</param>
	/// <param name="j3">degree value of j3</param>
	/// <param name="p4">position of joint 4</param>
	/// <returns>True means the j1,j2,j3 value are one of the solution of inverse kinematic.</returns>
	bool validJ4Position(float j1, float j2, float j3, Vector3d p4) {
		vector<float> jointAngle = { j1,j2,j3 };
		Matrix4d t03 = Matrix4d::Identity();
		for (int j = 0; j < 3; j++) {
			auto param = dh_table[j];
			t03 = t03 * getTransMat_Craig(DH_param{ param.alpha, param.a, param.d, jointAngle[j] });
		}
		auto t04 = t03 * getTransMat_Craig(dh_table[3]);

		Vector3d tp = t04.block<3, 1>(0, 3);
		auto cd1 = abs(round(tp.x(), 3) - round(p4.x(), 3));
		auto cd2 = abs(round(tp.y(), 3) - round(p4.y(), 3));
		auto cd3 = abs(round(tp.z(), 3) - round(p4.z(), 3));
		return cd1 < 3E-2 && cd2 < 3E-2 && cd3 < 3E-2;
	}
};

