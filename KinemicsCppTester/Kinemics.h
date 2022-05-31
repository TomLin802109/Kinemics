#pragma once
#include "Matrix.h"
#include "PolynomialRoot.h"
#include <iostream>
#include <vector>

using namespace QrMatrix;
using namespace std;

class RobotArmKinemics {
public:
	RobotArmKinemics(RobotSpec spec, WorldCoordinate tool = WorldCoordinate{0,0,0,0,0,0}) {
		this->dh_table = spec.dh_table;
		this->limits = spec.limits;
		this->tool = tool;
		k1 = get_coeffs_k1();
		k2 = get_coeffs_k2();
		k3 = get_coeffs_k3();
		k4 = get_coeffs_k4();
		//std::cout << k1[0] << "," << k1[1] << "," << k1[2] << std::endl;
		//std::cout << k2[0] << "," << k2[1] << "," << k2[2] << std::endl;
		//std::cout << k3[0] << "," << k3[1] << "," << k3[2] << std::endl;
		//std::cout << k4[0] << "," << k4[1] << "," << k4[2] << std::endl;
	}
	~RobotArmKinemics() {
		delete[] k1, k2, k3, k4;
	}
	WorldCoordinate Forward(JointCoordinate jc) {
		float crd[7]{ jc.j1,jc.j2 ,jc.j3 ,jc.j4 ,jc.j5 ,jc.j6 };
		Matrix4f mat{
			1,0,0,0,
			0,1,0,0,
			0,0,1,0,
			0,0,0,1
		};
		for (int i = 0; i < dh_table.size(); i++) {
			auto param = dh_table[i];
			mat = Multiply(mat, getTransformMatrix(DH_param{ param.alpha, param.a, param.d, crd[i] + param.theta }));
			std::cout<<"No." << i+1 << " x " << mat.r03 << " y " << mat.r13 << " z " << mat.r23 << std::endl;
			/*std::cout << "tool x: " << mat.r00 << "," << mat.r10 << "," << mat.r20 << std::endl;
			std::cout << "tool y: " << mat.r01 << "," << mat.r11 << "," << mat.r21 << std::endl;
			std::cout << "tool z: " << mat.r02 << "," << mat.r12 << "," << mat.r22 << std::endl;
			std::cout << std::endl;*/
		}
		auto shift = Multiply(mat, Vector4f{ tool.x,tool.y,tool.z });
		Matrix4f tool_rot = getRotate_xyz(toRad(tool.a), toRad(tool.b), toRad(tool.c));
		Vector3f angle = getEulerAngle_xyz(Multiply(mat, tool_rot));
		return WorldCoordinate{ mat.r03 + shift.x,mat.r13 + shift.y,mat.r23 + shift.z, angle.x,angle.y,angle.z };
		//return WorldCoordinate{ mat.r03 ,mat.r13,mat.r23 };
	};
	JointCoordinate Inverse(WorldCoordinate rc, PostureCfg cfg = PostureCfg{}) {

		Matrix4f mat = getRotate_xyz(toRad(rc.a), toRad(rc.b), toRad(rc.c));
		mat.r03 = rc.x; mat.r13 = rc.y; mat.r23 = rc.z;
		mat = Multiply(mat, QrMatrix::Inverse(getRotate_xyz(toRad(tool.a), toRad(tool.b), toRad(tool.c))));
		mat = Multiply(mat, QrMatrix::Inverse(getTransformMatrix(dh_table[6])));
		Vector3f p4{ mat.r03 ,mat.r13,mat.r23 };
		
		std::cout << "p4=p6= " << mat.r03 << "," << mat.r13 << "," << mat.r23 << std::endl;
		std::cout << "tool x: " << mat.r00 << "," << mat.r10 << "," << mat.r20 << std::endl;
		std::cout << "tool y: " << mat.r01 << "," << mat.r11 << "," << mat.r21 << std::endl;
		std::cout << "tool z: " << mat.r02 << "," << mat.r12 << "," << mat.r22 << std::endl;
		//Vector3f p4{ rc.x,rc.y,rc.z };
		std::cout << "p4=p6= " << p4.x << "," << p4.y << "," << p4.z << std::endl;
		vector<JointCoordinate> jointSols;
		vector<Vector3f> pos;
		
		auto j3s_pi= solveJ3_method3(p4);
		for (int i = 0; i < j3s_pi.size(); i++) {
			std::cout << "j3 solution:" << toDeg(j3s_pi[i]) << std::endl;
			auto j2s_pi = solveJ2(p4, j3s_pi[i]);
			for (int j = 0; j < j2s_pi.size(); j++) {
				auto j1s_pi = solveJ1(p4, j3s_pi[i], j2s_pi[j]);
				for (int k = 0; k < j1s_pi.size(); k++) {
					std::cout << "j3,j2,j1: (" << toDeg(j3s_pi[i]) << "," << toDeg(j2s_pi[j]) << "," << toDeg(j1s_pi[k]) << ")" << std::endl;
					jointSols.push_back(JointCoordinate{ toDeg(j1s_pi[k]),toDeg(j2s_pi[j]),toDeg(j3s_pi[i]) });
				}
			}
			/*float j1_pi = solveJ1(p4, j3s_pi[i], j2_pi);
			std::cout << "j2:" << toDeg(j2_pi) << std::endl;
			std::cout << "j1:" << toDeg(j1_pi) << std::endl;*/
		}

		vector<Matrix4f> rots;
		for (int i = 0; i < jointSols.size(); i++) {
			vector<float> jointAngle = { jointSols[i].j1,jointSols[i].j2 ,jointSols[i].j3 };
			Matrix4f t03{
			1,0,0,0,
			0,1,0,0,
			0,0,1,0,
			0,0,0,1
			};
			for (int j = 0; j <3; j++) {
				auto param = dh_table[j];
				t03 = Multiply(t03, getTransformMatrix(DH_param{ param.alpha, param.a, param.d, jointAngle[j] }));
				/*if(j==2)
					std::cout << "No." << j + 1 << " x " << t03.r03 << " y " << t03.r13 << " z " << t03.r23 << std::endl;*/
			}
			auto t04 = Multiply(t03, getTransformMatrix(dh_table[3]));
			
			Vector3f tp{ t04.r03 ,t04.r13,t04.r23 };
			auto cd1 = abs(round(tp.x, 3) - round(p4.x, 3));
			auto cd2 = abs(round(tp.y, 3) - round(p4.y, 3));
			auto cd3 = abs(round(tp.z, 3) - round(p4.z, 3));
			std::cout << "tp= " << t04.r03 << "," << t04.r13 << "," << t04.r23 << std::endl;
			if ( cd1 < 3E-2 &&cd2 < 3E-2 &&cd3 < 3E-2) {
				
				std::cout << "match sol(j1,j2,j3)= " << jointAngle[0]-dh_table[0].theta << "," << jointAngle[1] - dh_table[1].theta << "," << jointAngle[2] - dh_table[2].theta << std::endl;
				rots.push_back(t03);
			}
		}
		auto t06 = getRotate_xyz(toRad(rc.a), toRad(rc.b), toRad(rc.c));
		auto tmpx = getRotate_x(toRad(rc.a));
		auto tmpy = getRotate_y(toRad(rc.b));
		auto tmpz = getRotate_z(toRad(rc.c));
		auto ttt = Multiply(Multiply(tmpx, tmpy), tmpz);
		for (int i = 0; i < rots.size(); i++) {
			rots[i] = Multiply(QrMatrix::Inverse(rots[i]), t06);
			auto eulerAngle = getEulerAngle_zyz(rots[i]);
			cout << "(j4,j5,j6)= (" << eulerAngle.x << "," << eulerAngle.y << "," << eulerAngle.z << ")" << endl;
		}

		
		return JointCoordinate{};
	}
private:
	vector<DH_param> dh_table;
	vector<JointLimit> limits;
	WorldCoordinate tool;
	float* k1;
	float* k2;
	float* k3;
	float* k4;
	Matrix4f getTransformMatrix(DH_param param) {
		Matrix4f rx = getRotate_x(toRad(param.alpha));
		Matrix4f rz = getRotate_z(toRad(param.theta));
		Matrix4f tx = Matrix4f{
			1,0,0,param.a,
			0,1,0,0,
			0,0,1,0,
			0,0,0,1
		};
		Matrix4f tz = Matrix4f{
			1,0,0,0,
			0,1,0,0,
			0,0,1,param.d,
			0,0,0,1
		};
		return Multiply(Multiply(rx, tx), Multiply(rz, tz));
	}
#pragma region k functions
	///k functions are u*cos(th)+v*sin(th)+w
	float* get_coeffs_k1() {
		float* ptr = new float[3]{
			dh_table[3].a,
			dh_table[3].d * sinf(toRad(dh_table[3].alpha)),
			dh_table[2].a
		};
		return ptr;
	}
	float* get_coeffs_k2() {
		float alpha2_pi = toRad(dh_table[2].alpha);
		float alpha3_pi = toRad(dh_table[3].alpha);
		float* ptr = new float[3]{
			dh_table[3].d * cosf(alpha2_pi) * sinf(alpha3_pi),
			-dh_table[3].a * cosf(alpha2_pi),
			dh_table[2].d * sinf(alpha2_pi) + dh_table[3].d * cosf(alpha3_pi) * sinf(alpha2_pi)
		};
		return ptr;
	}
	float* get_coeffs_k3() {
		float alpha2_pi = toRad(dh_table[2].alpha);
		float alpha3_pi = toRad(dh_table[3].alpha);
		float* ptr = new float[3]{
			2 * (dh_table[2].a * dh_table[3].a + dh_table[1].d * dh_table[3].d * sinf(alpha2_pi) * sinf(alpha3_pi)),
			2 * (dh_table[3].a * dh_table[1].d * sinf(alpha2_pi) + dh_table[2].a * dh_table[3].d * sinf(alpha3_pi)),
			powf(dh_table[1].a,2) + powf(dh_table[2].a,2) + powf(dh_table[3].a,2) +
			powf(dh_table[1].d,2) + powf(dh_table[2].d,2) + powf(dh_table[3].d,2) +
			2 * (dh_table[1].d * dh_table[2].d * cosf(alpha2_pi) + dh_table[1].d * dh_table[3].d * cosf(alpha2_pi) * cosf(alpha3_pi) + dh_table[2].d * dh_table[3].d * cosf(alpha3_pi))
		};
		return ptr;
	}
	float* get_coeffs_k4() {
		float alpha1_pi = toRad(dh_table[1].alpha);
		float alpha2_pi = toRad(dh_table[2].alpha);
		float alpha3_pi = toRad(dh_table[3].alpha);
		float* ptr = new float[3]{
			-dh_table[3].d * cosf(alpha1_pi) * sinf(alpha2_pi) * sinf(alpha3_pi),
			dh_table[3].a * cosf(alpha1_pi) * sinf(alpha2_pi),
			dh_table[1].d * cosf(alpha1_pi) + dh_table[2].d * cosf(alpha1_pi) * cosf(alpha2_pi) + dh_table[3].d * cosf(alpha1_pi) * cosf(alpha2_pi) * cosf(alpha3_pi)
		};
		return ptr;
	}
#pragma endregion

	std::vector<float> solveJ3_method3(Vector3f p4) {
		std::vector<float> ansAry;
		float r = powf(p4.x, 2) + powf(p4.y, 2) + powf(p4.z, 2);
		float alpha1_pi = toRad(dh_table[1].alpha);
		float coef1 = powf(r + k3[0] - k3[2], 2) - 4 * powf(dh_table[1].a, 2) * (powf(k1[0] - k1[2], 2) + powf(k2[0] - k2[2], 2)) +
			4 * powf(dh_table[1].a, 2) / powf(sinf(alpha1_pi), 2) * powf(k4[0] - k4[2] + p4.z, 2);
		float coef2 = 4 * k3[1] * (k3[2] - k3[0] - r) + 16 * powf(dh_table[1].a, 2) * (k1[1] * (k1[0] - k1[2]) + k2[1] * (k2[0] - k2[2])) -
			16 * powf(dh_table[1].a, 2) * k4[1] / powf(sinf(alpha1_pi), 2) * (k4[0] - k4[2] + p4.z);
		float coef3 = 2 * powf(r - k3[2], 2) - 2 * powf(k3[0], 2) + 4 * powf(k3[1], 2) +
			8 * powf(dh_table[1].a, 2) * (powf(k1[0], 2) + powf(k2[0], 2) - 2 * (powf(k1[1], 2) + powf(k2[1], 2)) - powf(k1[2], 2) - powf(k2[2], 2)) +
			8 * powf(dh_table[1].a, 2) / powf(sinf(alpha1_pi), 2) * (powf(k4[0], 2) + 2 * powf(k4[1], 2) + powf(k4[2] - p4.z, 2));
		float coef4 = 4 * k3[1] * (k3[0] + k3[2] - r) - 16 * powf(dh_table[1].a, 2) * (k1[1] * (k1[0] + k1[2]) + k2[1] * (k2[0] + k2[2])) +
			16 * powf(dh_table[1].a, 2) / powf(sinf(alpha1_pi), 2) * k4[1] * (k4[0] + k4[2] - p4.z);
		float coef5 = powf(r - k3[0] - k3[2], 2) - 4 * powf(dh_table[1].a, 2) * (powf(k1[0] + k1[2], 2) + powf(k2[0] + k2[2], 2)) +
			4 * powf(dh_table[1].a, 2) / powf(sinf(alpha1_pi), 2) * powf(k4[0] + k4[2] - p4.z, 2);
		/*std::cout << "solve j3 coeff:" << std::endl;
		std::cout << coef1 << std::endl;
		std::cout << coef2 << std::endl;
		std::cout << coef3 << std::endl;
		std::cout << coef4 << std::endl;
		std::cout << coef5 << std::endl;*/
		auto ans = Polynomial::solve_quartic_eq(coef1, coef2, coef3, coef4, coef5);
		vector<complex<double>> tt;
		for (int i = 0; i < 4; i++) {
			tt.push_back(ans[i]);
			if (abs(ans[i].imag()) <= 1E-6) {
				ansAry.push_back(atanf((float)ans[i].real()) * 2.0f);
			}
		}
		delete[] ans;
		return ansAry;
	}

	vector<float> solveJ2(Vector3f p4, float j3_pi) {
		vector<float> ansAry;
		float r = powf(p4.x, 2) + powf(p4.y, 2) + powf(p4.z, 2);
		float cos3 = cosf(j3_pi);
		float sin3 = sinf(j3_pi);
		float coef1 = -r - 2 * dh_table[1].a * k1[2] + k3[2] + (k3[0] - 2 * dh_table[1].a * k1[0]) * cos3 + (k3[1] -
			2 * dh_table[1].a * k1[1]) * sin3;
		float coef2 = 4 * dh_table[1].a * (k2[2] + k2[0] * cos3 + k2[1] * sin3);
		float coef3= -r + 2 * dh_table[1].a * k1[2] + k3[2] + (k3[0] + 2 * dh_table[1].a * k1[0]) * cos3 + (k3[1] +
			2 * dh_table[1].a * k1[1]) * sin3;
		/*std::cout << "solve j2 coeff:" << std::endl;
		std::cout << coef1 << std::endl;
		std::cout << coef2 << std::endl;
		std::cout << coef3 << std::endl;*/
		auto ans = Polynomial::solve_quadratic_eq(coef1, coef2, coef3);
		for (int i = 0; i < 2; i++) {
			if (abs(ans[i].imag()) <= 1E-6) {
				float j2_pi = atanf((float)ans[i].real()) * 2;
				ansAry.push_back(j2_pi);
				//std::cout << "j2: " << toDeg(j2_pi) << std::endl;
				//solveJ1(p4, j3_pi, j2_pi);
			}
			
		}
		delete[] ans;
		return ansAry;
	}
	vector<float> solveJ1(Vector3f p4, float j3_pi, float j2_pi) {
		vector<float> ansAry;
		float cos3 = cosf(j3_pi); float sin3 = sinf(j3_pi);
		float alpha1 = toRad(dh_table[1].alpha);
		float alpha2 = toRad(dh_table[2].alpha);
		float alpha3 = toRad(dh_table[3].alpha);
		float f1 = dh_table[2].a + dh_table[3].a * cos3 + dh_table[3].d * sinf(alpha3) * sin3;
		float f2 = -dh_table[2].d * sinf(alpha2) - dh_table[3].d * cosf(alpha3) * sinf(alpha2) - dh_table[3].d *
			cosf(alpha2) * cos3 * sinf(alpha3) + dh_table[3].a * cosf(alpha2) * sin3;
		float f3 = dh_table[2].d * cosf(alpha2) + dh_table[3].d * (cosf(alpha2) * cosf(alpha3) - cos3 * sinf(alpha2) * sinf(alpha3)) + 
			dh_table[3].a * sinf(alpha2) * sin3;

		float cos2 = cosf(j2_pi); float sin2 = sinf(j2_pi);
		float g1 = cos2 * f1 - sin2 * f2 + dh_table[1].a;
		float g2 = sin2 * cosf(alpha1) * f1 + cos2 * cosf(alpha1) * f2 - sinf(alpha1) * f3 - dh_table[1].d * sinf(alpha1);
		float coef1 = -g1 - p4.x;
		float coef2 = -2* g2;
		float coef3= g1 - p4.x;
		/*std::cout << "solve j1 coeff:" << std::endl;
		std::cout << coef1 << std::endl;
		std::cout << coef2 << std::endl;
		std::cout << coef3 << std::endl;*/
		auto ans = Polynomial::solve_quadratic_eq(coef1, coef2, coef3);
		for (int i = 0; i < 2; i++) {
			if (abs(ans[i].imag()) <= 1E-6) {
				float j1_pi = atanf((float)ans[i].real()) * 2;
				ansAry.push_back(j1_pi);
			}
			//std::cout << "j1: " << toDeg(j1_pi) << std::endl;
		}
		delete[] ans;
		return ansAry;
	}
};

