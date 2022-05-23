#pragma once
#include "Matrix.h"
#include "PolynomialRoot.h"
#include <iostream>
#include <vector>

using namespace QrMatrix;

class Kinemics {
public:
	Kinemics(DH_table table, WorldCoordinate tool = WorldCoordinate{0,0,0,0,0,0}) {
		this->table = table;
		this->tool = tool;
		k1 = get_coeffs_k1();
		k2 = get_coeffs_k2();
		k3 = get_coeffs_k3();
		k4 = get_coeffs_k4();
		std::cout << k1[0] << "," << k1[1] << "," << k1[2] << std::endl;
		std::cout << k2[0] << "," << k2[1] << "," << k2[2] << std::endl;
		std::cout << k3[0] << "," << k3[1] << "," << k3[2] << std::endl;
		std::cout << k4[0] << "," << k4[1] << "," << k4[2] << std::endl;
	}
	~Kinemics() {
		delete[] k1, k2, k3, k4;
	}
	WorldCoordinate Forward(JointCoordinate jc) {
		float crd[7]{ jc.j1,jc.j2 ,jc.j3 ,jc.j4 ,jc.j5 ,jc.j6, 0 };
		Matrix4f mat{
			1,0,0,0,
			0,1,0,0,
			0,0,1,0,
			0,0,0,1
		};
		for (int i = 0; i < table.size; i++) {
			auto param = table.params[i];
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
	};
	JointCoordinate Inverse(WorldCoordinate rc, PostureCfg cfg = PostureCfg{}) {
		Matrix4f mat = getRotate_xyz(toRad(rc.a), toRad(rc.b), toRad(rc.c));
		mat.r03 = rc.x; mat.r13 = rc.y; mat.r23 = rc.z;
		mat = Multiply(mat, QrMatrix::Inverse(getRotate_xyz(toRad(tool.a), toRad(tool.b), toRad(tool.c))));
		mat = Multiply(mat, QrMatrix::Inverse(getTransformMatrix(table.params[6])));
		Vector3f p4{ mat.r03 ,mat.r13,mat.r23 };
		
		std::cout << "p4=p6= " << mat.r03 << "," << mat.r13 << "," << mat.r23 << std::endl;
		std::cout << "tool x: " << mat.r00 << "," << mat.r10 << "," << mat.r20 << std::endl;
		std::cout << "tool y: " << mat.r01 << "," << mat.r11 << "," << mat.r21 << std::endl;
		std::cout << "tool z: " << mat.r02 << "," << mat.r12 << "," << mat.r22 << std::endl;
		auto j3s_pi= solveJ3_method3(p4);
		for (int i = 0; i < 4; i++) {
			std::cout << "j3 solution:" << toDeg(j3s_pi[i]) << std::endl;
		}
		auto j2s_pi = solveJ2(p4, j3s_pi[1]);
		for (int i = 0; i < 2; i++) {
			std::cout << "j2 solution:" << toDeg(j2s_pi[i]) << std::endl;
		}
		auto j1s_pi = solveJ1(p4, j3s_pi[1],j2s_pi[1]);
		for (int i = 0; i < 2; i++) {
			std::cout << "j1 solution:" << toDeg(j1s_pi[i]) << std::endl;
		}
		return JointCoordinate{};
	}
private:
	DH_table table;
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
			table.params[3].a,
			table.params[3].d * sinf(toRad(table.params[3].alpha)),
			table.params[2].a
		};
		return ptr;
	}
	float* get_coeffs_k2() {
		float alpha2_pi = toRad(table.params[2].alpha);
		float alpha3_pi = toRad(table.params[3].alpha);
		float* ptr = new float[3]{
			table.params[3].d * cosf(alpha2_pi) * sinf(alpha3_pi),
			-table.params[3].a * cosf(alpha2_pi),
			table.params[2].d * sinf(alpha2_pi) + table.params[3].d * cosf(alpha3_pi) * sinf(alpha2_pi)
		};
		return ptr;
	}
	float* get_coeffs_k3() {
		float alpha2_pi = toRad(table.params[2].alpha);
		float alpha3_pi = toRad(table.params[3].alpha);
		float* ptr = new float[3]{
			2 * (table.params[2].a * table.params[3].a + table.params[1].d * table.params[3].d * sinf(alpha2_pi) * sinf(alpha3_pi)),
			2 * (table.params[3].a * table.params[1].d * sinf(alpha2_pi) + table.params[2].a * table.params[3].d * sinf(alpha3_pi)),
			powf(table.params[1].a,2) + powf(table.params[2].a,2) + powf(table.params[3].a,2) +
			powf(table.params[1].d,2) + powf(table.params[2].d,2) + powf(table.params[3].d,2) +
			2 * (table.params[1].d * table.params[2].d * cosf(alpha2_pi) + table.params[1].d * table.params[3].d * cosf(alpha2_pi) * cosf(alpha3_pi) + table.params[2].d * table.params[3].d * cosf(alpha3_pi))
		};
		return ptr;
	}
	float* get_coeffs_k4() {
		float alpha1_pi = toRad(table.params[1].alpha);
		float alpha2_pi = toRad(table.params[2].alpha);
		float alpha3_pi = toRad(table.params[3].alpha);
		float* ptr = new float[3]{
			-table.params[3].d * cosf(alpha1_pi) * sinf(alpha2_pi) * sinf(alpha3_pi),
			table.params[3].a * cosf(alpha1_pi) * sinf(alpha2_pi),
			table.params[1].d * cosf(alpha1_pi) + table.params[2].d * cosf(alpha1_pi) * cosf(alpha2_pi) + table.params[3].d * cosf(alpha1_pi) * cosf(alpha2_pi) * cosf(alpha3_pi)
		};
		return ptr;
	}
#pragma endregion

	double* solveJ3_method3(Vector3f p4) {
		double* ansAry = new double[4];
		double r = pow(p4.x, 2) + pow(p4.y, 2) + pow(p4.z, 2);
		double alpha1_pi = toRad(table.params[1].alpha);
		double coef1 = pow(r + k3[0] - k3[2], 2) - 4.0 * pow(table.params[1].a, 2) * (pow(k1[0] - k1[2], 2) + pow(k2[0] - k2[2], 2)) +
			4.0 * pow(table.params[1].a, 2) / pow(sin(alpha1_pi), 2) * pow(k4[0] - k4[2] + p4.z, 2);
		double coef2 = 4 * k3[1] * (k3[2] - k3[0] - r) + 16 * pow(table.params[1].a, 2) * (k1[1] * (k1[0] - k1[2]) + k2[1] * (k2[0] - k2[2])) -
			16 * pow(table.params[1].a, 2) * k4[1] / pow(sin(alpha1_pi), 2) * (k4[0] - k4[2] + p4.z);
		double coef3 = 2 * pow(r - k3[2], 2) - 2 * pow(k3[0], 2) + 4 * pow(k3[1], 2) +
			8 * pow(table.params[1].a, 2) * (pow(k1[0], 2) + pow(k2[0], 2) - 2 * (pow(k1[1], 2) + pow(k2[1], 2)) - pow(k1[2], 2) - pow(k2[2], 2)) +
			8 * pow(table.params[1].a, 2) / pow(sin(alpha1_pi), 2) * (pow(k4[0], 2) + 2 * pow(k4[1], 2) + pow(k4[2] - p4.z, 2));
		double coef4 = 4 * k3[1] * (k3[0] + k3[2] - r) - 16 * pow(table.params[1].a, 2) * (k1[1] * (k1[0] + k1[2]) + k2[1] * (k2[0] + k2[2])) +
			16 * pow(table.params[1].a, 2) / pow(sin(alpha1_pi), 2) * k4[1] * (k4[0] + k4[2] - p4.z);
		double coef5 = pow(r - k3[0] - k3[2], 2) - 4 * pow(table.params[1].a, 2) * (pow(k1[0] + k1[2], 2) + pow(k2[0] + k2[2], 2)) +
			4 * pow(table.params[1].a, 2) / pow(sin(alpha1_pi), 2) * pow(k4[0] + k4[2] - p4.z, 2);
		std::cout << "solve j3 coeff:" << std::endl;
		std::cout << coef1 << std::endl;
		std::cout << coef2 << std::endl;
		std::cout << coef3 << std::endl;
		std::cout << coef4 << std::endl;
		std::cout << coef5 << std::endl;
			
		auto ans = Polynomial::solve_quartic_eq(coef1, coef2, coef3, coef4, coef5);
		for (int i = 0; i < 4; i++) {
			ansAry[i] = atan(ans[i].real()) * 2.0;
		}
		delete[] ans;
		return ansAry;
	}

	float* solveJ2(Vector3f p4, float j3_pi) {
		float* ansAry = new float[2];
		float r = pow(p4.x, 2) + pow(p4.y, 2) + pow(p4.z, 2);
		//j3_pi = 0.0432256;
		float cos3 = cos(j3_pi);
		float sin3 = sin(j3_pi);
		float coef1 = -r - 2 * table.params[1].a * k1[2] + k3[2] + (k3[0] - 2 * table.params[1].a * k1[0]) * cos3 + (k3[1] -
			2 * table.params[1].a * k1[1]) * sin3;
		float coef2 = 4 * table.params[1].a * (k2[2] + k2[0] * cos3 + k2[1] * sin3);
		float coef3= -r + 2 * table.params[1].a * k1[2] + k3[2] + (k3[0] + 2 * table.params[1].a * k1[0]) * cos3 + (k3[1] +
			2 * table.params[1].a * k1[1]) * sin3;
		std::cout << "solve j2 coeff:" << std::endl;
		std::cout << coef1 << std::endl;
		std::cout << coef2 << std::endl;
		std::cout << coef3 << std::endl;
		auto ans = Polynomial::solve_quadratic_eq(coef1, coef2, coef3);
		//auto ans = Polynomial::solve_quadratic_eq(43550.2, 40314.7, 9298.45);
		for (int i = 0; i < 2; i++) {
			ansAry[i] = atanf(ans[i].real()) * 2;
		}
		delete[] ans;
		return ansAry;
	}
	float* solveJ1(Vector3f p4, float j3_pi, float j2_pi) {
		float* ansAry = new float[2];
		
		float cos3 = cosf(j3_pi); float sin3 = sinf(j3_pi);
		double alpha1 = toRad((double)table.params[1].alpha);
		double alpha2 = toRad((double)table.params[2].alpha);
		double alpha3 = toRad((double)table.params[3].alpha);
		float f1 = table.params[2].a + table.params[3].a * cos3 + table.params[3].d * sinf(alpha3) * sin3;
		float f2 = -table.params[2].d * sinf(alpha2) - table.params[3].d * cosf(alpha3) * sinf(alpha2) - table.params[3].d *
			cosf(alpha2) * cos3 * sinf(alpha3) + table.params[3].a * cosf(alpha2) * sin3;
		float f3 = table.params[2].d * cosf(alpha2) + table.params[3].d * (cosf(alpha2) * cosf(alpha3) - cos3 * sinf(alpha2) * sinf(alpha3)) + 
			table.params[3].a * sinf(alpha2) * sin3;

		float cos2 = cosf(j2_pi); float sin2 = sinf(j2_pi);
		float g1 = cos2 * f1 - sin2 * f2 + table.params[1].a;
		double g2 = sin2 * cos(alpha1) * f1 + cos2 * cos(alpha1) * f2 - sin(alpha1) * f3 - table.params[1].d * sin(alpha1);
		float coef1 = -g1 - p4.x;
		float coef2 = -2* g2;
		float coef3= g1 - p4.x;
		std::cout << "solve j1 coeff:" << std::endl;
		std::cout << coef1 << std::endl;
		std::cout << coef2 << std::endl;
		std::cout << coef3 << std::endl;
		auto ans = Polynomial::solve_quadratic_eq(coef1, coef2, coef3);
		for (int i = 0; i < 2; i++) {
			ansAry[i] = atanf(ans[i].real()) * 2;
		}
		delete[] ans;
		return ansAry;
	}
};

