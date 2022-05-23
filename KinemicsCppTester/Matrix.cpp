#pragma once
#include "Matrix.h"
namespace QrMatrix {
	Matrix4f getRotate_x(float angle_pi) {
		return Matrix4f{
			1,0,0,0,
			0,cosf(angle_pi),-sinf(angle_pi),0,
			0,sinf(angle_pi),cosf(angle_pi),0,
			0,0,0,1
		};
	}
	Matrix4f getRotate_y(float angle_pi) {
		return Matrix4f{
			cosf(angle_pi),0,sinf(angle_pi),0,
			0,1,0,0,
			-sinf(angle_pi),0,cosf(angle_pi),0,
			0,0,0,1
		};
	}
	Matrix4f getRotate_z(float angle_pi) {
		return Matrix4f{
			cosf(angle_pi),-sinf(angle_pi),0,0,
			sinf(angle_pi),cosf(angle_pi),0,0,
			0,0,1,0,
			0,0,0,1
		};
	}
	Matrix4f getRotate_xyz(float rx_pi, float ry_pi, float rz_pi) {
		auto rx = getRotate_x(rx_pi);
		auto ry = getRotate_y(ry_pi);
		auto rz = getRotate_y(rz_pi);
		return Multiply(Multiply(rx, ry), rz);
	}
	Matrix4f getRotate_zyx(float rz_pi, float ry_pi, float rx_pi) {
		auto rz = getRotate_y(rz_pi);
		auto ry = getRotate_y(ry_pi);
		auto rx = getRotate_x(rx_pi);
		return Multiply(Multiply(rz, ry), rx);
	}
	Matrix4f getRotate_zyz(float rz1_pi, float ry_pi, float rz2_pi) {
		auto rz1 = getRotate_y(rz1_pi);
		auto ry = getRotate_y(ry_pi);
		auto rz2 = getRotate_x(rz2_pi);
		return Multiply(Multiply(rz1, ry), rz2);
	}
	Matrix4f Multiply(Matrix4f A, Matrix4f B) {
		return Matrix4f{
			A.r00 * B.r00 + A.r01 * B.r10 + A.r02 * B.r20 + A.r03 * B.r30,
			A.r00 * B.r01 + A.r01 * B.r11 + A.r02 * B.r21 + A.r03 * B.r31,
			A.r00 * B.r02 + A.r01 * B.r12 + A.r02 * B.r22 + A.r03 * B.r32,
			A.r00 * B.r03 + A.r01 * B.r13 + A.r02 * B.r23 + A.r03 * B.r33,
			A.r10 * B.r00 + A.r11 * B.r10 + A.r12 * B.r20 + A.r13 * B.r30,
			A.r10 * B.r01 + A.r11 * B.r11 + A.r12 * B.r21 + A.r13 * B.r31,
			A.r10 * B.r02 + A.r11 * B.r12 + A.r12 * B.r22 + A.r13 * B.r32,
			A.r10 * B.r03 + A.r11 * B.r13 + A.r12 * B.r23 + A.r13 * B.r33,
			A.r20 * B.r00 + A.r21 * B.r10 + A.r22 * B.r20 + A.r23 * B.r30,
			A.r20 * B.r01 + A.r21 * B.r11 + A.r22 * B.r21 + A.r23 * B.r31,
			A.r20 * B.r02 + A.r21 * B.r12 + A.r22 * B.r22 + A.r23 * B.r32,
			A.r20 * B.r03 + A.r21 * B.r13 + A.r22 * B.r23 + A.r23 * B.r33,
			A.r30 * B.r00 + A.r31 * B.r10 + A.r32 * B.r20 + A.r33 * B.r30,
			A.r30 * B.r01 + A.r31 * B.r11 + A.r32 * B.r21 + A.r33 * B.r31,
			A.r30 * B.r02 + A.r31 * B.r12 + A.r32 * B.r22 + A.r33 * B.r32,
			A.r30 * B.r03 + A.r31 * B.r13 + A.r32 * B.r23 + A.r33 * B.r33
		};
	}
	Vector4f Multiply(Matrix4f A, Vector4f v) {
		return Vector4f{
			A.r00 * v.x + A.r01 * v.y + A.r02 * v.z + A.r03 * v.w,
			A.r10 * v.x + A.r11 * v.y + A.r12 * v.z + A.r13 * v.w,
			A.r20 * v.x + A.r21 * v.y + A.r22 * v.z + A.r23 * v.w,
			A.r30 * v.x + A.r31 * v.y + A.r32 * v.z + A.r33 * v.w
		};
	}

	Vector3f getEulerAngle_xyz(Matrix4f mat) {
		float a, b, c;
		if (mat.r02 < 1) {
			if (mat.r02 > -1) {
				a = toDeg(atan2f(-mat.r12, mat.r22));
				b = toDeg(asinf(mat.r02));
				c = toDeg(atan2f(-mat.r01, mat.r00));
			}
			else {
				a = -toDeg(atan2f(mat.r10, mat.r11));
				b = -90.0f;
				c = 0.0f;
			}
		}
		else {
			a = toDeg(atan2f(mat.r10, mat.r11));
			b = 90.0f;
			c = 0.0f;
		}
		return Vector3f{ a, b, c };
	}
	Vector3f getEulerAngle_zyx(Matrix4f mat) {
		float a, b, c;
		if (mat.r20 < 1) {
			if (mat.r20 > -1) {
				a = toDeg(atan2f(mat.r10, mat.r00));
				b = toDeg(asinf(-mat.r20));
				c = toDeg(atan2f(mat.r21, mat.r22));
			}
			else {
				a = -toDeg(atan2f(-mat.r12, mat.r11));
				b = 90.0f;
				c = 0.0f;
			}
		}
		else {
			a = toDeg(atan2f(-mat.r12, mat.r11));
			b = -90.0f;
			c = 0.0f;
		}
		return Vector3f{ a,b,c };
	}
	Vector3f getEulerAngle_zyz(Matrix4f mat) {
		float a, b, c;
		if (mat.r22 < 1) {
			if (mat.r22 > -1) {
				a = toDeg(atan2f(mat.r12, mat.r02));
				b = toDeg(acosf(mat.r22));
				c = toDeg(atan2f(mat.r21, -mat.r20));
			}
			else {
				a = -toDeg(atan2f(mat.r10, mat.r11));
				b = 180.0f;
				c = 0.0f;
			}
		}
		else {
			a = toDeg(atan2f(mat.r10, mat.r11));
			b = 0.0f;
			c = 0.0f;
		}
		return Vector3f{ a,b,c };
	}

	Matrix4f Transpose(Matrix4f mat) {
		return Matrix4f{
			mat.r00,mat.r10,mat.r20,mat.r30,
			mat.r01,mat.r11,mat.r21,mat.r31,
			mat.r02,mat.r12,mat.r22,mat.r32,
			mat.r03,mat.r13,mat.r23,mat.r33
		};
	}
	float Determinant(Matrix3f mat)
	{
		return mat.r00 * mat.r11 * mat.r22 + mat.r10 * mat.r21 * mat.r02 + mat.r20 * mat.r12 * mat.r01 - mat.r02 * mat.r11 * mat.r20 - mat.r01 * mat.r10 * mat.r22 - mat.r00 * mat.r21 * mat.r12;
	}
	float Determinant(Matrix4f mat) {
		float pVal1 = (mat.r00 * mat.r11 * mat.r22 * mat.r33) + (mat.r00 * mat.r12 * mat.r23 * mat.r31) + (mat.r00 * mat.r13 * mat.r21 * mat.r32) + (mat.r01 * mat.r10 * mat.r23 * mat.r32) + (mat.r01 * mat.r12 * mat.r20 * mat.r33) + (mat.r01 * mat.r13 * mat.r22 * mat.r30);
		float pVal2 = (mat.r02 * mat.r10 * mat.r21 * mat.r33) + (mat.r02 * mat.r11 * mat.r23 * mat.r30) + (mat.r02 * mat.r13 * mat.r20 * mat.r31) + (mat.r03 * mat.r10 * mat.r22 * mat.r31) + (mat.r03 * mat.r11 * mat.r20 * mat.r32) + (mat.r03 * mat.r12 * mat.r21 * mat.r30);
		float nVal1 = (mat.r00 * mat.r11 * mat.r23 * mat.r32) + (mat.r00 * mat.r12 * mat.r21 * mat.r33) + (mat.r00 * mat.r13 * mat.r22 * mat.r31) + (mat.r01 * mat.r10 * mat.r22 * mat.r33) + (mat.r01 * mat.r12 * mat.r23 * mat.r30) + (mat.r01 * mat.r13 * mat.r20 * mat.r32);
		float nVal2 = (mat.r02 * mat.r10 * mat.r23 * mat.r31) + (mat.r02 * mat.r11 * mat.r20 * mat.r33) + (mat.r02 * mat.r13 * mat.r21 * mat.r30) + (mat.r03 * mat.r10 * mat.r21 * mat.r32) + (mat.r03 * mat.r11 * mat.r22 * mat.r30) + (mat.r03 * mat.r12 * mat.r20 * mat.r31);
		return pVal1 + pVal2 - nVal1 - nVal2;
	}
	Matrix4f Inverse(Matrix4f mat)
	{
		float det = Determinant(mat);
		Matrix4f m4f = Matrix4f();
		if (fabsf(det) < 1E-2f)
		{
			return m4f;
		}
		else
		{
			m4f.r00 = Determinant(Matrix3f{ mat.r11, mat.r12, mat.r13, mat.r21, mat.r22, mat.r23, mat.r31, mat.r32, mat.r33 }) / det;
			m4f.r01 = Determinant(Matrix3f{ mat.r10, mat.r12, mat.r13, mat.r20, mat.r22, mat.r23, mat.r30, mat.r32, mat.r33 }) * -1.0f / det;
			m4f.r02 = Determinant(Matrix3f{ mat.r10, mat.r11, mat.r13, mat.r20, mat.r21, mat.r23, mat.r30, mat.r31, mat.r33 }) / det;
			m4f.r03 = Determinant(Matrix3f{ mat.r10, mat.r11, mat.r12, mat.r20, mat.r21, mat.r22, mat.r30, mat.r31, mat.r32 }) * -1.0f / det;

			m4f.r10 = Determinant(Matrix3f{ mat.r01, mat.r02, mat.r03, mat.r21, mat.r22, mat.r23, mat.r31, mat.r32, mat.r33 }) * -1.0f / det;
			m4f.r11 = Determinant(Matrix3f{ mat.r00, mat.r02, mat.r03, mat.r20, mat.r22, mat.r23, mat.r30, mat.r32, mat.r33 }) / det;
			m4f.r12 = Determinant(Matrix3f{ mat.r00, mat.r01, mat.r03, mat.r20, mat.r21, mat.r23, mat.r30, mat.r31, mat.r33 }) * -1.0f / det;
			m4f.r13 = Determinant(Matrix3f{ mat.r00, mat.r01, mat.r02, mat.r20, mat.r21, mat.r22, mat.r30, mat.r31, mat.r32 }) / det;

			m4f.r20 = Determinant(Matrix3f{ mat.r01, mat.r02, mat.r03, mat.r11, mat.r12, mat.r13, mat.r31, mat.r32, mat.r33 }) / det;
			m4f.r21 = Determinant(Matrix3f{ mat.r00, mat.r02, mat.r03, mat.r10, mat.r12, mat.r13, mat.r30, mat.r32, mat.r33 }) * -1.0f / det;
			m4f.r22 = Determinant(Matrix3f{ mat.r00, mat.r01, mat.r03, mat.r10, mat.r11, mat.r13, mat.r30, mat.r31, mat.r33 }) / det;
			m4f.r23 = Determinant(Matrix3f{ mat.r00, mat.r01, mat.r02, mat.r10, mat.r11, mat.r12, mat.r30, mat.r31, mat.r32 }) * -1.0f / det;

			m4f.r30 = Determinant(Matrix3f{ mat.r01, mat.r02, mat.r03, mat.r11, mat.r12, mat.r13, mat.r21, mat.r22, mat.r23 }) * -1.0f / det;
			m4f.r31 = Determinant(Matrix3f{ mat.r00, mat.r02, mat.r03, mat.r10, mat.r12, mat.r13, mat.r20, mat.r22, mat.r23 }) / det;
			m4f.r32 = Determinant(Matrix3f{ mat.r00, mat.r01, mat.r03, mat.r10, mat.r11, mat.r13, mat.r20, mat.r21, mat.r23 }) * -1.0f / det;
			m4f.r33 = Determinant(Matrix3f{ mat.r00, mat.r01, mat.r02, mat.r10, mat.r11, mat.r12, mat.r20, mat.r21, mat.r22 }) / det;

			return Transpose(m4f);
		}
	}
}
