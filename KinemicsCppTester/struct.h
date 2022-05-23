#pragma once

struct DH_param
{
	float alpha;
	float a;
	float d;
	float theta;
	DH_param(float _alpha=0, float _a=0, float _d=0, float _theta=0) {
		this->alpha = _alpha; this->a = _a; this->d = _d; this->theta = _theta;
	}
};

struct DH_table
{
	int size;
	DH_param* params;
};

struct PostureCfg {
	bool Right;
	bool ElbowUp;
	bool Flip;
	PostureCfg(bool right=true, bool elbowUp=true, bool flip=false) {
		this->Right = right; this->ElbowUp = elbowUp; this->Flip = flip;
	}
};

struct Vector3f {
	float x; float y; float z;
};

struct Vector4f {
	float x; float y; float z; float w;
};

struct Matrix3f {
	float r00; float r01; float r02;
	float r10; float r11; float r12;
	float r20; float r21; float r22;
};

struct Matrix4f {
	float r00; float r01; float r02; float r03;
	float r10; float r11; float r12; float r13;
	float r20; float r21; float r22; float r23;
	float r30; float r31; float r32; float r33;
};

struct WorldCoordinate {
	float x;
	float y;
	float z;
	float a;
	float b;
	float c;
};

struct JointCoordinate {
	float j1;
	float j2;
	float j3;
	float j4;
	float j5;
	float j6;
};