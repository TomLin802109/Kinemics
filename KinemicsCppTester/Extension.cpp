#include "Extension.h"

float toRad(float degree) {
	return degree / 180 * 3.14159265358979323846;
}

float toDeg(float rad) {
	return rad / 3.14159265358979323846 * 180;
}

double toRad(double degree) {
	return degree / 180 * 3.14159265358979323846;
}

double toDeg(double rad) {
	return rad / 3.14159265358979323846 * 180;
}

float round(float num, int index) {
	auto multiplier = pow(10, index);
	return round(num * multiplier) / multiplier;
};

double round(double num, int index) {
	auto multiplier = pow(10, index);
	return round(num * multiplier) / multiplier;
};