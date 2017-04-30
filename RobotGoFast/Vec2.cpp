#include "Vec2.h"

Vec2::Vec2(double x, double y) {
	this->x = x;
	this->y = y;
}

Vec2::Vec2() {
	this->x = 1;
	this->y = 0;
}

Vec2 Vec2::operator+ (Vec2 b) {
	return Vec2(this->x + b.x, this->y + b.y);
}

Vec2 Vec2::operator- (Vec2 b) {
	return Vec2(this->x - b.x, this->y - b.y);
}

float Vec2::dot(Vec2 b) {
	return this->x * b.x + this->y * b.y;
}

float Vec2::cross(Vec2 b) {
	return this->x * b.y - this->y * b.x;
}

float Vec2::angleTo(Vec2 b) {

	//return acos(b.unit().dot(unit()));

	// http://stackoverflow.com/questions/21483999/using-atan2-to-find-angle-between-two-vectors
	// https://gamedev.stackexchange.com/questions/69649/using-atan2-to-calculate-angle-between-two-vectors
	//return atan2(b.y, b.x) - atan2(y, x);

	// Clockwise
	// http://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors
	return atan2(x*b.y - y*b.x, x*b.x + y*b.y);
}

Vec2 Vec2::operator* (float c) {
	return Vec2(this->x * c, this->y * c);
}

Vec2 Vec2::operator/ (float c) {
	return Vec2(this->x / c, this->y / c);
}

float Vec2::size() {
	return sqrt(this->x*this->x + this->y*this->y);
}

float Vec2::angle() {
	return atan2(this->y, this->x);
}

Vec2 Vec2::left(){
	return Vec2(-this->y, this->x);
}

Vec2 Vec2::right(){
	return Vec2(this->y, -this->x);
}

Vec2 Vec2::unit(){
	float len = this->size();
	return Vec2(this->x / len, this->y / len);
}

Vec2 Vec2::intersect(Vec2 p, Vec2 r, Vec2 q, Vec2 s) { 
	float rs = r.cross(s);			//local rs = r:cross(s)
	Vec2 qp = q-p;					//local qp = q-p
	return p + (r * qp.cross(s) / rs);	//p + r * qp:cross(s)/rs
}

Vec2 Vec2::fromPolar(double r, double a) {
	return Vec2(r*cos(a), r*sin(a));
}

Vec2 Vec2::fromPolarDeg(double r, double d) {
	return Vec2(r*cos(d / 180.0*PI), r*sin(d / 180.0*PI));
}