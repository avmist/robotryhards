	#include "Vec2.cpp"

Vec2::Vec2(double x, double y) {
	this.x = x;
	this.y = y;
}

Vec2 Vec2::Vec2.operator+ (Vec2 b) {
	return Vec2(this.x + b.x, this.y + b.y);
}

Vec2 Vec2::Vec2.operator- (Vec2 b) {
	return Vec2(this.x - b.x, this.y - b.y);
}

float Vec2::Vec2.dot(Vec2 b) {
	return this.x * b.x + this.y * b.y;
}

float Vec2::Vec2.cross(Vec2 b) {
	return this.x * b.y - this.y * b.x;
}

Vec2 Vec2::Vec2.operator* (float c) {
	return Vec2(this.x * c, this.y * c);
}

Vec2 Vec2::Vec2.operator/ (float c) {
	return Vec2(this.x / c, this.y / c);
}





float Vec2::Vec2.size() {
	return sqrt(this.x*this.x + this.y*this.y);
}

float Vec2::Vec2.angle() {
	return atan2(this.y, this.x);
}

Vec2 Vec2::Vec2.left(){
	return Vec2(-this.y, this.x);
}

Vec2 Vec2::Vec2.right(){
	return Vec2(this.y, -this.x);
}

Vec2::Vec2 Vec2.unit(){
	float len = this.size();
	return Vec2(this.x / len, this.y / len);
}

static Vec2 intersect(Vec2 p, Vec2 r, Vec2 q, Vec2 s) 
	float rs = r.cross(s);			//local rs = r:cross(s)
	Vec2 qp = q-p;					//local qp = q-p
	return p + r * qp.cross(s)/rs	//p + r * qp:cross(s)/rs
}