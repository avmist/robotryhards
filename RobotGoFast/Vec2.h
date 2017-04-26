#ifndef VEC2_H
#define VEC2_H

#include "math.h"

class Vec2 {

public:
  double x;
  double y;

  // Constructors
  Vec2(double x, double y);

  // Methods
  Vec2 operator+ (Vec2 b);	//add 2 vectors
  Vec2 operator- (Vec2 b);	//subtract 2 vectors, (vector from b to this)
  float dot(Vec2 b);		//dot product
  float cross(Vec2 b);		//2D cross product (area of a paralellogram formed by the two vectors)
  
  Vec2 operator* (float c);	//scale by c
  Vec2 operator/ (float c);	//scale by 1/c
  
  float size();	//length of vector
  float angle();	//direction of vector, radians
  Vec2 left();		//rotate left a quarter turn
  Vec2 right();	//rotate right a quarter turn
  Vec2 unit();		//vector which points in the same direction but has a magnitude of 1
  
  static Vec2 intersect(Vec2 p, Vec2 r, Vec2 q, Vec2 s);

};

#endif