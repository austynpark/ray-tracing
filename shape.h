#ifndef SHAPE_H
#define SHAPE_H

#include "ray_util.inl"

class Sphere : public Shape {
public:
	bool intersect(Ray ray, Intersection& intersection) override;


};

class Box : public Shape {
public:
	bool intersect(Ray ray, Intersection& intersection) override;


};

class Cylinder : public Shape {
public:
	bool intersect(Ray ray, Intersection& intersection) override;


};

class Triangle : public Shape {
public:
	bool intersect(Ray ray, Intersection& intersection) override;


};

#endif // !SHAPE_H
