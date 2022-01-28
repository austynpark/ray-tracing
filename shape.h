#ifndef SHAPE_H
#define SHAPE_H

#include "ray_util.inl"

class Material;

// Infinite volume bounded by two parallel planes represented by two plane-
// equations N, d0 and N, d1 sharing a normal Intersection a ray with a collection of n slabs
typedef struct Slab{
	vec3 N;
	float d0;
	float d1;
	
} Slab;


typedef struct Interval {
	void initialize(float t0 = 0.0f, float t1 = FLT_MAX, vec3 N0 = vec3(0), vec3 N1 = vec3(0));

	void empty();

	void intersect(const Interval* other);
	// single slab intersect with ray
	bool intersect(const Ray& ray, const Slab* slab);
	// multiple slab intersect with ray
	bool intersect(const Ray& ray, Slab slab[], uint32_t slabs_count);

	float t0;
	vec3 N0;

	float t1;
	vec3 N1;

} Interval;

class Sphere : public Shape {
public:
	Sphere(vec3 center, float radius, Material* mat);

	bool intersect(const Ray& ray, Intersection& intersection) override;
private:
	vec3 C;		// center point
	float r;	// radius

};

class Box : public Shape {
public:
	Box(vec3 corner, vec3 diagonal, Material* mat);

	bool intersect(const Ray& ray, Intersection& intersection) override;
private:
	Slab slabs[3];
	vec3 corner;
	vec3 diag;
};

class Cylinder : public Shape {
public:
	Cylinder(vec3 base, vec3 axis, float radius, Material* mat);

	bool intersect(const Ray& ray, Intersection& intersection) override;
private:
	vec3 B;		// Base
	vec3 A;		// Axis
	float r;	// radius
	Slab slab;
};

class Triangle : public Shape {
public:
	Triangle(vec3 V0, vec3 V1, vec3 V2, vec3 N0, vec3 N1, vec3 N2, Material* mat);

	bool intersect(const Ray& ray, Intersection& intersection) override;
private:
	vec3 V0;
	vec3 V1;
	vec3 V2;

	vec3 N0;
	vec3 N1;
	vec3 N2;
};

#endif // !SHAPE_H
