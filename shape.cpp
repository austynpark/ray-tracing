#include "shape.h"
#include "geom.h"

#include <cmath>

Sphere::Sphere(vec3 center, float radius, Material* mat) : Shape(), C(center), r(radius)
{
	material = mat;
}

bool Sphere::intersect(const Ray& ray, Intersection& out_intersection)
{
	vec3 center_to_origin = ray.Q - C;
	
	const float b = dot(center_to_origin, ray.D);
	// since a = dot(D, D) = 1
	const float discriminant_sqrt = sqrtf(b * b - dot(center_to_origin, center_to_origin) + r * r);

	if (-b - discriminant_sqrt > 0) {
		out_intersection.t = -b - discriminant_sqrt;
	}
	else if (-b + discriminant_sqrt > 0) {
		out_intersection.t = -b + discriminant_sqrt;
	}
	else {
		return false;
	}

	out_intersection.P = ray.eval(out_intersection.t);
	out_intersection.N = normalize(out_intersection.P - C);
	out_intersection.object = this;

	return true;
}

void Sphere::bounding_box(vec3& out_min, vec3& out_max)
{
	out_min = C - r;
	out_max = C + r;
}

Box::Box(vec3 corner, vec3 diagonal, Material* mat) : Shape()
{
	material = mat;
	this->corner = corner;
	this->diag = diagonal;

	slabs[0].N = vec3(1, 0, 0);
	slabs[0].d0 = -corner.x;
	slabs[0].d1 = -(corner.x + diagonal.x);

	slabs[1].N = vec3(0, 1, 0);
	slabs[1].d0 = -corner.y;
	slabs[1].d1 = -(corner.y + diagonal.y);

	slabs[2].N = vec3(0, 0, 1);
	slabs[2].d0 = -corner.z;
	slabs[2].d1 = -(corner.z + diagonal.z);

}

bool Box::intersect(const Ray& ray, Intersection& intersection)
{
	Interval out_interval;
	out_interval.intersect(ray, slabs, 3);

	if (out_interval.t0 > out_interval.t1) {
		return false;
	}
	if (out_interval.t0 > 0) {
		intersection.t = out_interval.t0;
		intersection.N = normalize(out_interval.N0);
	}
	else if (out_interval.t1 > 0) {
		intersection.t = out_interval.t1;
		intersection.N = normalize(out_interval.N1);
	}
	else {
		return false;
	}

	intersection.P = ray.eval(intersection.t);
	intersection.object = this;

	return true;
}

void Box::bounding_box(vec3& out_min, vec3& out_max)
{
	out_min = corner;
	out_max = corner + diag;
}

Cylinder::Cylinder(vec3 base, vec3 axis, float radius, Material* mat) : Shape(), B(base), A(axis), r(radius)
{
	slab.N = vec3(0, 0, 1);
	slab.d0 = 0.0f;
	slab.d1 = -length(A);

 	material = mat;
}

bool Cylinder::intersect(const Ray& ray, Intersection& intersection)
{
	Ray ray_;

	mat3 R = rotate_to_zaxis(A);

	ray_.Q = R * (ray.Q - B);
	ray_.D = R * ray.D;

	Interval first_interval;

	first_interval.initialize();

	if (!first_interval.intersect(ray_, &slab)) {
		return false;
	}

	// Second Interval
	float a = dot(ray_.D.x, ray_.D.x) + dot(ray_.D.y, ray_.D.y);
	float b = 2 * (dot(ray_.D.x, ray_.Q.x) + dot(ray_.D.y, ray_.Q.y));
	float c = dot(ray_.Q.x, ray_.Q.x) + dot(ray_.Q.y, ray_.Q.y) - r * r;

	float det = (b * b) - (4 * a * c);

	if (det < 0) {
		return false;
	}

	float b0 = (-b - sqrtf(det)) / (2 * a);
	float b1 = (-b + sqrtf(det)) / (2 * a);

	float t0 = first_interval.t0 > b0 ? first_interval.t0 : b0;
	vec3 N0 = first_interval.t0 > b0 ? first_interval.N0 : vec3(ray_.Q.x + ray_.D.x * t0, ray_.Q.y + ray_.D.y * t0, 0);

	float t1 = first_interval.t1 < b1 ? first_interval.t1 : b1;
	vec3 N1 = first_interval.t1 < b1 ? first_interval.N1 : vec3(ray_.Q.x + ray_.D.x * t1, ray_.Q.y + ray_.D.y * t1, 0);


	// No Intersection, the "off the corner" case
	if (t0 > t1) {
		return false;
	}

	if (t0 > 0) {
		intersection.t = t0;
		intersection.N = normalize(glm::transpose(R) * N0);

	}
	else if (t1 > 0) {
		intersection.t = t1;
		intersection.N = normalize(glm::transpose(R) * N1);
	}
	else {
		return false;
	}

	intersection.P = ray.eval(intersection.t);
	intersection.object = this;

	return true;
}

void Cylinder::bounding_box(vec3& out_min, vec3& out_max)
{
	vec3 p0 = B + r;
	vec3 p1 = B - r;

	vec3 p2 = A + p0;
	vec3 p3 = A + p1;

	for (int i = 0; i < 3; ++i) {
		out_max[i] = fmaxf(fmaxf(p0[i], p1[i]), fmaxf(p2[i], p3[i]));
		out_min[i] = fminf(fminf(p0[i], p1[i]), fminf(p2[i], p3[i]));
	}
}

Triangle::Triangle(vec3 V0, vec3 V1, vec3 V2, vec3 N0, vec3 N1, vec3 N2, Material* mat) : Shape()
{
	material = mat;

	this->V0 = V0;
	this->V1 = V1;
	this->V2 = V2;

	this->N0 = N0;
	this->N1 = N1;
	this->N2 = N2;
}

bool Triangle::intersect(const Ray& ray, Intersection& intersection)
{
	vec3 E1 = V1 - V0;
	vec3 E2 = V2 - V0;
	vec3 p = cross(ray.D, E2);
	float d = dot(p, E1);

	// NO INTERSECTION (RAY IS PARALLEL TO TRIANGLE)
	if (d == 0) {
		return false;
	}

	vec3 S = ray.Q - V0;
	float u = dot(p, S) / d;

	// RAY INTERSECTS PLANE, BUT OUTSIDE E2 edge
	if((u < 0) || (u > 1)){
		return false;
	}

	vec3 q = cross(S, E1);
	float v = dot(ray.D, q) / d;

	// RAY INTERSECTS PLANE, BUT OUTSIDE OTHER EDGES
	if ((v < 0) || (u + v) > 1) {
		return false;
	}

	float t = dot(E2, q) / d;

	// RAY'S NEGATIVE HALF INTERSECTS TRIANGLE
	if(t < 0) {
		return false;
	}

	intersection.t = t;
	intersection.P = ray.eval(t);

	//TODO: If vertex normals are known: (1 - u - v)N0 + uN1 + vN2 and else
	intersection.N = normalize(cross(E2, E1));
	intersection.object = this;

	return true;
}

void Triangle::bounding_box(vec3& out_min, vec3& out_max)
{
	for (int i = 0; i < 3; ++i) {
		out_min[i] = fminf(fminf(V0[i], V1[i]), V2[i]);
		out_max[i] = fmaxf(fmaxf(V0[i], V1[i]), V2[i]);
	}
}

void Interval::initialize(float t0, float t1, vec3 N0, vec3 N1)
{
	this->t0 = t0;
	this->t1 = t1;

	this->N0 = N0;
	this->N1 = N1;
}

void Interval::empty()
{
	this->t0 = 0;
	this->t1 = -1;
}

void Interval::intersect(const Interval* other)
{
	if (t0 < other->t0) {
		t0 = other->t0;
		N0 = other->N0;
	}
	
	if (t1 > other->t1) {
		t1 = other->t1;
		N1 = other->N1;
	}
}

bool Interval::intersect(const Ray& ray, const Slab* slab)
{
	assert(slab, "slab is nullptr");

	float n_dot_d = dot(slab->N, ray.D);
	float n_dot_q = dot(slab->N, ray.Q);


	//TODO: comparing float with 0.0f? maybe with epsilon
	// ray parallel to slab planes
	if (n_dot_d == 0.0f) {

		float s0 = n_dot_q + slab->d0;
		float s1 = n_dot_q + slab->d1;

		if (s0 * s1 < 0.0f) {
			// ray.Q between planes
			//initialize()
			t0 = 0.0f;
			t1 = FLT_MAX;
		}
		else {
			// ray.Q outside planes
			//empty()
			t0 = 1.0f;
			t1 = 0.0f;
		}

		return false;
	}

	// plane 0
	t0 = -(slab->d0 + n_dot_q) / n_dot_d;

	// plane 1
	t1 = -(slab->d1 + n_dot_q) / n_dot_d;

	if (t0 > t1) {
		float temp = t0;
		t0 = t1;
		t1 = temp;
	}

	if (n_dot_d > 0) {
		N0 = -slab->N;
		N1 = slab->N;
	}
	else {
		N0 = slab->N;
		N1 = -slab->N;
	}

	return true;

}

bool Interval::intersect(const Ray& ray, Slab slab[], uint32_t slabs_count)
{
	initialize();
	
	assert(slab, "slab is nullptr");

	float t0_ = t0;
	vec3 N0_ = N0;
	float t1_ = t1;
	vec3 N1_ = N1;



	for (uint32_t i = 0; i < slabs_count; ++i) {

		if (!intersect(ray, &slab[i]))
			continue;

		if (t0_ < t0) {
			t0_ = t0;
			N0_ = N0;
		}

		if (t1_ > t1) {
			t1_ = t1;
			N1_ = N1;
		}
	}


	t0 = t0_;
	t1 = t1_;
	N0 = N0_;
	N1 = N1_;

	//TODO: This code might be worthless since intersect function returns false if t0 > t1
	if (t0 > t1) {
		return false;
	}

	return true;
}
