#ifndef RAY_H
#define RAY_H

#include "geom.h"

constexpr float EPSILON = 0.0001f;
const float PI = 3.14159f;
const float Radians = PI/180.0f;    // Convert degrees to radians


class Shape;
class Material;

/*
* @param Q	Starting Point
* @param D	Direction - should be scaled to a unit length vector
*/
typedef struct Ray {
	vec3 Q;
	vec3 D;

	__forceinline vec3 eval(float t) const {
		return Q + t * D;
	};
} Ray;

/*
* Storage of a ray intersection with any Shape object
* 
* @param t		Parameter value on ray of the point of intersection
* @param object A pointer to the Shape intersected
* @param P		Point of intersection (in world coordinates)
* @param N		Normal of surface at interesection point (in world coordinates)
*/
typedef struct Intersection {

	float t;
	Shape* object;
	vec3 P;
	vec3 N;
	// other 2D or 3D texture coord

	float distance() const { return t; }
	float distribution(const vec3& H) const;

	float Gfactor(const vec3& input_dir, const vec3& output_dir, const vec3& m) const {
		return G1(input_dir, m) * G1(output_dir, m);
	}

	float G1(const vec3& v, const vec3& m) const;
	// Fresnel (reflection)
	vec3 Ffactor(float d) const;


} Intersection;

class Shape {
public:
	virtual bool intersect(const Ray& ray, Intersection& intersection) = 0;
	virtual void bounding_box(vec3& out_min, vec3& out_max) = 0;
	virtual ~Shape() {};

	Material* material;
	vec3 position;
	float area;
};

#endif // !RAY_H
