#ifndef RAY_H
#define RAY_H

#include "geom.h"

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

} Intersection;

class Shape {
public:
	virtual bool intersect(const Ray& ray, Intersection& intersection) = 0;
	virtual void bounding_box(vec3& out_min, vec3& out_max) = 0;
	Material* material;

};

#endif // !RAY_H
