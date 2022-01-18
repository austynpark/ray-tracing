#ifndef RAY_H
#define RAY_H

#include "geom.h"

class Shape;

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

} Intersection;

class Shape {
public:
	virtual bool intersect(Ray ray, Intersection& intersection) = 0;
};

class Interval {
public:
	Interval();
	~Interval();

	Interval(vec3 t0, vec3 t1, vec3 N0, vec3 N1);

	void empty();



};


#endif // !RAY_H
