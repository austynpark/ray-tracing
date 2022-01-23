#ifndef CAMERA_H
#define CAMERA_H


#include "geom.h"

typedef struct Camera {
	vec3 E; // eye position
	quat Q; // orientation as a quaternion
	float ry; // frustum's Y versus Z slope
} Camera;


#endif // !CAMERA_H

